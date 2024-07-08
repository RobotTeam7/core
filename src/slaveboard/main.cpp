#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <common/robot_motor.h>
#include <common/pin.h>


#define MOTOR_1_FORWARD_PIN PA0
#define MOTOR_1_REVERSE_PIN PA1
#define MOTOR_2_FORWARD_PIN PA2
#define MOTOR_2_REVERSE_PIN PA3
#define MOTOR_3_FORWARD_PIN PA6
#define MOTOR_3_REVERSE_PIN PA7
#define MOTOR_4_FORWARD_PIN PB0
#define MOTOR_4_REVERSE_PIN PB1


#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <string.h>
#include <STM32FreeRTOS.h>

// put function declarations here:
#define REFLECTANCE_ONE PA5
#define REFLECTANCE_TWO PA4
int reflectance_right;
int reflectance_left;
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET  -1 // This display does not have a reset pin accessible
Adafruit_SSD1306 display_handler(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);


void TaskPollReflectance(void *pvParameters) {
  (void) pvParameters;
  int threshhold = 150;
  char drive_state[20] = "find tape";
  for(;;) {
    reflectance_right = analogRead(REFLECTANCE_ONE);
    reflectance_left = analogRead(REFLECTANCE_TWO);
    if (reflectance_right - reflectance_left > threshhold) {
      strcpy(drive_state, "---->>");
    } else if (reflectance_left - reflectance_right > threshhold) {
      strcpy(drive_state, "<<----");
    }else {
      strcpy(drive_state, "^^^^^");
    }
    display_handler.clearDisplay();
    display_handler.setCursor(0,0);
    display_handler.println("Sensor Right:");
    display_handler.setCursor(0,10);
    display_handler.println(reflectance_right);
    display_handler.setCursor(0,20);
    display_handler.println("Sensor Left:");
    display_handler.setCursor(0,30);
    display_handler.println(reflectance_left);
    display_handler.setCursor(50, 50);
    display_handler.println(drive_state);
    display_handler.display();
    vTaskDelay(100/ portTICK_PERIOD_MS);
  }
}

void TaskPollDigitalSensor(void *pvParameters) {
  (void) pvParameters;
  pinMode(PB13, INPUT_PULLUP);
  int digital_value;
  for(;;) {
    digital_value = digitalRead(PB13);
    display_handler.clearDisplay();
    display_handler.setCursor(0,0);
    display_handler.println(digital_value);
    display_handler.display();
  }
}

// End Reflectance

void setup() {
  Serial.begin(9600);
  Serial.println("Got serial!");

  // Reflectance Setup
  pinMode(REFLECTANCE_ONE, INPUT);
  pinMode(REFLECTANCE_TWO, INPUT);
  // set up display handler
  display_handler.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display_handler.display();

  delay(2000);
  
  display_handler.clearDisplay();
  display_handler.setTextSize(1);
  display_handler.setTextColor(SSD1306_WHITE);
  display_handler.setCursor(0,0);
  display_handler.println("Hello world!");
  display_handler.display();

  xTaskCreate(TaskPollReflectance, "Poll", 4096, NULL, 1, NULL);
  vTaskStartScheduler();
}

void loop() {
  RobotMotor motor_1 = RobotMotor(MOTOR_1_FORWARD_PIN, MOTOR_1_REVERSE_PIN);
  RobotMotor motor_2 = RobotMotor(MOTOR_2_FORWARD_PIN, MOTOR_2_REVERSE_PIN);
  RobotMotor motor_3 = RobotMotor(MOTOR_3_FORWARD_PIN, MOTOR_3_REVERSE_PIN);
  RobotMotor motor_4 = RobotMotor(MOTOR_4_FORWARD_PIN, MOTOR_4_REVERSE_PIN);

  motor_1.set_drive(100, forward);
  motor_2.set_drive(100, forward);
  motor_3.set_drive(100, forward);
  motor_4.set_drive(100, forward);

  delay(250);

  motor_1.stop();
  motor_2.stop();
  motor_3.stop();
  motor_4.stop();

  delay(5000);
}