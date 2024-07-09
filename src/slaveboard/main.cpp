#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <common/robot_motor.h>
#include <common/pin.h>
#include <Adafruit_SSD1306.h>
#include <string.h>
#include <FreeRTOS.h>
#include <task.h>
#include <tape/task_poll_reflectance.h>
#include <tape/reflectance_polling_config.h>

#define MOTOR_BACK_RIGHT_FORWARD PB1
#define MOTOR_BACK_RIGHT_REVERSE PB0
#define MOTOR_BACK_LEFT_FORWARD PA7
#define MOTOR_BACK_LEFT_REVERSE PA6
#define MOTOR_FRONT_LEFT_FORWARD PA1
#define MOTOR_FRONT_LEFT_REVERSE PA0
#define MOTOR_FRONT_RIGHT_FORWARD PA3
#define MOTOR_FRONT_RIGHT_REVERSE PA2

#define SPEED 16000
#define TIME_DELAY_MOTOR 500

#define PRIORITY_REFLECTANCE_POLLING 3
#define PRIORITY_MOTOR_ADJUSTMENT 1

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET  -1 // This display does not have a reset pin accessible

Adafruit_SSD1306 display_handler(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
RobotMotor motor_front_left;
RobotMotor motor_front_right;
RobotMotor motor_back_left;
RobotMotor motor_back_right;
ReflectancePollingConfig config;
CircularBuffer<int, BUFFER_SIZE> leftBuffer;
CircularBuffer<int, BUFFER_SIZE> rightBuffer;


void setup() {
  Serial.begin(9600);
  Serial.println("Got serial!");

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

  config.left_sensor_buffer = &leftBuffer;
  config.right_sensor_buffer = &rightBuffer;

  BaseType_t xReturned = xTaskCreate(TaskPollReflectance, "Reflectance Polling", 50, &config, PRIORITY_REFLECTANCE_POLLING, NULL);

  if (xReturned == pdPASS)
    {
      // Task was created successfully
      Serial.println("Poll was created successfully.");
  } else
  {
    // Task was not created successfully
    Serial.println("Poll was not created successfully!");
  }

  motor_front_left = RobotMotor(MOTOR_FRONT_LEFT_FORWARD, MOTOR_FRONT_LEFT_REVERSE);
  motor_front_right = RobotMotor(MOTOR_FRONT_RIGHT_FORWARD, MOTOR_FRONT_RIGHT_REVERSE);
  motor_back_left = RobotMotor(MOTOR_BACK_LEFT_FORWARD, MOTOR_BACK_LEFT_REVERSE);
  motor_back_right = RobotMotor(MOTOR_BACK_RIGHT_FORWARD, MOTOR_BACK_RIGHT_REVERSE);


  // size_t freeHeap = xPortGetFreeHeapSize();
  // Serial.println("Free Heap: " + String(freeHeap));

  vTaskStartScheduler();
}


void loop() {
  motor_front_left.set_drive(SPEED, forward);
  motor_front_right.set_drive(SPEED, forward);
  motor_back_left.set_drive(SPEED, forward);
  motor_back_right.set_drive(SPEED, forward);
  delay(TIME_DELAY_MOTOR);
  motor_front_left.stop();
  motor_front_right.stop();
  motor_back_left.stop();
  motor_back_right.stop();
  delay(1000);

  motor_front_left.set_drive(SPEED, forward);
  motor_front_right.set_drive(SPEED, reverse);
  motor_back_left.set_drive(SPEED, reverse);
  motor_back_right.set_drive(SPEED, forward);
  delay(TIME_DELAY_MOTOR * 2);
  motor_front_left.stop();
  motor_front_right.stop();
  motor_back_left.stop();
  motor_back_right.stop();
  delay(1000);

  motor_front_left.set_drive(SPEED, reverse);
  motor_front_right.set_drive(SPEED, reverse);
  motor_back_left.set_drive(SPEED, reverse);
  motor_back_right.set_drive(SPEED, reverse);
  delay(TIME_DELAY_MOTOR);
  motor_front_left.stop();
  motor_front_right.stop();
  motor_back_left.stop();
  motor_back_right.stop();
  delay(1000);

  motor_front_left.set_drive(SPEED, reverse);
  motor_front_right.set_drive(SPEED, forward);
  motor_back_left.set_drive(SPEED, forward);
  motor_back_right.set_drive(SPEED, reverse);
  delay(TIME_DELAY_MOTOR * 2);
  motor_front_left.stop();
  motor_front_right.stop();
  motor_back_left.stop();
  motor_back_right.stop();
  delay(1000);
  }