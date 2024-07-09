#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <common/robot_motor.h>
#include <common/pin.h>
#include <Adafruit_SSD1306.h>
#include <string.h>
#include <FreeRTOS.h>
#include <task.h>

#define MOTOR_1_FORWARD_PIN PA1
#define MOTOR_1_REVERSE_PIN PA0
#define MOTOR_2_FORWARD_PIN PA3
#define MOTOR_2_REVERSE_PIN PA2
#define MOTOR_3_FORWARD_PIN PA7
#define MOTOR_3_REVERSE_PIN PA6
#define MOTOR_4_FORWARD_PIN PB1
#define MOTOR_4_REVERSE_PIN PB0

#include <tape/task_poll_reflectance.h>
#include <tape/reflectance_polling_config.h>
ReflectancePollingConfig config;
CircularBuffer<int, BUFFER_SIZE> leftBuffer;
CircularBuffer<int, BUFFER_SIZE> rightBuffer;

#define PRIORITY_REFLECTANCE_POLLING 3
#define PRIORITY_MOTOR_ADJUSTMENT 1





#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET  -1 // This display does not have a reset pin accessible
Adafruit_SSD1306 display_handler(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

RobotMotor motor_1;
RobotMotor motor_2;
RobotMotor motor_3;
RobotMotor motor_4;


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

  motor_1 = RobotMotor(MOTOR_1_FORWARD_PIN, MOTOR_1_REVERSE_PIN);
  motor_2 = RobotMotor(MOTOR_2_FORWARD_PIN, MOTOR_2_REVERSE_PIN);
  motor_3 = RobotMotor(MOTOR_3_FORWARD_PIN, MOTOR_3_REVERSE_PIN);
  motor_4 = RobotMotor(MOTOR_4_FORWARD_PIN, MOTOR_4_REVERSE_PIN);

  // size_t freeHeap = xPortGetFreeHeapSize();
  // Serial.println("Free Heap: " + String(freeHeap));

  vTaskStartScheduler();
}


void loop() {
  // motor_1.set_drive(0, forward);
  // motor_2.set_drive(0, forward);
  // motor_3.set_drive(0, forward);
  // motor_4.set_drive(0, forward);

  // delay(2000);

  // motor_1.set_drive(32000, forward);
  // motor_2.set_drive(32000, forward);
  // motor_3.set_drive(32000, forward);
  // motor_4.set_drive(32000, forward);

  // delay(250);
  // Print the last 5 values in each buffer


  // Add a delay to prevent flooding the serial output
  delay(1000); // Print every second
}