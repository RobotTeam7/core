#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <common/robot_motor.h>
#include <common/pin.h>
#include <string.h>
#include <FreeRTOS.h>
#include <task.h>

// tape following imports
#include <tape/task_poll_reflectance.h>
#include <tape/reflectance_polling_config.h>
#include <tape/tape_following_config.h>
#include <tape/task_follow_tape.h>

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
#define PRIORITY_FOLLOW_TAPE 1

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET  -1 // This display does not have a reset pin accessible

RobotMotor motor_front_left;
RobotMotor motor_front_right;
RobotMotor motor_back_left;
RobotMotor motor_back_right;
ReflectancePollingConfig config_reflectance;
TapeFollowingConfig config_following;
CircularBuffer<int, BUFFER_SIZE> leftBuffer;
CircularBuffer<int, BUFFER_SIZE> rightBuffer;


void setup() {
  Serial.begin(9600);
  Serial.println("Got serial!");

  motor_front_left = RobotMotor(MOTOR_FRONT_LEFT_FORWARD, MOTOR_FRONT_LEFT_REVERSE);
  motor_front_right = RobotMotor(MOTOR_FRONT_RIGHT_FORWARD, MOTOR_FRONT_RIGHT_REVERSE);
  motor_back_left = RobotMotor(MOTOR_BACK_LEFT_FORWARD, MOTOR_BACK_LEFT_REVERSE);
  motor_back_right = RobotMotor(MOTOR_BACK_RIGHT_FORWARD, MOTOR_BACK_RIGHT_REVERSE);

  config_reflectance.left_sensor_buffer = &leftBuffer;
  config_reflectance.right_sensor_buffer = &rightBuffer;
  BaseType_t xReturnedReflectance = xTaskCreate(TaskPollReflectance, "Reflectance Polling", 50, &config_reflectance, PRIORITY_REFLECTANCE_POLLING, NULL);

  config_following.left_sensor_buffer = &leftBuffer;
  config_following.right_sensor_buffer = &rightBuffer;
  config_following.motor_back_left = &motor_back_left;
  config_following.motor_back_right = &motor_back_right;
  config_following.motor_front_left = &motor_front_left;
  config_following.motor_front_right = &motor_front_right;
  BaseType_t xReturnedFollowing = xTaskCreate(TaskFollowTape, "Tape Following", 100, &config_following, PRIORITY_FOLLOW_TAPE, NULL);


  // check if reflectance polling task was created
  if (xReturnedReflectance == pdPASS) {
    Serial.println("Reflectance polling task was created successfully.");
  } else
  {
    Serial.println("Reflectance polling task was not created successfully!");
  }

  // check if tape following task was created
  if(xReturnedFollowing == pdPASS) {
    Serial.println("Tape following task was created successfully.");
  } else
  {
    Serial.println("Tape following task was not created successfully!");
  }


  // size_t freeHeap = xPortGetFreeHeapSize();
  // Serial.println("Free Heap: " + String(freeHeap));

  vTaskStartScheduler();
}


void loop() {
  // motor_front_left.set_drive(SPEED, forward);
  // motor_front_right.set_drive(SPEED, forward);
  // motor_back_left.set_drive(SPEED, forward);
  // motor_back_right.set_drive(SPEED, forward);
  // delay(TIME_DELAY_MOTOR);
  // motor_front_left.stop();
  // motor_front_right.stop();
  // motor_back_left.stop();
  // motor_back_right.stop();
  // delay(1000);

  // motor_front_left.set_drive(SPEED, forward);
  // motor_front_right.set_drive(SPEED, reverse);
  // motor_back_left.set_drive(SPEED, reverse);
  // motor_back_right.set_drive(SPEED, forward);
  // delay(TIME_DELAY_MOTOR * 2);
  // motor_front_left.stop();
  // motor_front_right.stop();
  // motor_back_left.stop();
  // motor_back_right.stop();
  // delay(1000);

  // motor_front_left.set_drive(SPEED, reverse);
  // motor_front_right.set_drive(SPEED, reverse);
  // motor_back_left.set_drive(SPEED, reverse);
  // motor_back_right.set_drive(SPEED, reverse);
  // delay(TIME_DELAY_MOTOR);
  // motor_front_left.stop();
  // motor_front_right.stop();
  // motor_back_left.stop();
  // motor_back_right.stop();
  // delay(1000);

  // motor_front_left.set_drive(SPEED, reverse);
  // motor_front_right.set_drive(SPEED, forward);
  // motor_back_left.set_drive(SPEED, forward);
  // motor_back_right.set_drive(SPEED, reverse);
  // delay(TIME_DELAY_MOTOR * 2);
  // motor_front_left.stop();
  // motor_front_right.stop();
  // motor_back_left.stop();
  // motor_back_right.stop();
  // delay(1000);
  }