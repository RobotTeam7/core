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

// RobotMotor motor_1;
// RobotMotor motor_2;
// RobotMotor motor_3;
// RobotMotor motor_4;

void setup() {
  RobotMotor motor_1 = RobotMotor(MOTOR_1_FORWARD_PIN, MOTOR_1_REVERSE_PIN);
  motor_1.set_drive(10000, forward);
}

void loop() {
  // motor_1.set_drive(10000, forward);
  // motor_2.set_drive(32000, forward);
  // motor_3.set_drive(32000, reverse);
  // motor_4.set_drive(32000, reverse);
  // delay(5000);
  // motor_1.stop();
  // motor_2.stop();
  // motor_3.stop();
  // motor_4.stop();
  // delay(5000);
  // motor_1.set_drive(32000, reverse);
  // motor_2.set_drive(32000, reverse);
  // motor_3.set_drive(32000, forward);
  // motor_4.set_drive(32000, forward);
  // delay(5000);
  // motor_1.stop();
  // motor_2.stop();
  // motor_3.stop();
  // motor_4.stop();
  // delay(5000);
}