#ifdef SLAVE_BOARD

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <math.h>
#include <slaveboard/robot_motor.h>

#define MOTOR_1_FORWARD_PIN PA_0
#define MOTOR_1_REVERSE_PIN PA_1
#define MOTOR_2_FORWARD_PIN PA_2
#define MOTOR_2_REVERSE_PIN PA_3
#define MOTOR_3_FORWARD_PIN PA_6
#define MOTOR_3_REVERSE_PIN PA_7
#define MOTOR_4_FORWARD_PIN PB_0
#define MOTOR_4_REVERSE_PIN PB_1

RobotMotor motor_1;
RobotMotor motor_2;
RobotMotor motor_3;
RobotMotor motor_4;

void setup() {
  motor_1 = RobotMotor(MOTOR_1_FORWARD_PIN, MOTOR_1_REVERSE_PIN);
  motor_2 = RobotMotor(MOTOR_2_FORWARD_PIN, MOTOR_2_REVERSE_PIN);
  motor_3 = RobotMotor(MOTOR_3_FORWARD_PIN, MOTOR_3_REVERSE_PIN);
  motor_4 = RobotMotor(MOTOR_4_FORWARD_PIN, MOTOR_4_REVERSE_PIN);
}

void loop() {
  motor_1.set_drive(32000, forward);
  motor_2.set_drive(32000, forward);
  motor_3.set_drive(32000, reverse);
  motor_4.set_drive(32000, reverse);
  delay(5000);
  motor_1.stop();
  motor_2.stop();
  motor_3.stop();
  motor_4.stop();
  delay(5000);
  motor_1.set_drive(32000, reverse);
  motor_2.set_drive(32000, reverse);
  motor_3.set_drive(32000, forward);
  motor_4.set_drive(32000, forward);
  delay(5000);
  motor_1.stop();
  motor_2.stop();
  motor_3.stop();
  motor_4.stop();
  delay(5000);
}

#endif