#ifndef ROBOT_MOTOR_H
#define ROBOT_MOTOR_H

#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <common/pwm.h>
#include <common/utils.h>

/**
 * @brief This struct encapsulates the data of a robot's motor.
 * 
 * @details Use this class to bind forward and reverse pins, and control the motor abstractly through simple commands.
 */
typedef struct {
    uint8_t boundForwardPin;  // Pin responsible for delivering a "drive forward" pulse train
    uint8_t boundReversePin;  // Pin responsible for delivering a "drive reverse" pulse train
    uint8_t currentState;     // 0: none, 1: forward, 2:
    uint16_t currentDrive;    // 0 – 65535
} RobotMotor_t;

/**
 * @brief Instantiate a motor, binding `forwardPin` and `reversePin` to it. The pins will be configured as necessary.
 */
RobotMotor_t* instantiate_robot_motor(uint8_t forwardPin, uint8_t reversePin, ledc_timer_t timer);

/**
 * @brief Set the drive state of this motor.
 * 
 * @param driveValue Drive power, where 0 is stopped and 65535 is full power
 * @param newState Direction of the motor, either `forward`, `reverse`, or `none`.
 * 
 * @returns Heap-allocated (created with `malloc()`) pointer. Remember to free!
 */
void motor_set_drive(RobotMotor_t* robotMotor, int16_t driveValue);

/**
 * @brief Stop the motor.
 */
void motor_stop(RobotMotor_t* robotMotor); 



#endif // ROBOT_MOTOR_H