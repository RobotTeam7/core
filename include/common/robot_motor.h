#ifndef ROBOT_MOTOR_H
#define ROBOT_MOTOR_H

#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <common/pwm.h>
#include <common/utils.h>


#define DRIVE_MOTOR_PWM_FREQUENCY   (50)
#define FORWARD_DRIVE               (1)
#define REVERSE_DRIVE               (-1)

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
RobotMotor_t* instantiate_robot_motor(uint8_t forward_pin, uint8_t reverse_pin, ledc_timer_t timer);

/**
 * @brief Set the drive state of this motor.
 * 
 * @param drive_value Drive power, where 0 is stopped and 65535 is full power. Negative value indicates reverse direction.
 * 
 * @returns Heap-allocated (created with `malloc()`) pointer. Remember to free!
 */
void motor_set_drive(RobotMotor_t* robot_motor, int16_t drive_value);

/**
 * @brief Stop the motor.
 */
void motor_stop(RobotMotor_t* robotMotor); 



#endif // ROBOT_MOTOR_H