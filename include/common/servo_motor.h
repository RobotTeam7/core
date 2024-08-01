#ifndef SERVO_MOTOR_H
#define SERVO_MOTOR_H

#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <common/pwm.h>
#include <common/constants.h>
#include <common/utils.h>

#define SERVO_MOTOR_CONTROL_FREQUENCY 50


/**
 * @brief This struct encapsulates the data for a servo motor 
 * 
 * @details Use this struct and its methods to control a servo motor connected to this robot
 */
typedef struct {
    uint8_t boundControlPin;
    uint16_t position;
    float max_duty_cycle;
    float min_duty_cycle;
} ServoMotor_t;


/**
 * @brief Create a new servo motor, binding `controlPin` to it with `position` as its initial position.
 * 
 * @param controlPin The pin that will be used to control the servo motor's position.
 * @param position The servo motor's initial position.
 * 
 * @returns Heap-allocated (created with `malloc()`) pointer. Remember to free!
 */
ServoMotor_t* instantiate_servo_motor(uint8_t controlPin, float max_duty_cycle, float min_duty_cycle);

/**
 * @brief Set the position of this servo motor by passing a percentage where 0% is fully closed and 100% is fully open.
 * 
 * @param percentage value within the range 0.0 – 1.0.
 */
void set_servo_position_percentage(ServoMotor_t* servoMotor, int percentage);


#endif // SERVO_MOTOR_H