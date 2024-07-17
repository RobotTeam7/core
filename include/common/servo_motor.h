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


typedef struct {
    uint8_t boundControlPin;
    uint16_t position;
} ServoMotor_t;

void set_servo_position(ServoMotor_t*, uint16_t newPosition);
ServoMotor_t* instantiate_servo_motor(uint8_t boundControlPin, uint16_t position);
void set_servo_position_percentage(ServoMotor_t*, float percentage);


#endif // SERVO_MOTOR_H