#ifndef SERVO_MOTOR_H
#define SERVO_MOTOR_H

#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <common/pwm.h>
#include <common/pin.h>

#ifdef USING_BLUE_PILL
    #include <FreeRTOS.h>
    #include <task.h>
#elif USING_ESP32
    #include "freertos/FreeRTOS.h"
    #include "freertos/task.h"
#endif

class ServoMotor {
private:
    uint8_t boundControlPin;
    uint16_t position;
public:
    ServoMotor();
    ServoMotor(uint8_t boundControlPin, uint16_t position);

    void set_position(uint16_t newPosition);
    uint16_t get_position();
};

#endif