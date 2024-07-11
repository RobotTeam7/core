#ifndef SERVO_MOTOR_H
#define SERVO_MOTOR_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <math.h>
#include <common/pwm.h>
#include <common/pin.h>
#include <FreeRTOS.h>
#include <task.h>

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