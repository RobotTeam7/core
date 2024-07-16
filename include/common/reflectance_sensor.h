#ifndef REFLECTANCE_SENSOR_H
#define REFLECTANCE_SENSOR_H

#include <Arduino.h>
#include <common/utils.h>


typedef struct {
    uint8_t leftPin;
    uint8_t rightPin;
    uint16_t leftValue;
    uint16_t rightValue;
} TapeSensor_t;

TapeSensor_t* instantiate_tape_sensor(uint8_t leftSensorPin, uint8_t rightSensorPin);
void read_tape_sensor(TapeSensor_t* tapeSensor);
int is_tape_left_or_right(TapeSensor_t* tapeSensor);
int is_tape_visible(TapeSensor_t* tapeSensor);


#endif