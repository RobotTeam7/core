#ifndef REFLECTANCE_SENSOR_H
#define REFLECTANCE_SENSOR_H

#include <Arduino.h>

#include <common/utils.h>

/**
 * @brief This struct encapsulates the data contained by an analog tape sensor.
 */
typedef struct {
    uint8_t pin;
    uint16_t value;
} TapeSensor_t;

/**
 * @brief Instantiate a new analog tape sensor.
 * 
 * @param leftSensorPin Pin connected to the left tape sensor's output
 * @param rightSensorPin Pin connected to the right tape sensor's output
 * 
 * @returns Heap-allocated (created with `malloc()`) pointer. Remember to free!
 */
TapeSensor_t* instantiate_tape_sensor(uint8_t pin);

/**
 * @brief Perform a read of `tapeSensor`, updating its internal data.
 */
void read_tape_sensor(TapeSensor_t* tapeSensor);


#endif // REFLECTANCE_SENSOR_H