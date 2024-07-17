#ifndef REFLECTANCE_SENSOR_H
#define REFLECTANCE_SENSOR_H

#include <Arduino.h>

#include <common/utils.h>

/**
 * @brief This struct encapsulates the data contained by an analog tape sensor.
 */
typedef struct {
    uint8_t leftPin;
    uint8_t rightPin;
    uint16_t leftValue;
    uint16_t rightValue;
} TapeSensor_t;

/**
 * @brief Instantiate a new analog tape sensor.
 * 
 * @param leftSensorPin Pin connected to the left tape sensor's output
 * @param rightSensorPin Pin connected to the right tape sensor's output
 * 
 * @returns Heap-allocated (created with `malloc()`) pointer. Remember to free!
 */
TapeSensor_t* instantiate_tape_sensor(uint8_t leftSensorPin, uint8_t rightSensorPin);

/**
 * @brief Perform a read of `tapeSensor`, updating its internal data.
 */
void read_tape_sensor(TapeSensor_t* tapeSensor);

/**
 * @brief Determine whether this tape sensor is detecting tape more on the left or right.
 * 
 * @return 1 if detecting tape on the left, -1 if on the right, and 0 if inconclusive.
 */
int is_tape_left_or_right(TapeSensor_t* tapeSensor);

// return 1 if both sensors see tape, -1 if neither sensor sees tape, 0 if only one of them sees tape
/**
 * @brief Determine whether this tape sensor can see tape with both sensors, with neither sensor, or inconclusive.
 * 
 * @return 1 if detecting both sensors can see tape, -1 if neither can see tape, and 0 if inconclusive.
 */
int is_tape_visible(TapeSensor_t* tapeSensor);


#endif // REFLECTANCE_SENSOR_H