#ifndef COMMON_UTILS
#define COMMON_UTILS

#include <Arduino.h>
#include <CircularBuffer.hpp>

#include <common/constants.h>


void log_status(const char*);
void log_error(const char*);
void log_message(const char*);

/**
 * @brief Obtain the average of the values contained within the buffer, as an integer
 */
int get_buffer_average(CircularBuffer<int, REFLECTANCE_SENSOR_BUFFER_SIZE> &sensorBuffer);


#endif