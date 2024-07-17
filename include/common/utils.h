#ifndef COMMON_UTILS
#define COMMON_UTILS

#include <Arduino.h>
#include <CircularBuffer.hpp>

#include <common/constants.h>


void log_status(const char*);   // Log a status to the serial output, if verbosity is great enough.
void log_error(const char*);    // Log an error to the serial output, if verbosity is great enough.
void log_message(const char*);  // Log a message to the serial output, if verbosity is great enough.

/**
 * @brief Obtain the average of the values contained within `sensorBuffer`, as an integer
 */
int get_buffer_average(CircularBuffer<int, REFLECTANCE_SENSOR_BUFFER_SIZE> &sensorBuffer);


#endif