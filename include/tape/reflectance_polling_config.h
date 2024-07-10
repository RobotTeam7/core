#ifndef REFLECTANCE_POLLING_CONFIG_H
#define REFLECTANCE_POLLING_CONFIG_H

#include <Arduino.h>
#include <CircularBuffer.h>

#define BUFFER_SIZE 1

struct ReflectancePollingConfig {
    CircularBuffer<int, BUFFER_SIZE>* left_sensor_buffer;
    CircularBuffer<int, BUFFER_SIZE>* right_sensor_buffer;
};

#endif