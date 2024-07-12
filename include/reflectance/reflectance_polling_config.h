#ifndef REFLECTANCE_POLLING_CONFIG_H
#define REFLECTANCE_POLLING_CONFIG_H

#include <CircularBuffer.h>
#include <FreeRTOS.h>
#include <task.h>
#include <FreeRTOS/Source/include/queue.h>

#define BUFFER_SIZE 1


struct ReflectancePollingConfig {
    CircularBuffer<int, BUFFER_SIZE>* left_sensor_buffer;
    CircularBuffer<int, BUFFER_SIZE>* right_sensor_buffer;
    QueueHandle_t* xSharedQueue;
};

#endif