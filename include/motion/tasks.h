#ifndef MOTION_TASKS_H
#define MOTION_TASKS_H

#include <Arduino.h>
#include <CircularBuffer.hpp>

#include <common/robot_motor.h>

#include <motion/constants.h>
#include <motion/utils.h>


typedef struct {
    DualTapeSensor_t* tapeSensor;
    QueueHandle_t* xSharedQueue;
} TapeAwarenessData_t;

void TaskRotate(void* pvParameters);
void TaskFollowTape(void* pvParameters);
void TaskStationTracking(void* pvParameters);
void TaskDrive(void* pvParameters);


#endif // MOTION_TASKS_H