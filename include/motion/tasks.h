#ifndef MOTION_TASKS_H
#define MOTION_TASKS_H

#include <Arduino.h>
#include <CircularBuffer.hpp>

#include <common/robot_motor.h>

#include <motion/constants.h>
#include <motion/utils.h>


typedef struct {
    RobotMotorData_t* robotMotors;
    DualTapeSensor_t* tapeSensor;
    QueueHandle_t* xSharedQueue;
} TapeAwarenessData_t;

typedef struct {
    TapeAwarenessData_t* tapeAwarenessData;
    QueueHandle_t* xSharedQueue;
} RobotControlData_t;

void TaskRotate(void* pvParameters);
void TaskFollowTape(void* pvParameters);
void TaskPollReflectance(void* pvParameters);
void TaskStationTracking(void* pvParameters);
void TaskDocking(void* pvParameters);


#endif // MOTION_TASKS_H