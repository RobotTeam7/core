#ifndef MOTION_TASKS_H
#define MOTION_TASKS_H

#include <Arduino.h>
#include <CircularBuffer.hpp>

#include <common/robot_motor.h>

#include <motion/constants.h>
#include <motion/utils.h>


extern TaskHandle_t xHandleRotating;
extern TaskHandle_t xDriveHandle;
extern TaskHandle_t xHandleFollowing;
extern TaskHandle_t xStationTrackingHandle;
extern TaskHandle_t xDockingHandle;
extern TaskHandle_t xCounterDockingHandle;
extern TaskHandle_t xReturnToTapeHandle;

typedef struct {
    DualTapeSensor_t* tapeSensor;
    QueueHandle_t* xSharedQueue;
} TapeAwarenessData_t;

void TaskRotate(void* pvParameters);
void TaskFollowTape(void* pvParameters);
void TaskStationTracking(void* pvParameters);
void TaskDrive(void* pvParameters);
void TaskDocking(void* pvParameters);
void TaskCounterDocking(void* pvParameters);
void TaskReturnToTape(void* pvParameters);


#endif // MOTION_TASKS_H