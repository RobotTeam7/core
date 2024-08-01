#ifndef MOTION_TASKS_H
#define MOTION_TASKS_H

#include <Arduino.h>
#include <CircularBuffer.hpp>

#include <common/robot_motor.h>

#include <motion/constants.h>
#include <motion/utils.h>
#include <common/reflectance_sensor.h>


extern TaskHandle_t xHandleRotating;
extern TaskHandle_t xDriveHandle;
extern TaskHandle_t xHandleFollowing;
extern TaskHandle_t xStationTrackingHandle;
extern TaskHandle_t xDockingHandle;
extern TaskHandle_t xCounterDockingHandle;
extern TaskHandle_t xReturnToTapeHandle;
extern TaskHandle_t xFollowWallHandle;

typedef struct {
    TapeSensor_t* fontTapeSensor;
    TapeSensor_t* backTapeSensor;
    QueueHandle_t* xSharedQueue;
} NavigationData_t;

typedef struct {
    TapeSensor_t* middleTapeSensor;
    TaskHandle_t* masterHandle;
} ReturnToTapeData_t;

typedef struct {
    QueueHandle_t* xSharedQueue;
} DockingData_t;

typedef struct {
    TapeSensor_t* fontTapeSensor;
    TapeSensor_t* backTapeSensor;
} FullSensorData_t;

void TaskRotate(void* pvParameters);
void TaskFollowTape(void* pvParameters);
void TaskStationTracking(void* pvParameters);
void TaskDrive(void* pvParameters);
void TaskDocking(void* pvParameters);
void TaskCounterDocking(void* pvParameters);
void TaskReturnToTape(void* pvParameters);
void TaskCounterDocking(void* pvParameters);
void TaskFollowWall(void* pvParameters);


#endif // MOTION_TASKS_H