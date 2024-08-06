#ifndef MOTION_TASKS_H
#define MOTION_TASKS_H

#include <common/hal.h>

#include <motion/constants.h>


extern TaskHandle_t xDriveHandle;
extern TaskHandle_t xCounterDockingHandle;
extern TaskHandle_t xFollowWallHandle;
extern TaskHandle_t xHomingHandle;

typedef struct {
    TapeSensor_t* middleTapeSensor;
    TaskHandle_t* masterHandle;
} ReturnToTapeData_t;

typedef struct {
    TapeSensor_t* frontTapeSensor;
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
void TaskHoming(void* pvParameters);


#endif // MOTION_TASKS_H