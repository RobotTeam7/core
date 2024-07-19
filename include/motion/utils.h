#ifndef MOTION_UTILS_H
#define MOTION_UTILS_H

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <common/utils.h>

#include <motion/constants.h>


void checkResetCause();
void monitorStackUsage(TaskHandle_t* xHandleRotating, TaskHandle_t* xReflectanceHandle, TaskHandle_t* xHandleFollowing, TaskHandle_t* xMasterHandle, TaskHandle_t* xStationTrackingHandle);


extern "C"
{
  void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName);
}


#endif // MOTION_UTILS_H