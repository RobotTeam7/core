#ifndef SLAVEBOARD_UTILS
#define SLAVEBOARD_UTILS

#include <FreeRTOS.h>
#include <Arduino.h>
#include <task.h>
#include <slaveboard/constants.h>
#include <common/utils.h>

void checkResetCause();
void monitorStackUsage(TaskHandle_t* xHandleRotating, TaskHandle_t* xReflectanceHandle, TaskHandle_t* xHandleFollowing, TaskHandle_t* xMasterHandle);


extern "C"
{
  void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName);
}

#endif