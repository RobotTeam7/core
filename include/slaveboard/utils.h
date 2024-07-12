#ifndef SLAVEBOARD_UTILS
#define SLAVEBOARD_UTILS

#include <FreeRTOS.h>
#include <Arduino.h>
#include <task.h>
#include <slaveboard/constants.h>

void checkResetCause();
void monitorStackUsage(TaskHandle_t* xHandleRotating, TaskHandle_t* xReflectanceHandle, TaskHandle_t* xHandleFollowing, TaskHandle_t* xMasterHandle);

void log_status(const char*);
void log_error(const char*);
void log_message(const char*);

extern "C"
{
  void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName);
}

#endif