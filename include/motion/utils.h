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

int get_last_station_server(int side_station, int y_direction);
int get_last_station_chef(int side_station, int y_direction);
int get_last_side_station_server(int last_station, int y_direction);
int get_last_side_station_chef(int last_station, int y_direction);



#endif // MOTION_UTILS_H