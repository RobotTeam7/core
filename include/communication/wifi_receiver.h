#ifndef WIFI_RECEIVER_H
#define WIFI_RECEIVER_H

#include <Arduino.h>
#include <WiFi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <communication/wifi.h>

void connect_robot_wifi(WiFiConfig_t* config);

#endif