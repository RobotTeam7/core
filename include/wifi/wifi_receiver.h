#ifndef WIFI_SENDER_H
#define WIFI_SENDER_H

#include <Arduino.h>
#include <WiFi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

const char* ssid = "UniqueESP32_AP";
const char* password = "12345678";
const char* host = "192.168.4.1"; // This should be the IP address of the ESP32 AP

namespace wifi_receiver {
    void begin_wifi();
}

#endif