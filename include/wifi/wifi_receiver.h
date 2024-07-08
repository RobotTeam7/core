#ifndef WIFI_RECEIVER_H
#define WIFI_RECEIVER_H

#include <Arduino.h>
#include <WiFi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <wifi/config.h>

class Receiver {
private:
    WiFiConfig* config;

public:
    Receiver(WiFiConfig* config);

    void begin_wifi();
};

#endif