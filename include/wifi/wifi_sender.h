#ifndef WIFI_SENDER_H
#define WIFI_SENDER_H

#include <WiFi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <wifi/config.h>


extern WiFiServer server;

class Sender {
private:
    WiFiConfig* config;

public:
    Sender(WiFiConfig* config);

    void begin_server();
};

#endif