#ifndef ROBOT_WIFI_H
#define ROBOT_WIFI_H

#include <WiFi.h>


typedef struct {
    const char* ssid;
    const char* password;
    const char* host;
} WiFiConfig_t;

extern const WiFiConfig_t wifi_config;


#endif // ROBOT_WIFI_H