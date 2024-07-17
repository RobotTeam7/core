#ifndef ROBOT_WIFI_H
#define ROBOT_WIFI_H

#include <WiFi.h>
#include <freertos/queue.h>
#include <common/utils.h>

typedef struct {
    const char* ssid;
    const char* password;
    const char* host;
} WiFiConfig_t;

extern const WiFiConfig_t wifi_config;

typedef struct {
    const WiFiConfig_t* wifi_config;
    QueueHandle_t* inbound_wifi_queue;
    QueueHandle_t* outbound_wifi_queue;
} WiFiHandler_t;

typedef struct {
    uint8_t byte1;
    uint8_t byte2;
} WiFiPacket_t;

extern const uint16_t port;

int checkWiFiHandler(WiFiHandler_t* wifiHandler);


#endif // ROBOT_WIFI_H