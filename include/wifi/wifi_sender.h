#include <WiFi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

const char* ssid = "UniqueESP32_AP";
const char* password = "12345678";
WiFiServer server(80);

namespace wifi_sender {
    void begin_server();
}