#include <communication/wifi.h>


const WiFiConfig_t wifi_config = {
    .ssid = "UniqueESP32_AP",
    .password = "12345678",
    .host = "192.168.4.1"
};

const uint16_t port = 12345;

int checkWiFiHandler(WiFiHandler_t* wifiHandler) {
    if (wifiHandler == NULL) {
        return 1;
    } else {
        return (wifiHandler->wifi_config == NULL) || (wifiHandler->outbound_wifi_queue == NULL) || (wifiHandler->inbound_wifi_queue == NULL);
    }
}
