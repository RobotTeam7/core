#ifndef ROBOT_WIFI_H
#define ROBOT_WIFI_H

#include <WiFi.h>
#include <freertos/queue.h>
#include <common/utils.h>
#include <communication/decode.h>
#include <esp_now.h>
#include <communication/communication.h>

extern QueueHandle_t wifi_message_queue;

#if robot == 0
    #pragma message "Compiling Midnight Rambler!"
    #define PEER_MAC_ADDRESS {0x64, 0xB7, 0x08, 0x9D, 0x70, 0x18} // Fiddler's MAC address
#elif robot == 1
    #pragma message "Compiling Fiddler!"
    #define PEER_MAC_ADDRESS {0x64, 0xB7, 0x08, 0x9C, 0x5B, 0x90} // Midnight Rambler's MAC address
#endif  

extern const uint8_t mac_address[6];

void init_wifi();


#endif // ROBOT_WIFI_H