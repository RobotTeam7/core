#ifndef ROBOT_WIFI_H
#define ROBOT_WIFI_H

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <WiFi.h>
#include <esp_now.h>

#include <common/utils.h>

#include <communication/communication.h>

// The queue of type Packet_t which will be the destination for messages received over WiFi (ESP-NOW).
extern QueueHandle_t wifi_message_queue;

#if robot == 0
    #pragma message "Compiling Midnight Rambler!"
    #define PEER_MAC_ADDRESS {0x64, 0xB7, 0x08, 0x9D, 0x70, 0x18} // Fiddler's MAC address
#elif robot == 1
    #pragma message "Compiling Fiddler!"
    #define PEER_MAC_ADDRESS {0x64, 0xB7, 0x08, 0x9C, 0x5B, 0x90} // Midnight Rambler's MAC address
#endif  

// The MAC address of this board's peer.
extern const uint8_t mac_address[6];

/**
 * @brief Initialize WiFi communications to this board's peer through ESP-NOW. This function blocks until the peer
 * can be added.
 */
void init_wifi();

/**
 * @brief Send a message consisting of `command` and `value` over WiFi (ESP-NOW) to this board's registered peer.
 */
void send_wifi_message(CommandMessage_t command, int8_t value);


#endif // ROBOT_WIFI_H