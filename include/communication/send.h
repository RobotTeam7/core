#ifndef SEND_COMMUNICATION_H
#define SEND_COMMUNICATION_H

#include <Arduino.h>
#include <WiFi.h>

#include <common/utils.h>

#include <communication/wifi.h>
#include <communication/decode.h>


// UART
/**
 * @brief Send a `command` over the UART stream to the connected peripheral (with optional parameter `value`).
 */
void send_uart_message(CommandMessage_t command, uint8_t value = 0U);

// WiFi
extern WiFiServer server;
void begin_wifi_server(const WiFiConfig_t* config); 


#endif // SEND_COMMUNICATION_H