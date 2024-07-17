#ifndef READ_COMMUNICATION_H
#define READ_COMMUNICATION_H

#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <communication/uart.h>
#include <communication/wifi.h>
#include <communication/decode.h>


// UART
void begin_uart_read();

// WiFi
void connect_robot_wifi(const WiFiConfig_t* config);


#endif