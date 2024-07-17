#ifndef READ_UART_H
#define READ_UART_H

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
void connect_robot_wifi(WiFiConfig_t* config);


#endif