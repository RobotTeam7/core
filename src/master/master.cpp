#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <common/resource_manager.h>
#include <common/pwm.h>
#include <common/pin.h>
#include <common/robot_motor.h>

#include <communication/read.h>
#include <communication/send.h>
#include <communication/decode.h>

#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include <common/servo_motor.h>

#define step 7
#define dir 5

const char* ssid = "UniqueESP32_AP";
const char* password = "12345678";
const char* host = "192.168.4.1"; // This should be the IP address of the ESP32 AP

WiFiConfig_t config;


void setup() {
    Serial.begin(115200); // Initialize serial monitor

    initialize_uart();
    begin_uart_read();

    send_uart_message(GOTO, 8);
    delay(1000);
    send_uart_message(GOTO, 9);
    delay(1000);
    send_uart_message(DO_SPIN, 0);
    delay(1000);
}

// float dutyCycleHigh = 0.06;
// float dutyCycleLow = 0.02;
// int powerValueHigh = dutyCycleHigh * 65536;
// int powerValueLow = dutyCycleLow * 65536;
// int unit_16_number = 65536;

// float granularity = 0.001;
// float dutyCycle;
// int delay_value = 5;

void loop() {
    // motor_lit.set_position_percentage(0);
    // delay(500);
    // motor_lit.set_position_percentage(.50);
    // delay(500);
    // motor_lit.set_position_percentage(1);
    // delay(500);
}