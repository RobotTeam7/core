#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <common/resource_manager.h>
#include <common/pwm.h>
#include <common/pin.h>
#include <common/robot_motor.h>
#include <communication/read.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#define step 7
#define dir 5

const char* ssid = "UniqueESP32_AP";
const char* password = "12345678";
const char* host = "192.168.4.1"; // This should be the IP address of the ESP32 AP

WiFiConfig_t config;

void setup() {
    Serial.begin(115200);
    Serial.println("Beginning...");

    config = {ssid, password, host};
    
    connect_robot_wifi(&config);
}

void loop() {
//   digitalWrite(step, HIGH);
  
//   delay(5);

//   digitalWrite(step, LOW);

//   delay(5);
}