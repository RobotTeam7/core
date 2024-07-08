#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <common/resource_manager.h>
#include <common/pwm.h>
#include <common/pin.h>
#include <common/robot_motor.h>
#include <wifi/wifi_sender.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#define step 7
#define dir 5

const char* ssid = "UniqueESP32_AP";
const char* password = "12345678";
const char* host = "192.168.4.1"; // This should be the IP address of the ESP32 AP


void getChannel(void *pvParameter)
{
    static bool channelAvailable = true;

    while (1) {
        if (channelAvailable) {
            int channel = ChannelManager::getInstance().requestChannel();
            if (channel != -1) {
                Serial.print("Got channel: ");
                Serial.println(channel);
            } else {
                Serial.println("Couldn't get channel...");
                channelAvailable = false;
            }
        } else {
            Serial.println("No channels available!");
        }
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}

WiFiConfig config;

void setup() {
    Serial.begin(115200);
    Serial.println("Beginning...");

    config = {ssid, password, host};
    Sender wifi_sender = Sender(&config);
    
    wifi_sender.begin_server();

//   pinMode(step, OUTPUT);
//   pinMode(dir, OUTPUT);

//   xTaskCreate(&getChannel, "getChannel", 2048, NULL, 5, NULL);
//   digitalWrite(dir, HIGH);
    // Pin pin3(7);

    // pwm::bind_pwm(pin1.getNumber());
    // pwm::bind_pwm(pin3.getNumber());
    // pwm::set_pwm(pin1.getNumber(), 16000);
    // pwm::set_pwm(pin3.getNumber(), 32000);

    // RobotMotor motor = RobotMotor(step, dir);
    // Serial.println("here");
    // motor.set_drive(16000, forward);
}

void loop() {
//   digitalWrite(step, HIGH);
  
//   delay(5);

//   digitalWrite(step, LOW);

//   delay(5);
}