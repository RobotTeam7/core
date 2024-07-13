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

#include <common/servo_motor.h>

#define step 7
#define dir 5

const char* ssid = "UniqueESP32_AP";
const char* password = "12345678";
const char* host = "192.168.4.1"; // This should be the IP address of the ESP32 AP


WiFiConfig config;
ServoMotor motor_lit;

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
    float dutyCycleHigh = 0.06;
    int powerValueHigh = dutyCycleHigh * 65536;

    // RobotMotor motor = RobotMotor(step, dir);
    // Serial.println("here");
    // motor.set_drive(16000, forward);
    motor_lit = ServoMotor(4, powerValueHigh);
}

float dutyCycleHigh = 0.06;
float dutyCycleLow = 0.02;
int powerValueHigh = dutyCycleHigh * 65536;
int powerValueLow = dutyCycleLow * 65536;
int unit_16_number = 65536;

float granularity = 0.001;
float dutyCycle;
int delay_value = 5;

void loop() {
    motor_lit.set_position_percentage(0);
    delay(500);
    motor_lit.set_position_percentage(.50);
    delay(500);
    motor_lit.set_position_percentage(1);
    delay(500);
}