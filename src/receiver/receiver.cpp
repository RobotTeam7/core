#include <receiver/constants.h>

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <common/resource_manager.h>
#include <common/pwm.h>
#include <common/robot_motor.h>

#include <communication/read.h>


WiFiConfig_t config;

void setup() {
    Serial.begin(115200);
    Serial.println("Beginning...");
    
    connect_robot_wifi(&wifi_config);
}

void loop() {
//   digitalWrite(STEPPER_CONTROL_PIN, HIGH);
  
//   delay(5);

//   digitalWrite(STEPPER_CONTROL_PIN, LOW);

//   delay(5);
}
