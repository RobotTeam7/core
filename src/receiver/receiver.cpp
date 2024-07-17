#include <receiver/constants.h>

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <common/resource_manager.h>
#include <common/pwm.h>
#include <common/robot_motor.h>

#include <communication/wifi_server.h>
#include <communication/decode.h>


QueueHandle_t outboundWiFiQueue = xQueueCreate(10, sizeof(StatusMessage_t));
QueueHandle_t inboundWiFiQueue = xQueueCreate(10, sizeof(StatusMessage_t));

WiFiHandler_t wifi_handler = {
    .wifi_config = &wifi_config,
    .inbound_wifi_queue = &inboundWiFiQueue,
    .outbound_wifi_queue = &outboundWiFiQueue
};

void setup() {
    Serial.begin(115200);
    Serial.println("Beginning...");
    
    begin_wifi_as_server(&wifi_handler);
}

void loop() {
//   digitalWrite(STEPPER_CONTROL_PIN, HIGH);
  
//   delay(5);

//   digitalWrite(STEPPER_CONTROL_PIN, LOW);

//   delay(5);
}
