#include <server_robot/constants.h>

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <common/resource_manager.h>
#include <common/pwm.h>
#include <common/robot_motor.h>

#include <communication/wifi_server.h>
#include <communication/decode.h>


QueueHandle_t outboundWiFiQueue = xQueueCreate(10, sizeof(WiFiPacket_t));
QueueHandle_t inboundWiFiQueue = xQueueCreate(10, sizeof(WiFiPacket_t));

WiFiHandler_t wifi_handler = {
    .wifi_config = &wifi_config,
    .inbound_wifi_queue = &inboundWiFiQueue,
    .outbound_wifi_queue = &outboundWiFiQueue
};

WiFiPacket_t new_packet;

void setup() {
    Serial.begin(115200);
    Serial.println("Beginning...");

    new_packet.byte1 = 0x01;
    new_packet.byte2 = 0x06;
    
    begin_wifi_as_server(&wifi_handler);
}

void loop() {
    Serial.println("Trying to send packet...");
    xQueueSend(outboundWiFiQueue, &new_packet, portMAX_DELAY);
    delay(2000);
}
