#include <chef_robot/constants.h>

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <common/resource_manager.h>
#include <common/pwm.h>
#include <common/robot_motor.h>

#include <communication/wifi_server.h>
#include <communication/decode.h>
#include <common/limit_switch.h>


QueueHandle_t outboundWiFiQueue = xQueueCreate(10, sizeof(WiFiPacket_t));
QueueHandle_t inboundWiFiQueue = xQueueCreate(10, sizeof(WiFiPacket_t));

WiFiHandler_t wifi_handler = {
    .wifi_config = &wifi_config,
    .inbound_wifi_queue = &inboundWiFiQueue,
    .outbound_wifi_queue = &outboundWiFiQueue
};

WiFiPacket_t new_packet;


TaskHandle_t xTestSwitchHandle = NULL;

void TaskTestSwitch(void* pvParameters) {

    uint32_t ulNotificationValue;
    while (1) {
        Serial.println("before notify");
        // Wait to be notified that limit switch hit the counter
        xTaskNotifyWait(0x00, 0xFFFFFFFF, &ulNotificationValue, portMAX_DELAY);

        Serial.println("Switch pressed!");
        // send message to master that we reached the counter? 
    }
}

void setup() {
    Serial.begin(115200);
    xTaskCreate(TaskTestSwitch, "test switch", 2048, NULL, 1, &xTestSwitchHandle);
    LimitSwitch_t* limit_switch = instantiate_limit_switch(21, &xTestSwitchHandle);

}

void loop() {
    Serial.println("pin 21 value: " + String(digitalRead(21)));
    delay(500);

}

