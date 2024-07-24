#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <serving_robot/constants.h>

#include <common/resource_manager.h>
#include <common/servo_motor.h>
#include <common/robot_motor.h>
#include <common/pwm.h>
#include <common/limit_switch.h>

#include <communication/wifi_client.h>
#include <communication/uart.h>
#include <communication/decode.h>


QueueHandle_t outboundWiFiQueue = xQueueCreate(10, sizeof(WiFiPacket_t));
QueueHandle_t inboundWiFiQueue = xQueueCreate(10, sizeof(WiFiPacket_t));
QueueHandle_t uart_msg_queue = xQueueCreate(10, sizeof(Packet_t));

WiFiHandler_t wifi_handler = {
    .wifi_config = &wifi_config,
    .inbound_wifi_queue = &inboundWiFiQueue,
    .outbound_wifi_queue = &outboundWiFiQueue
};

ServoMotor_t* claw_servo;
ServoMotor_t* draw_bridge_servo;

bool MOTION_READY = false;
bool MOTION_BUSY = false;

void uart_msg_handler(void *parameter) {
    log_status("Started message handler");
    while (1) {
        Packet_t new_packet;
        if (xQueueReceive(uart_msg_queue, &new_packet, portMAX_DELAY)) {
            Serial.println("Received packet...");
            Serial.println(new_packet.command);
            Serial.println(new_packet.value);
            Serial.println("Received.");

            switch (new_packet.command) {
                case READY:
                    MOTION_READY = true;
                    break;
                
                case ACCEPTED:
                    log_status("Motion accepted command.");
                    break;

                case OCCUPIED:
                    log_error("Motion rejected command!");
                    break;

                case COMPLETED:
                    MOTION_BUSY = false;
                    break;
            }
        }
        vTaskDelay(10 / portTICK_PERIOD_MS); // Small delay to yield
    }
}

void wifi_msg_handler(void *parameter) {
    log_status("Started message handler");
    while (1) {
        WiFiPacket_t new_packet;
        if (xQueueReceive(inboundWiFiQueue, &new_packet, portMAX_DELAY)) {
            Serial.println("Received wifi packet...");
            Serial.println(new_packet.byte1);
            Serial.println(new_packet.byte2);
            // Serial.println("Sending over UART...");
            // send_uart_message((CommandMessage_t)new_packet.byte1, new_packet.byte2);
            // Serial.println("Sent packet over UART.");
        }
        vTaskDelay(10 / portTICK_PERIOD_MS); // Small delay to yield    
    }
}

void TaskMaster(void* pvParameters) {
    log_status("Beginning master...");

    // Wait for green light from motion board
    while (!MOTION_READY) {
        Serial.println("awaitng motion to be ready");
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    while (true) {
        set_servo_position_percentage(draw_bridge_servo, 0);
        vTaskDelay(pdMS_TO_TICKS(1000));

        log_status("GRABBY GRABBY!");
        set_servo_position(claw_servo, 0);
        vTaskDelay(pdMS_TO_TICKS(500));
        set_servo_position(claw_servo, 1);
        vTaskDelay(pdMS_TO_TICKS(500));

        log_status("Motion ready! Sending move command");
        send_uart_message(GOTO, 1);
        MOTION_BUSY = true; // should be set in send_uart, not here where we could forget
        while (MOTION_BUSY) {
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
        
        log_status("Motion ready! Sending move command");
        send_uart_message(GOTO, 3);
        MOTION_BUSY = true; // should be set in send_uart, not here where we could forget
        while (MOTION_BUSY) {
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
        vTaskDelay(pdMS_TO_TICKS(1000));

        log_status("Motion ready! Sending move command");
        send_uart_message(GOTO, 2);
        MOTION_BUSY = true; // should be set in send_uart, not here where we could forget
        while (MOTION_BUSY) {
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
        
        MOTION_BUSY = true;
        send_uart_message(COUNTER_DOCK, -1);
        while (MOTION_BUSY) {
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
        vTaskDelay(1000);

        
        set_servo_position_percentage(claw_servo, 0.2);
        break;
    }
}

void setup() {
    Serial.begin(115200); // Initialize serial monitor

    claw_servo = instantiate_servo_motor(SERVO_CLAW_PIN, SERVO_CLAW_MAX, SERVO_CLAW_MIN);
    draw_bridge_servo = instantiate_servo_motor(SERVO_DRAW_BRIDGE_PIN, SERVO_DRAW_BRIDGE_MAX, SERVO_DRAW_BRIDGE_MIN);

    // connect_to_wifi_as_client(&wifi_handler);

    initialize_uart();
    begin_uart_read(&uart_msg_queue);

    xTaskCreate(uart_msg_handler, "UART_msg_handler", 2048, NULL, 1, NULL);
    // xTaskCreate(wifi_msg_handler, "WiFi_msg_handler", 2048, NULL, 1, NULL);
    
    xTaskCreate(TaskMaster, "Master", 2048, NULL, 1, NULL);
}

void loop() {
    // delay(1500);
    // Serial.print("Free heap: ");
    // Serial.println(ESP.getFreeHeap());
    // delay(1000); // Check every second
    Serial.println("here in main board");
    delay(10000);
}
