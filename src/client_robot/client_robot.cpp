#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <client_robot/constants.h>

#include <common/resource_manager.h>
#include <common/servo_motor.h>
#include <common/robot_motor.h>
#include <common/pwm.h>

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

ServoMotor_t* servoMotor;

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
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }

    while (true) {
        log_status("Motion ready! Sending move command");
        send_uart_message(GOTO, 10);
        MOTION_BUSY = true; // should be set in send_uart, not here where we could forget
        while (MOTION_BUSY) {
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }

        log_status("Motion ready! Sending rotate command");
        MOTION_BUSY = true; // should be set in send_uart, not here where we could forget
        send_uart_message(DO_SPIN);
        while (MOTION_BUSY) {
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }

        log_status("Completed circuit!");
        vTaskDelete(NULL);
        break;
    }
}

void setup() {
    Serial.begin(115200); // Initialize serial monitor

    // servoMotor = instantiate_servo_motor(13, 0);

    // connect_to_wifi_as_client(&wifi_handler);

    initialize_uart();
    begin_uart_read(&uart_msg_queue);

    xTaskCreate(uart_msg_handler, "UART_msg_handler", 2048, NULL, 1, NULL);
    // xTaskCreate(wifi_msg_handler, "WiFi_msg_handler", 2048, NULL, 1, NULL);

    // send_uart_message(GOTO, 8);
    // delay(1000);
    // send_uart_message(GOTO, 9);
    // delay(1000);
    // send_uart_message(DO_SPIN, 0);
    // delay(1000);
    // delay(500);
    xTaskCreate(TaskMaster, "Master", 2048, NULL, 1, NULL);
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
    // set_servo_position_percentage(servoMotor, 0.00);
    // delay(1500);
    // set_servo_position_percentage(servoMotor, 1.00);
    // delay(1500);
    // Serial.print("Free heap: ");
    // Serial.println(ESP.getFreeHeap());
    // delay(1000); // Check every second
}
