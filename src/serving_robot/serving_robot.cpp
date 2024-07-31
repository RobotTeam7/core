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
ServoMotor_t* plating_servo;
ServoMotor_t* vertical_servo;

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
                    send_uart_message(CommandMessage_t::ACK, 0, false);
                    break;
                
                case ACCEPTED:
                    log_status("Motion accepted command.");
                    break;

                case OCCUPIED:
                    log_error("Motion rejected command!");
                    break;

                case COMPLETED:
                    MOTION_BUSY = false;
                    send_uart_message(CommandMessage_t::ACK, 0, false);
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
    Serial.println("awaitng motion to be ready");
    while (!MOTION_READY) {
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }

    while (true) {
        // delay for uart to work
        vTaskDelay(pdMS_TO_TICKS(500));

        vTaskDelay(pdMS_TO_TICKS(3000));


        log_status("dock on side");
        send_uart_message(COUNTER_DOCK, 1);
        MOTION_BUSY = true;
        while (MOTION_BUSY) {
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
        vTaskDelay(pdMS_TO_TICKS(1000));

        log_status("wall slam to 3");
        send_uart_message(FOLLOW_WALL_TO, 3);
        MOTION_BUSY = true;
        while (MOTION_BUSY) {
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
        vTaskDelay(pdMS_TO_TICKS(1000));

        log_status("vertical servo down");
        set_servo_position_percentage(vertical_servo, 1.0);
        vTaskDelay(pdMS_TO_TICKS(500));

        // close servo
        log_status("claw servo closed");
        set_servo_position_percentage(claw_servo, 0.2);
        vTaskDelay(pdMS_TO_TICKS(500));
    
        log_status("vertical servo up");
        set_servo_position_percentage(vertical_servo, 0.0);
        vTaskDelay(pdMS_TO_TICKS(500));

        log_status("wall slam to 4");
        send_uart_message(FOLLOW_WALL_TO, 4);
        MOTION_BUSY = true;
        while (MOTION_BUSY) {
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
        vTaskDelay(pdMS_TO_TICKS(1000));

        set_servo_position_percentage(claw_servo, 1.0);
        vTaskDelay(pdMS_TO_TICKS(500));

        log_status("wall slam to 3");
        send_uart_message(FOLLOW_WALL_TO, 3);
        MOTION_BUSY = true;
        while (MOTION_BUSY) {
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
        vTaskDelay(pdMS_TO_TICKS(1000));

        log_status("pirouette");
        send_uart_message(DO_PIROUETTE, 1);
        MOTION_BUSY = true;
        while (MOTION_BUSY) {
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
        vTaskDelay(pdMS_TO_TICKS(1000));

        log_status("wall slam to 2");
        send_uart_message(FOLLOW_WALL_TO, 2);
        MOTION_BUSY = true;
        while (MOTION_BUSY) {
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
        vTaskDelay(pdMS_TO_TICKS(1000));

        
        log_status("vertical servo down");
        set_servo_position_percentage(vertical_servo, 1.0);
        vTaskDelay(pdMS_TO_TICKS(500));

        // close servo
        log_status("claw servo closed");
        set_servo_position_percentage(claw_servo, 0.2);
        vTaskDelay(pdMS_TO_TICKS(500));
    
        log_status("vertical servo up");
        set_servo_position_percentage(vertical_servo, 0.0);
        vTaskDelay(pdMS_TO_TICKS(500));


        log_status("pirouette");
        send_uart_message(DO_PIROUETTE, 3);
        MOTION_BUSY = true;
        while (MOTION_BUSY) {
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
        vTaskDelay(pdMS_TO_TICKS(1000));

        log_status("wall slam to 4");
        send_uart_message(FOLLOW_WALL_TO, 4);
        MOTION_BUSY = true;
        while (MOTION_BUSY) {
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
        vTaskDelay(pdMS_TO_TICKS(1000));


        log_status("claw servo open");
        set_servo_position_percentage(claw_servo, 1.0);
        vTaskDelay(pdMS_TO_TICKS(500));




        // log_status("servos going to 1");
        // for(float percentage = 0; percentage < 1; percentage += 0.01){
        //     // set_servo_position_percentage(claw_servo, percentage);
        //     // set_servo_position_percentage(draw_bridge_servo, percentage);
        //     // set_servo_position_percentage(plating_servo, percentage);
        //     // set_servo_position_percentage(vertical_servo, percentage);
        //     vTaskDelay(10);
        // }

        // vTaskDelay(pdMS_TO_TICKS(3000));

        // log_status("servos going to 0");
        // for(float percentage = 1; percentage > 0; percentage -= 0.01){
        //     // set_servo_position_percentage(claw_servo, percentage);
        //     // set_servo_position_percentage(draw_bridge_servo, percentage);
        //     // set_servo_position_percentage(plating_servo, percentage);
        //     // set_servo_position_percentage(vertical_servo, percentage);
        //     vTaskDelay(10);
        // }

        Serial.println("Done!");
        while (1) {
            vTaskDelay(1000);
        }
    }
}

void setup() {
    Serial.begin(115200); // Initialize serial monitor

    init_pwm();

    Serial.println("servos initialized!");

    claw_servo = instantiate_servo_motor(SERVO_CLAW_PIN, SERVO_CLAW_OPEN, SERVO_CLAW_CLOSED);
    draw_bridge_servo = instantiate_servo_motor(SERVO_DRAW_BRIDGE_PIN, SERVO_DRAW_BRIDGE_UP, SERVO_DRAW_BRIDGE_DOWN);
    plating_servo = instantiate_servo_motor(SERVO_PLATE_PIN, SERVO_PLATE_OPEN, SERVO_PLATE_CLOSED);
    vertical_servo = instantiate_servo_motor(SERVO_VERTICAL_PIN, 0.1, SERVO_VERTICAL_UP);
    
    delay(1000);
    
    Serial.println("vertical servo go up!");
    set_servo_position_percentage(vertical_servo, 0);

    // connect_to_wifi_as_client(&wifi_handler);

    initialize_uart(&uart_msg_queue);

    xTaskCreate(uart_msg_handler, "UART_msg_handler", 2048, NULL, 1, NULL);

    delay(100);
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
