#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <serving_robot/constants.h>

#include <common/resource_manager.h>
#include <common/servo_motor.h>
#include <common/robot_motor.h>
#include <common/pwm.h>
#include <motion/limit_switch.h>

#include <communication/wifi_client.h>
#include <communication/uart.h>
#include <communication/decode.h>

#include <ESP32Servo.h>


QueueHandle_t outboundWiFiQueue = xQueueCreate(10, sizeof(WiFiPacket_t));
QueueHandle_t inboundWiFiQueue = xQueueCreate(10, sizeof(WiFiPacket_t));
QueueHandle_t uart_msg_queue = xQueueCreate(10, sizeof(Packet_t));

WiFiHandler_t wifi_handler = {
    .wifi_config = &wifi_config,
    .inbound_wifi_queue = &inboundWiFiQueue,
    .outbound_wifi_queue = &outboundWiFiQueue
};

Servo claw_servo;
Servo draw_bridge_servo;
Servo plating_servo;

StepperMotor_t* stepper_motor;

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
    Serial.println("awaitng motion to be ready");
    while (!MOTION_READY) {
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(2000));
        Serial.println(“raising stepper motor”);
        actuate_stepper_motor(stepper_motor, DOWN, 3000);
        Serial.println(String(draw_bridge_servo.read()));
        for(int i = 0 ; i < SERVO_DRAW_BRIDGE_UP; i++) {
            draw_bridge_servo.write(i);
            vTaskDelay(pdMS_TO_TICKS(15));
            Serial.println(“raising!“);
        }
        claw_servo.write(SERVO_CLAW_OPEN + 10);
        Serial.println(“servo is raised”);
        vTaskDelay(pdMS_TO_TICKS(1000));
        // // Rotate with intention to drive backwards after
        // log_status(“Motion ready! Rotating!“);
        // send_uart_message(DO_SPIN, -1);
        // MOTION_BUSY = true;
        // while (MOTION_BUSY) {
        //     vTaskDelay(10 / portTICK_PERIOD_MS);
        // }
        // vTaskDelay(pdMS_TO_TICKS(1000));
        // go to cheese station
        log_status(“Motion ready! Going to Station 1!“);
        send_uart_message(GOTO, 5);
        MOTION_BUSY = true;
        while (MOTION_BUSY) {
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
        log_status(“Motion ready! Docking at cheese station!“);
        MOTION_BUSY = true;
        send_uart_message(COUNTER_DOCK, 1);
        while (MOTION_BUSY) {
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
        log_status(“lowering stepper motor”);
        actuate_stepper_motor(stepper_motor, UP, 3000);
        vTaskDelay(pdMS_TO_TICKS(7000));
        log_status(“grabby!“);
        claw_servo.write(SERVO_CLAW_CLOSED - 20);
        vTaskDelay(pdMS_TO_TICKS(1000));
        log_status(“returning to counter!“);
        send_uart_message(TAPE_RETURN);
        MOTION_BUSY = true;
        while (MOTION_BUSY) {
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
        // log_status(“rotating!“);
        // send_uart_message(DO_SPIN, 1);
        // while (MOTION_BUSY) {
        //     vTaskDelay(10 / portTICK_PERIOD_MS);
        // }
        // vTaskDelay(pdMS_TO_TICKS(1000));
        log_status(“go to plate!“);
        send_uart_message(GOTO, 4);
        while (MOTION_BUSY) {
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
        log_status(“raising stepper motor”);
        actuate_stepper_motor(stepper_motor, DOWN, 3000);
        vTaskDelay(pdMS_TO_TICKS(7000));
        log_status(“dock at plates!“);
        send_uart_message(COUNTER_DOCK, 1);
        while (MOTION_BUSY) {
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
        log_status(“dropping cheese!“);
        claw_servo.write(SERVO_CLAW_OPEN);
        Serial.println(“Done!“);
        while (1) {
            vTaskDelay(1000);
        }
        // log_status(“Motion ready! Sending move command”);
        // send_uart_message(GOTO, 1);
        // MOTION_BUSY = true; // should be set in send_uart, not here where we could forget
        // while (MOTION_BUSY) {
        //     vTaskDelay(10 / portTICK_PERIOD_MS);
        // }
        // log_status(“Motion ready! Sending move command”);
        // send_uart_message(GOTO, 3);
        // MOTION_BUSY = true; // should be set in send_uart, not here where we could forget
        // while (MOTION_BUSY) {
        //     vTaskDelay(10 / portTICK_PERIOD_MS);
        // }
        // vTaskDelay(pdMS_TO_TICKS(1000));
        // log_status(“Motion ready! Sending move command”);
        // send_uart_message(GOTO, 2);
        // MOTION_BUSY = true; // should be set in send_uart, not here where we could forget
        // while (MOTION_BUSY) {
        //     vTaskDelay(10 / portTICK_PERIOD_MS);
        // }
        // MOTION_BUSY = true;
        // send_uart_message(COUNTER_DOCK, 1);
        // while (MOTION_BUSY) {
        //     vTaskDelay(10 / portTICK_PERIOD_MS);
        // }
        // vTaskDelay(1000);
        // actuate_stepper_motor(stepper_motor, UP, 3000);
        // vTaskDelay(pdMS_TO_TICKS(8000));
        // claw_servo.write(30);
        // vTaskDelay(pdMS_TO_TICKS(2000));
        // actuate_stepper_motor(stepper_motor, DOWN, 3000);
        // vTaskDelay(pdMS_TO_TICKS(8000));
        // break;
    }
}

void setup() {
    Serial.begin(115200); // Initialize serial monitor

    Serial.println("servos initialized!");

    claw_servo = Servo();
    claw_servo.attach(SERVO_CLAW_PIN);
    draw_bridge_servo = Servo();
    draw_bridge_servo.attach(SERVO_DRAW_BRIDGE_PIN);
    plating_servo = Servo();
    plating_servo.attach(SERVO_PLATE_PIN);

    stepper_motor = instantiate_stepper_motor(STEPPER_CONTROL_PIN, STEPPER_DIRECTION_PIN, STEPPER_SLEEP_PIN, 0, 500);
    // actuate_stepper_motor(stepper_motor, UP, 1000);
    // delay(2000);
    // claw_servo.write(90);
    // delay(1000);
    // claw_servo.write(30);
    // delay(1000);
    // actuate_stepper_motor(stepper_motor, DOWN, 1000);
    // delay(2000);

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
