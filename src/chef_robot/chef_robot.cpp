#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <chef_robot/constants.h>

#include <main/main.h>

#include <common/resource_manager.h>
#include <common/servo_motor.h>
#include <common/robot_motor.h>
#include <common/pwm.h>
#include <common/limit_switch.h>

#include <communication/wifi_client.h>
#include <communication/uart.h>
#include <communication/decode.h>


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

        send_command(FOLLOW_WALL_TO, 2);
        wait_for_motion;

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
    vertical_servo = instantiate_servo_motor(SERVO_VERTICAL_PIN, SERVO_VERTICAL_DOWN, SERVO_VERTICAL_UP);
    
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
