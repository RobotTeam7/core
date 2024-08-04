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

#include <communication/wifi.h>
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
        wait_for_motion();
        grab_with_claw(ServoPositionsPercentage_t::CLAW_CLOSED_BUN);

        // DROP BUN  
        log_status("Doing pirouette!");
        send_command(DO_PIROUETTE, 2);
        wait_for_motion();

        log_status("getting bun");
        send_command(FOLLOW_WALL_TO, 4);
        wait_for_motion();
        open_claw(ServoPositionsPercentage_t::VERTICAL_HEIGHT_2);

        // PATTY         
        log_status("Doing pirouette!");
        send_command(DO_PIROUETTE, 4);
        wait_for_motion();

        log_status("getting patty");
        send_command(FOLLOW_WALL_TO, 1);
        wait_for_motion();
        grab_with_claw(ServoPositionsPercentage_t::CLAW_CLOSED_PATTY);

        // COOK
        log_status("Doing pirouette!");
        send_command(DO_PIROUETTE, 1);
        wait_for_motion();

        log_status("Informing that patty is ready...");
        send_wifi_message(CommandMessage_t::NEXT_ACTION, 0);

        log_status("goto cooktop");
        send_command(FOLLOW_WALL_TO, 3);
        wait_for_motion();
        open_claw(ServoPositionsPercentage_t::VERTICAL_HEIGHT_2);
        
        // TOP BUN
        log_status("Doing pirouette!");
        send_command(DO_PIROUETTE, 3);
        wait_for_motion();

        log_status("getting top bun");
        send_command(FOLLOW_WALL_TO, 2);
        wait_for_motion();
        grab_with_claw(ServoPositionsPercentage_t::CLAW_CLOSED_BUN);

        // DROP BUN  
        log_status("Doing pirouette!");
        send_command(DO_PIROUETTE, 2);
        wait_for_motion();

        log_status("getting bun");
        send_command(FOLLOW_WALL_TO, 4);
        wait_for_motion();
        open_claw(ServoPositionsPercentage_t::VERTICAL_HEIGHT_2);

        // RETURN
        log_status("Doing pirouette!");
        send_command(DO_PIROUETTE, 3);
        wait_for_motion();

        log_status("Informing that top bun is ready...");
        send_wifi_message(CommandMessage_t::NEXT_ACTION, 0);

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

    initialize_uart(&uart_msg_queue);

    xTaskCreate(uart_msg_handler, "UART_msg_handler", 2048, NULL, 1, NULL);

    while (!MOTION_READY) {
        delay(100);
    }

    log_status("Connected to motion board!");
    
    Serial.println("vertical servo go up!");
    set_servo_position_percentage(vertical_servo, 0);

    init_wifi();

    xTaskCreate(wifi_msg_handler, "WiFi_msg_handler", 2048, NULL, 1, NULL);

    delay(500);

    while (!wifi_ready) {
        send_wifi_message(CommandMessage_t::READY, 0);
        log_status("Trying to handshake WiFi...");
        delay(500);
    }

    log_status("Connected to WiFi!");

    xTaskCreate(TaskMaster, "Master", 2048, NULL, 1, NULL);
}

void loop() {

}
