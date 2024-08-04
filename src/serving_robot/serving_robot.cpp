#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <serving_robot/constants.h>
#include <serving_robot/rack_and_pinion.h>

#include <main/main.h>

#include <common/servo_motor.h>
#include <common/robot_motor.h>
#include <common/pwm.h>
#include <common/limit_switch.h>

#include <communication/wifi.h>
#include <communication/uart.h>
#include <communication/decode.h>


ServoMotor_t* draw_bridge_servo;
ServoMotor_t* plating_servo;

// assumes plating servo is open initially
inline void grab_plate() {
    log_status("draw bridge down");
    set_servo_position_percentage(draw_bridge_servo, ServoPositionsPercentage_t::DRAW_BRIDGE_DOWN);
    vTaskDelayMS(SERVO_ACTUATION_DELAY);
    
    log_status("plate servo close");
    set_servo_position_percentage(plating_servo, ServoPositionsPercentage_t::PLATE_CLOSED);
    vTaskDelayMS(SERVO_ACTUATION_DELAY);

    set_servo_position_percentage(draw_bridge_servo, ServoPositionsPercentage_t::DRAW_BRIDGE_DOWN + 15);
    vTaskDelayMS(SERVO_ACTUATION_DELAY);
}

static inline void serve_food() {
    send_uart_message(CommandMessage_t::SET_MULTIPLIER, 80);

    send_command(FOLLOW_WALL_TO, 3);
    vTaskDelayMS(500);
    send_uart_message(CommandMessage_t::ABORT, 0, false);
    
    send_uart_message(CommandMessage_t::SET_MULTIPLIER, 70);
    
    send_command(DO_PIROUETTE, 2);
    wait_for_motion();

    send_command(FOLLOW_WALL_TO, 1);
    vTaskDelayMS(750);
    send_command(ABORT, 0);
    vTaskDelayMS(SERVO_ACTUATION_DELAY);

    set_servo_position_percentage(plating_servo, ServoPositionsPercentage_t::PLATE_OPEN);
    vTaskDelayMS(SERVO_ACTUATION_DELAY);
    set_servo_position_percentage(draw_bridge_servo, ServoPositionsPercentage_t::DRAW_BRIDGE_UP);

}


static inline void grab_with_rack_and_claw(ServoPositionsPercentage_t percentage) {
    if(!digitalRead(SWITCH_RACK_PLATESIDE)) {
        set_rack_zero();
        vTaskDelayMS(20);
        actuate_claw_forwards();
        vTaskDelayMS(200);
    }
    grab_with_claw(percentage);
}

void TaskMaster(void* pvParameters) {
    log_status("Beginning master...");

    // Wait for green light from motion board
    Serial.println("awaitng motion to be ready");
    while (!MOTION_READY) {
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }

    while (true) {
        // STARTUP ________
        log_status("dock on side");
        send_command(STARTUP_SERVER, 0);
        wait_for_motion();

        // TOMATO _________________
        log_status("getting tomato");
        send_command(FOLLOW_WALL_TO, 3);
        wait_for_motion();
        grab_with_rack_and_claw(ServoPositionsPercentage_t::CLAW_CLOSED_TOMATO);

        log_status("getting tomato");
        send_command(FOLLOW_WALL_TO, 4);
        vTaskDelayMS(500);
        send_command(CommandMessage_t::ABORT, 0);
        vTaskDelayMS(1000);
        send_command(DO_PIROUETTE, 1);
        wait_for_motion();

        // LETTUCE _________________
        send_command(FOLLOW_WALL_TO, 2);
        wait_for_motion();
        open_claw(ServoPositionsPercentage_t::VERTICAL_HEIGHT_2);
        grab_with_rack_and_claw(ServoPositionsPercentage_t::CLAW_CLOSED_CHEESE);

        // CHEESE _________________
        send_command(FOLLOW_WALL_TO, 1);
        wait_for_motion();
        open_claw(ServoPositionsPercentage_t::VERTICAL_HEIGHT_1);
        grab_with_rack_and_claw(ServoPositionsPercentage_t::CLAW_CLOSED_CHEESE);

        // PIROUETTE _________________
        send_command(FOLLOW_WALL_TO, 2);
        vTaskDelayMS(1000);
        send_command(CommandMessage_t::ABORT, 0);
        vTaskDelayMS(1000);
        send_command(DO_PIROUETTE, 2);
        wait_for_motion();

        if (use_wifi) {
            while (!action_ready) {
                log_status("Waiting for patty to be ready!");
                vTaskDelayMS(50);
            }
            action_ready = false;
            log_status("Patty is ready!");
        }

        // PATTY _________________
        send_command(FOLLOW_WALL_TO, 3);
        wait_for_motion();
        open_claw(ServoPositionsPercentage_t::VERTICAL_HEIGHT_3);
        grab_with_rack_and_claw(ServoPositionsPercentage_t::CLAW_CLOSED_PATTY);

        // DROP ON PLATE _________
        send_command(FOLLOW_WALL_TO, 4);
        wait_for_motion();
        open_claw(ServoPositionsPercentage_t::VERTICAL_UP);
    

        // SWITCHING    _______________
        send_command(DO_PIROUETTE, -3);
        wait_for_motion();

        if (use_wifi) {
            while (!action_ready) {
                log_status("Waiting for top bun to be ready!");
                vTaskDelayMS(50);
            }
            action_ready = false;
            log_status("Top bun is ready!");
        }

        // GRAB PLATE   _______________
        send_command(FOLLOW_WALL_TO, 4);
        wait_for_motion();
        actuate_claw_forwards();
        grab_plate();

        // SERVE FOOD   _______________
        serve_food();

        while (1) {
            vTaskDelay(1000);
        }

    }
}

void setup() {
    Serial.begin(115200); // Initialize serial monitor

    init_pwm();

    init_rack_and_pinion(RACK_FORWARD_PIN, RACK_REVERSE_PIN, 1, SWITCH_RACK_PLATESIDE, SWITCH_RACK_CLAWSIDE);

    Serial.println("servos initialized!");

    claw_servo = instantiate_servo_motor(SERVO_CLAW_PIN, SERVO_CLAW_OPEN, SERVO_CLAW_CLOSED);
    draw_bridge_servo = instantiate_servo_motor(SERVO_DRAW_BRIDGE_PIN, SERVO_DRAW_BRIDGE_UP, SERVO_DRAW_BRIDGE_DOWN);
    plating_servo = instantiate_servo_motor(SERVO_PLATE_PIN, SERVO_PLATE_OPEN, SERVO_PLATE_CLOSED);
    vertical_servo = instantiate_servo_motor(SERVO_VERTICAL_PIN, SERVO_VERTICAL_DOWN, SERVO_VERTICAL_UP);
    
    delay(1000);
    
    Serial.println("vertical servo go up!");
    set_servo_position_percentage(vertical_servo, 0);

    initialize_uart(&uart_msg_queue);

    xTaskCreate(uart_msg_handler, "UART_msg_handler", 2048, NULL, 1, NULL);

    while (!MOTION_READY) {
        log_status("Trying to connect to motion...");
        send_uart_message(CommandMessage_t::READY, 0);
        delay(300);
    }

    log_status("Connected to motion board!");

    if (use_wifi) {
        init_wifi();

        xTaskCreate(wifi_msg_handler, "WiFi_msg_handler", 2048, NULL, 1, NULL);

        delay(500);

        while (!wifi_ready) {
            send_wifi_message(CommandMessage_t::READY, 0);
            log_status("Trying to handshake WiFi...");
            delay(500);
        }

        log_status("Connected to WiFi!");
    }

    xTaskCreate(TaskMaster, "Master", 2048, NULL, 1, NULL);
}

void loop() {

}
