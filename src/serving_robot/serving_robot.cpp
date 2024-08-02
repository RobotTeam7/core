#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <serving_robot/constants.h>
#include <serving_robot/rack_and_pinion.h>

#include <common/resource_manager.h>
#include <common/servo_motor.h>
#include <common/robot_motor.h>
#include <common/pwm.h>
#include <common/limit_switch.h>

#include <communication/wifi_client.h>
#include <communication/uart.h>
#include <communication/decode.h>

#define SERVO_ACTUATION_DELAY 500
#define UART_INTERMESSAGE_DELAY 50


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

static inline void grab_with_claw(int claw_percentage) {
    log_status("vertical servo down");
    set_servo_position_percentage(vertical_servo, ServoPositionsPercentage_t::VERTICAL_DOWN);
    vTaskDelay(pdMS_TO_TICKS(SERVO_ACTUATION_DELAY));

    // close servo
    log_status("claw servo closed");
    set_servo_position_percentage(claw_servo, claw_percentage);
    vTaskDelay(pdMS_TO_TICKS(SERVO_ACTUATION_DELAY));

    log_status("vertical servo up");
    set_servo_position_percentage(vertical_servo, ServoPositionsPercentage_t::VERTICAL_UP);
}

// assumes plating servo is open initially
static inline void grab_plate() {
    log_status("draw bridge down");
    set_servo_position_percentage(draw_bridge_servo, ServoPositionsPercentage_t::DRAW_BRIDGE_DOWN);
    vTaskDelay(pdMS_TO_TICKS(SERVO_ACTUATION_DELAY));
    
    log_status("plate servo close");
    set_servo_position_percentage(plating_servo, ServoPositionsPercentage_t::PLATE_CLOSED);
    vTaskDelay(pdMS_TO_TICKS(SERVO_ACTUATION_DELAY));
}

static inline void open_claw(ServoPositionsPercentage_t percentage) {
    log_status("open claw");
    set_servo_position_percentage(vertical_servo, percentage);
    vTaskDelay(pdMS_TO_TICKS(SERVO_ACTUATION_DELAY));
    set_servo_position_percentage(claw_servo, ServoPositionsPercentage_t::CLAW_OPEN);
    vTaskDelay(pdMS_TO_TICKS(SERVO_ACTUATION_DELAY));
    set_servo_position_percentage(vertical_servo, ServoPositionsPercentage_t::VERTICAL_UP);
}

static inline void wait_for_motion() {
    while (MOTION_BUSY) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    // vTaskDelay(pdMS_TO_TICKS(UART_INTERMESSAGE_DELAY));
}

static inline void send_command(CommandMessage_t command, int8_t value) {
    send_uart_message(command, value);
    MOTION_BUSY = true;
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

        // GET TO COUNTER ________
        log_status("dock on side");
        send_command(COUNTER_DOCK, 1);
        wait_for_motion();
        send_command(FOLLOW_WALL_TO, 2);
        vTaskDelay(pdMS_TO_TICKS(200));
        send_command(ABORT, 0);
        send_command(FOLLOW_WALL_TO, 1);
        wait_for_motion();


        // TOMATO _________________
        log_status("getting tomato");
        send_command(FOLLOW_WALL_TO, 1);
        wait_for_motion();
        grab_with_claw(ServoPositionsPercentage_t::CLAW_CLOSED_TOMATO);

        // PIROUETTE _________________
        send_command(FOLLOW_WALL_TO, 4);
        vTaskDelay(pdMS_TO_TICKS(500));
        send_command(CommandMessage_t::ABORT, 0);
        vTaskDelay(pdMS_TO_TICKS(500));
        send_command(DO_PIROUETTE, 2);
        wait_for_motion();

        // CHEESE _________________
        send_command(FOLLOW_WALL_TO, 1);
        wait_for_motion();
        open_claw(ServoPositionsPercentage_t::VERTICAL_HEIGHT_1);
        grab_with_claw(ServoPositionsPercentage_t::CLAW_CLOSED_CHEESE);


        // LETTUCE _________________
        send_command(FOLLOW_WALL_TO, 2);
        wait_for_motion();
        open_claw(ServoPositionsPercentage_t::VERTICAL_HEIGHT_2);
        grab_with_claw(ServoPositionsPercentage_t::CLAW_CLOSED_CHEESE);

        // PIROUETTE _________________
        send_command(FOLLOW_WALL_TO, 1);
        vTaskDelay(pdMS_TO_TICKS(750));
        send_command(CommandMessage_t::ABORT, 0);
        vTaskDelay(pdMS_TO_TICKS(500));
        send_command(DO_PIROUETTE, 2);
        wait_for_motion();

        // PATTY _________________
        send_command(FOLLOW_WALL_TO, 3);
        wait_for_motion();
        open_claw(ServoPositionsPercentage_t::VERTICAL_HEIGHT_3);
        grab_with_claw(ServoPositionsPercentage_t::CLAW_CLOSED_PATTY);

        // DROP ON PLATE _________
        send_command(FOLLOW_WALL_TO, 4);
        wait_for_motion();
        open_claw(ServoPositionsPercentage_t::VERTICAL_HEIGHT_3);

        // SWITCHING _______________
        send_command(FOLLOW_WALL_TO, 3);
        wait_for_motion();
        send_command(DO_PIROUETTE, -2);
        wait_for_motion();

        // GRAB PLATE _______________
        send_command(FOLLOW_WALL_TO, 4);
        wait_for_motion();

        actuate_claw_forwards();

        grab_plate();

        send_uart_message(CommandMessage_t::SET_MULTIPLIER, 35);

        // SERVING  _______________
        set_servo_position_percentage(draw_bridge_servo, 15);
        send_command(FOLLOW_WALL_TO, 3);
        wait_for_motion();
        send_command(DO_PIROUETTE, 1);
        wait_for_motion();
        send_command(FOLLOW_WALL_TO, 2);
        wait_for_motion();
        send_command(FOLLOW_WALL_TO, 1);
        vTaskDelay(pdMS_TO_TICKS(1050));
        send_command(ABORT, 0);
        vTaskDelay(pdMS_TO_TICKS(1000));
        set_servo_position_percentage(plating_servo, ServoPositionsPercentage_t::PLATE_OPEN);
        vTaskDelay(pdMS_TO_TICKS(SERVO_ACTUATION_DELAY));
        set_servo_position_percentage(draw_bridge_servo, ServoPositionsPercentage_t::DRAW_BRIDGE_UP);



        Serial.println("Done!");
        while (1) {
            vTaskDelay(1000);
        }
    }
}

void setup() {
    Serial.begin(115200); // Initialize serial monitor

    init_pwm();

    init_rack_and_pinion(RACK_FORWARD_PIN, RACK_REVERSE_PIN, -1, SWITCH_RACK_PLATESIDE, SWITCH_RACK_CLAWSIDE);

    Serial.println("servos initialized!");

    claw_servo = instantiate_servo_motor(SERVO_CLAW_PIN, SERVO_CLAW_OPEN, SERVO_CLAW_CLOSED);
    draw_bridge_servo = instantiate_servo_motor(SERVO_DRAW_BRIDGE_PIN, SERVO_DRAW_BRIDGE_UP, SERVO_DRAW_BRIDGE_DOWN);
    plating_servo = instantiate_servo_motor(SERVO_PLATE_PIN, SERVO_PLATE_OPEN, SERVO_PLATE_CLOSED);
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
