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

static inline void wait_for_motion() {
    while (MOTION_BUSY) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    vTaskDelay(pdMS_TO_TICKS(UART_INTERMESSAGE_DELAY));
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

        log_status("dock on side");
        send_command(COUNTER_DOCK, 1);
        wait_for_motion();

        log_status("wall slam to 2");
        send_command(FOLLOW_WALL_TO, 2);
        wait_for_motion();

        grab_with_claw(ServoPositionsPercentage_t::CLAW_CLOSED_BUN);

        log_status("wall slam to 4");
        send_command(FOLLOW_WALL_TO, 4);
        wait_for_motion();

        log_status("claw servo closed");
        set_servo_position_percentage(claw_servo, ServoPositionsPercentage_t::CLAW_OPEN);
        vTaskDelay(pdMS_TO_TICKS(SERVO_ACTUATION_DELAY));

        log_status("wall slam to 3");
        send_command(FOLLOW_WALL_TO, 3);
        wait_for_motion();

        grab_with_claw(ServoPositionsPercentage_t::CLAW_CLOSED_PATTY);

        log_status("wall slam to 4");
        send_command(FOLLOW_WALL_TO, 4);
        wait_for_motion();

        set_servo_position_percentage(claw_servo, ServoPositionsPercentage_t::CLAW_OPEN);
        vTaskDelay(pdMS_TO_TICKS(SERVO_ACTUATION_DELAY));

        log_status("wall slam to 3");
        send_command(FOLLOW_WALL_TO, 3);
        wait_for_motion();

        log_status("pirouette");
        send_command(DO_PIROUETTE, 1);
        wait_for_motion();

        log_status("wall slam to 2");
        send_command(FOLLOW_WALL_TO, 2);
        wait_for_motion();
        
        grab_with_claw(ServoPositionsPercentage_t::CLAW_CLOSED_BUN);

        log_status("pirouette");
        send_command(DO_PIROUETTE, 3);
        wait_for_motion();

        log_status("wall slam to 4");
        send_command(FOLLOW_WALL_TO, 4);
        wait_for_motion();

        log_status("claw servo open");
        set_servo_position_percentage(claw_servo, ServoPositionsPercentage_t::CLAW_OPEN);
        vTaskDelay(pdMS_TO_TICKS(SERVO_ACTUATION_DELAY));

        Serial.println("Done!");
        while (1) {
            vTaskDelay(1000);
        }
    }
}

void setup() {
    Serial.begin(115200); // Initialize serial monitor

    init_pwm();

    // instantiate_limit_switch(SWITCH_RACK_CLAWSIDE, test_isr_0);
    // instantiate_limit_switch(SWITCH_RACK_PLATESIDE, test_isr_1);

    init_rack_and_pinion(RACK_FORWARD_PIN, RACK_REVERSE_PIN, -1, SWITCH_RACK_PLATESIDE, SWITCH_RACK_CLAWSIDE);
    
    actuate_claw_forwards();
    // Serial.println("servos initialized!");

    claw_servo = instantiate_servo_motor(SERVO_CLAW_PIN, SERVO_CLAW_OPEN, SERVO_CLAW_CLOSED);
    draw_bridge_servo = instantiate_servo_motor(SERVO_DRAW_BRIDGE_PIN, SERVO_DRAW_BRIDGE_UP, SERVO_DRAW_BRIDGE_DOWN);
    plating_servo = instantiate_servo_motor(SERVO_PLATE_PIN, SERVO_PLATE_OPEN, SERVO_PLATE_CLOSED);
    vertical_servo = instantiate_servo_motor(SERVO_VERTICAL_PIN, SERVO_VERTICAL_DOWN, SERVO_VERTICAL_UP);
    
    delay(1000);
    
    Serial.println("vertical servo go up!");
    set_servo_position_percentage(vertical_servo, 0);

    // connect_to_wifi_as_client(&wifi_handler);

    // initialize_uart(&uart_msg_queue);

    xTaskCreate(uart_msg_handler, "UART_msg_handler", 2048, NULL, 1, NULL);

    delay(100);
    // xTaskCreate(wifi_msg_handler, "WiFi_msg_handler", 2048, NULL, 1, NULL);
    
    // xTaskCreate(TaskMaster, "Master", 2048, NULL, 1, NULL);
}

void loop() {
    // delay(1500);
    // Serial.print("Free heap: ");
    // Serial.println(ESP.getFreeHeap());
    // delay(1000); // Check every second
    Serial.println("here in main board");
    delay(10000);
}
