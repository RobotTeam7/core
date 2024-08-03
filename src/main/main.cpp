#include <main/main.h>


QueueHandle_t outboundWiFiQueue = xQueueCreate(10, sizeof(WiFiPacket_t));
QueueHandle_t inboundWiFiQueue = xQueueCreate(10, sizeof(WiFiPacket_t));
QueueHandle_t uart_msg_queue = xQueueCreate(10, sizeof(Packet_t));

WiFiHandler_t wifi_handler = {
    .wifi_config = &wifi_config,
    .inbound_wifi_queue = &inboundWiFiQueue,
    .outbound_wifi_queue = &outboundWiFiQueue
};

bool MOTION_BUSY = false;
bool MOTION_READY = false;

ServoMotor_t* claw_servo;
ServoMotor_t* vertical_servo;

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
        vTaskDelayMS(10); // Small delay to yield
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
        vTaskDelayMS(10); // Small delay to yield    
    }
}

void grab_with_claw(int claw_percentage) {
    log_status("vertical servo down");
    set_servo_position_percentage(vertical_servo, ServoPositionsPercentage_t::VERTICAL_DOWN);
    vTaskDelayMS(SERVO_ACTUATION_DELAY);

    // close servo
    log_status("claw servo closed");
    set_servo_position_percentage(claw_servo, claw_percentage);
    vTaskDelayMS(SERVO_ACTUATION_DELAY);

    log_status("vertical servo up");
    set_servo_position_percentage(vertical_servo, ServoPositionsPercentage_t::VERTICAL_UP);
}

void open_claw(ServoPositionsPercentage_t percentage) {
    log_status("open claw");
    set_servo_position_percentage(vertical_servo, percentage);
    vTaskDelayMS(SERVO_ACTUATION_DELAY);
    set_servo_position_percentage(claw_servo, ServoPositionsPercentage_t::CLAW_OPEN);
    vTaskDelayMS(SERVO_ACTUATION_DELAY);
    set_servo_position_percentage(vertical_servo, ServoPositionsPercentage_t::VERTICAL_UP);
}

void wait_for_motion() {
    while (MOTION_BUSY) {
        vTaskDelayMS(10);
    }
    // vTaskDelay(pdMS_TO_TICKS(UART_INTERMESSAGE_DELAY));
}

void send_command(CommandMessage_t command, int8_t value) {
    send_uart_message(command, value);
    MOTION_BUSY = true;
}