#include <robot/robot.h>


QueueHandle_t uart_msg_queue = xQueueCreate(10, sizeof(Packet_t));

bool MOTION_BUSY = false;
bool MOTION_READY = false;

bool wifi_ready = false;

bool action_ready = false;
bool block_action = false;

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

                case ACK:
                {
                    MOTION_READY = true;
                    break;
                }
            }
        }
        vTaskDelayMS(10); // Small delay to yield
    }
}

void init_communications(uint8_t tx_pin, uint8_t rx_pin) {
    initialize_uart(&uart_msg_queue, tx_pin, rx_pin);

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
}

void wifi_msg_handler(void *parameter) {
    log_status("Started message handler");
    while (1) {
        Packet_t new_packet;
        if (xQueueReceive(wifi_message_queue, &new_packet, portMAX_DELAY)) {
            Serial.println("Received wifi packet...");
            Serial.println(new_packet.command);
            Serial.println(new_packet.value);

            switch (new_packet.command) {
                case READY:
                {
                    send_wifi_message(CommandMessage_t::ACK, 0);
                    log_status("Readied WiFi!");
                    break;
                }

                case NEXT_ACTION: 
                {
                    action_ready = true;
                    log_status("Ready for next action!");
                    break;
                }

                case WAIT_FOR_ACTION:
                {
                    block_action = true;
                    log_status("Blocking action!");
                    break;
                }

                case ACK:
                {
                    wifi_ready = true;
                    log_status("Readiness acknowledged!");
                    break;
                }
            }
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
}

void send_command(CommandMessage_t command, int8_t value) {
    send_uart_message(command, value);
    MOTION_BUSY = true;
}
