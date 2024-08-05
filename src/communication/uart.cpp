#include <communication/uart.h>


#define UART_BAUD_RATE  9600
#define UART_PORT       UART_NUM_2
#define MAX_RETRIES     4

static QueueHandle_t uart_queue;
const int uart_buffer_size = 128;
Packet_t last_sent_packet;
int retries = 0;

static void uart_receive_event_task(void *pvParameters) {
    QueueHandle_t* uart_msg_queue = (QueueHandle_t*)pvParameters;
    if (uart_msg_queue == NULL) {
        log_error("UART message queue is null!");
        vTaskDelete(NULL);
        return;
    }

    uart_event_t event;
    uint8_t data_buffer[uart_buffer_size];
    Packet_t new_packet;
    uint8_t data[2];
    memset(data_buffer, 0, uart_buffer_size);

    while (1) {
        // Waiting for UART event
        int length = 0;
        if (xQueueReceive(uart_queue, (void*)&event, portMAX_DELAY)) {
            switch (event.type) {
                case UART_DATA:
                {
                    // Read data from the UART
                    length = uart_read_bytes(UART_PORT, data_buffer, event.size, portMAX_DELAY);

                    if (length % 8 != 0) {
                        Serial.println("Warning! Read " + String(length) + " bytes from buffer!");
                    }

                    int num_messages = length / 8;
                    uint8_t* data_pointer = data_buffer; // Pointer into the buffer that will increment by eight bytes for every message processed
                    
                    for (int i = 0; i < num_messages; i++) {
                        // Decompose UART message into constituent parts as described by COMMUNICATION PROTOCOL
                        bool invalid_message = false;

                        uint8_t start_byte = data_pointer[0];
                        uint8_t command_byte = data_pointer[1];
                        uint8_t value_byte = data_pointer[2];
                        uint32_t crc_value;
                        memcpy(&crc_value, data_pointer + sizeof(uint8_t) * 3, sizeof(crc_value));
                        uint8_t stop_byte = data_pointer[sizeof(uint8_t) * 3 + sizeof(crc_value)];

                        if (start_byte != START_BYTE) {
                            log_error("Start byte is invalid!");
                            invalid_message = true;
                        }

                        if (stop_byte != STOP_BYTE) {
                            log_error("Stop byte is invalid!");
                            invalid_message = true;
                        }

                        data[0] = command_byte;
                        data[1] = value_byte;

                        if (esp_crc32_le(0, data, sizeof(data)) != crc_value) {
                            log_error("CRC is invalid!");
                            invalid_message = true;
                        }

                        if (invalid_message) {
                            send_nack();
                            continue;
                        }

                        switch (command_byte) {
                            case UART_ACK_BEGIN ... UART_ACK_END: // Indicates we received an ACK message
                            {
                                log_status("Received acknowledgmenet!");
                                retries = 0;
                            }   // INTENTIONALLY FALL THROUGH

                            case UART_CMD_BEGIN ... UART_CMD_END: // Indicates that there is a command that should go on the command queue
                            {
                                log_status("Putting new packet onto queue!");
                                new_packet.command = (CommandMessage_t)command_byte;
                                new_packet.value = value_byte;
                                xQueueSend(*uart_msg_queue, &new_packet, portMAX_DELAY);
                                break;
                            }

                            case UART_NACK_BEGIN ... UART_NACK_END: // Indicates we received a NACK
                            {
                                log_status("Received NACK!");
                                
                                if (retries < MAX_RETRIES) {
                                    // Wait for a moment, hopefully for the noise to subside
                                    vTaskDelay(pdMS_TO_TICKS(500));

                                    // Try to resend the most recent transmission
                                    send_uart_message(last_sent_packet.command, last_sent_packet.value, false);
                                    retries++;

                                    break;
                                } else { // Initiate fault state for max retries
                                    while (1) {
                                        vTaskDelay(pdMS_TO_TICKS(500));
                                        Serial.println("FAULT STATE: MAX UART RETRIES REACHED!");
                                    }
                                }
                            }
                        }

                        data_pointer += sizeof(uint8_t) * 8;
                    }     

                    memset(data_buffer, 0, uart_buffer_size);  // Reset the buffer
                    break;
                }

                case UART_FIFO_OVF:
                {
                    Serial.println("FIFO buffer fail!");

                    // Handle overflow
                    uart_flush_input(UART_PORT);
                    xQueueReset(uart_queue);

                    break;
                }
                    
                case UART_BUFFER_FULL:
                {
                    Serial.println("Ring buffer fail!");

                    uart_flush_input(UART_PORT);
                    xQueueReset(uart_queue);

                    break;
                }
                
                case UART_BREAK:
                {
                    Serial.println("UART Break!");

                    break;
                }

                case UART_PARITY_ERR:
                {
                    Serial.println("Parity error!");

                    break;
                }

                case UART_FRAME_ERR:
                {
                    Serial.println("Frame error!");

                    break;
                }

                default:
                {
                    Serial.println("Default!");

                    break;
                }
            }
        }
    }
}

static void send_nack() {
    if (xTaskGetCurrentTaskHandle() == NULL) { // If we're not in a task, use Arduino delay
        delay(500);
    } else {
        vTaskDelay(pdMS_TO_TICKS(500));        // Otherwise, just block the task as normal
    }
    send_uart_message(CommandMessage_t::NACK, 0, false);
}

void send_uart_message(CommandMessage_t command, uint8_t value, bool memorize) {
    log_message("Sending message over UART!");

    if (memorize) {
        log_status("Memorizing packet!");
        last_sent_packet.command = command;
        last_sent_packet.value = value;
    }

    uint8_t* message_buffer = encode_message(command, value);
    
    uart_write_bytes(UART_PORT, (const char*)message_buffer, MESSAGE_SIZE);
    free(message_buffer);

    log_status("Sent message!");
}

void initialize_uart(QueueHandle_t* packet_queue, uint8_t tx_pin, uint8_t rx_pin) {
    uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,            // Slow baud rate as we aren't transmitting that much data
        .data_bits = UART_DATA_8_BITS,          // One byte per frame
        .parity = UART_PARITY_EVEN,             // Enable even parity checking
        .stop_bits = UART_STOP_BITS_1,          // Just one stop bit as our baud rate is pretty slow 
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE   // No flow control, as we are not sending more than a few bytes at a time
    };

    // Configure UART parameters
    ESP_ERROR_CHECK(uart_param_config(UART_PORT, &uart_config));

    // Set the pins (UART_PIN_NO_CHANGE for RTS and CTS as we aren't using them)
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT, tx_pin, rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    // Setup drivers
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT, uart_buffer_size * 2, uart_buffer_size * 2, 10, &uart_queue, 0));
    
    // Create receive task
    xTaskCreate(uart_receive_event_task, "uart_receive_event_task", 4096, packet_queue, 4, NULL);
    
    log_status("UART initialized!");
}
