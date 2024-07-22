#include <communication/uart.h>


void initialize_uart() {
    Serial2.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);
}

void receiveData(void *parameter) {
    QueueHandle_t* uart_msg_queue = (QueueHandle_t*)parameter;
    for (;;) {
        if (Serial2.available() >= 2) {
            uint8_t value1 = Serial2.read(); // Read the first byte
            uint8_t value2 = Serial2.read(); // Read second byte

            // Serial.print("Received ");
            // Serial.print(value1);
            // Serial.println(value2);

            CommandMessage_t command = decode_command(value1);
            Packet_t new_packet = { (CommandMessage_t)value1, value2 };

            if (command == GOTO) {
                Serial.print("Commanded to goto ");
                Serial.println(String(value2));
            }

            xQueueSend(*uart_msg_queue, &new_packet, portMAX_DELAY);
        }
        vTaskDelay(50 / portTICK_PERIOD_MS); // Check for data every 100ms
    }
}

void begin_uart_read(QueueHandle_t* uart_msg_queue) {
    log_status("Beginning to read UART");
    xTaskCreate(receiveData, "ReceiveUART", 1024, (void*)uart_msg_queue, 1, NULL);
}

void send_uart_message(CommandMessage_t message, uint8_t value) {
    log_message("Sending message over UART!");
    Serial2.write(message);
    Serial2.write(value);
}