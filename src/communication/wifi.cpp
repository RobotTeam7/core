#include <communication/wifi.h>


QueueHandle_t wifi_message_queue = xQueueCreate(10, sizeof(CommandMessage_t));

const uint8_t mac_address[6] = PEER_MAC_ADDRESS;

void on_data_receive(const uint8_t *mac, const uint8_t *incoming_data, int len) {
    Serial.print("ESP-NOW Bytes received: ");
    Serial.println(len);
    Message_t message;
    decode_message(incoming_data, &message);
    Serial.println("Command: " + String(message.command) + " " + String(message.value));
    
    Packet_t packet = { message.command, message.value };
    xQueueSend(wifi_message_queue, &packet, portMAX_DELAY);

    log_status("Put message onto queue!");
}

void init_wifi() {
    WiFi.mode(WIFI_STA);
    Serial.println(WiFi.macAddress());


    // Init ESP-NOW
    if (esp_now_init() != ESP_OK) {
        while (1) {
            Serial.println("Error initializing ESP-NOW!");
            delay(500);
        }
    }

    // Register peer
    esp_now_peer_info_t peerInfo;
    memset(&peerInfo, 0, sizeof(peerInfo));
    memcpy(peerInfo.peer_addr, mac_address, 6);
    peerInfo.channel = 0;  
    peerInfo.encrypt = false;

    // Add peer
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        while (1) {
            Serial.println("Failed to add ESP-NOW Peer!");
            delay(500);
            Serial.println("Retrying...");
        }    
    }

    // Add the receive function
    esp_now_register_recv_cb(on_data_receive);

    log_status("ESP-NOW is initialized!");
}

void send_wifi_message(CommandMessage_t command, int8_t value) {
    uint8_t* message_buffer = encode_message(command, value);

    while (1) {
        esp_err_t result = esp_now_send(mac_address, message_buffer, sizeof(MESSAGE_SIZE));

        if (result == ESP_OK) {
            Serial.println("Message sent successfully");
            break;
        } else {
            Serial.printf("Error sending the message: %d\n", result);
            delay(500);
        }
    }
}
