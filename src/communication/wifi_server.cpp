#include <communication/wifi_server.h>


WiFiServer server(port);

// Task to listen for incoming bytes and push them onto xQueue1
void server_wifi_listen_task(void *pvParameters) {
    WiFiHandler_t* wifiHandler = (WiFiHandler_t*)pvParameters; 
    if (checkWiFiHandler(wifiHandler)) {
        log_error("WiFi handler buffer contains nulls!");
        vTaskDelete(NULL);
        return;
    }

    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        log_status("Connecting to WiFi...");
    }

    log_status("Connected to WiFi");

    server.begin();

    while (1) {
        if (!client || !client.connected()) {
            client = server.available();
            if (client) {
                log_status("New Client Connected");
            }
        }

        if (client && client.connected() && (client.available() >= 2)) {
            uint8_t byte1 = client.read();
            uint8_t byte2 = client.read();

            Serial.println("Reading...");
            Serial.println(byte1);
            Serial.println(byte2);
            Serial.println("Read!");

            WiFiPacket_t* new_packet = (WiFiPacket_t*)malloc(sizeof(WiFiPacket_t));
            new_packet->byte1 = byte1;
            new_packet->byte2 = byte2;

            free(new_packet);
            // xQueueSend(*wifiHandler->inbound_wifi_queue, &byte1, portMAX_DELAY);
        }
        vTaskDelay(10 / portTICK_PERIOD_MS); // Small delay to yield
    }
}

// Task to watch xQueue2 for bytes and push them onto the network
void server_wifi_transmit_task(void *pvParameters) {
    WiFiHandler_t* wifiHandler = (WiFiHandler_t*)pvParameters; 
    if (checkWiFiHandler(wifiHandler)) {
        log_error("WiFi handler buffer contains nulls!");
        vTaskDelete(NULL);
        return;
    }

    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        log_status("Connecting to WiFi...");
    }
    log_status("Connected to WiFi");

    while (1) {
        if (client && client.connected()) {
            uint8_t bytes[2];
            if (xQueueReceive(*wifiHandler->outbound_wifi_queue, &bytes, portMAX_DELAY)) {
                client.write(bytes, 2);
            }
            vTaskDelay(10 / portTICK_PERIOD_MS); // Small delay to yield
        }
    }
}


void begin_wifi_as_server(WiFiHandler_t* wifiHandler) {
    // Set up the access point
    Serial.println("Starting WiFi AP on:");
    Serial.println("SSID: " + String(wifiHandler->wifi_config->ssid));
    Serial.println("Host: " + String(wifiHandler->wifi_config->host));
    Serial.println("Password: " + String(wifiHandler->wifi_config->password));

    WiFi.softAP(wifiHandler->wifi_config->ssid, wifiHandler->wifi_config->password, 1, 0, 1);
    IPAddress IP = WiFi.softAPIP();
    
    Serial.print("AP IP address: ");
    Serial.println(IP);

    if (checkWiFiHandler(wifiHandler)) {
        log_error("Nulls in WiFi handler buffer!");
        return;
    }

    // Create the server task
    xTaskCreate(server_wifi_listen_task, "Server_Listen", 4096, (void*)wifiHandler, 1, NULL);
    xTaskCreate(server_wifi_transmit_task, "Server_Send", 4096, (void*)wifiHandler, 1, NULL);

}