#include <communication/wifi_client.h>


WiFiClient client;

void client_wifi_listen_task(void *pvParameters) {
    WiFiHandler_t* wifiHandler = (WiFiHandler_t*)pvParameters; 
    if (checkWiFiHandler(wifiHandler)) {
        log_error("WiFi handler buffer contains nulls!");
        vTaskDelete(NULL);
        return;
    }

    if (VERBOSITY_LEVEL >= MOST_VERBOSE) {
        Serial.println("SSID: " + String(wifiHandler->wifi_config->ssid));
        Serial.println("Host: " + String(wifiHandler->wifi_config->host));
        Serial.println("Password: " + String(wifiHandler->wifi_config->password));
    }

    while (WiFi.status() != WL_CONNECTED) {
        delay(200);
        log_status("Connecting to WiFi...");
    }

    if (!client.connect(wifiHandler->wifi_config->host, port)) {
        Serial.println("Connection to server failed!");
        vTaskDelete(NULL);
    }

    Serial.println("Connected to server");

    while (1) {
        if (client.connected() && (client.available() >= 2)) {
            uint8_t byte1 = client.read();
            uint8_t byte2 = client.read();

            Serial.println("Reading...");
            Serial.println(byte1);
            Serial.println(byte2);
            Serial.println("Read!");

            WiFiPacket_t new_packet = { byte1, byte2 };

            xQueueSend(*wifiHandler->inbound_wifi_queue, &new_packet, portMAX_DELAY);
        }
        vTaskDelay(10 / portTICK_PERIOD_MS); // Small delay to yield
    }
}


void client_wifi_sender_task(void *pvParameters) {
    WiFiHandler_t* wifiHandler = (WiFiHandler_t*)pvParameters; 
    if (checkWiFiHandler(wifiHandler)) {
        log_error("WiFi handler buffer contains nulls!");
        vTaskDelete(NULL);
        return;
    }
    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.println("Connecting to WiFi...");
    }
    Serial.println("Connected to WiFi");

    if (!client.connect(wifiHandler->wifi_config->host, port)) {
        Serial.println("Connection to server failed!");
        vTaskDelete(NULL);
    }

    Serial.println("Connected to server");

    while (1) {
        WiFiPacket_t wifiPacket;
        uint8_t bytes[2];
        if (xQueueReceive(*wifiHandler->outbound_wifi_queue, &wifiPacket, portMAX_DELAY)) {
            bytes[0] = wifiPacket.byte1;
            bytes[1] = wifiPacket.byte2;
            client.write(bytes, 2);
        }
        vTaskDelay(10 / portTICK_PERIOD_MS); // Small delay to yield
    }
}


void connect_to_wifi_as_client(WiFiHandler_t* wifiHandler)  {
    // Connect to the access point
    Serial.println("Starting WiFi connection on:");
    Serial.println("SSID: " + String(wifiHandler->wifi_config->ssid));
    Serial.println("Host: " + String(wifiHandler->wifi_config->host));
    Serial.println("Password: " + String(wifiHandler->wifi_config->password));

    WiFi.begin(wifiHandler->wifi_config->ssid, wifiHandler->wifi_config->password);

    if (checkWiFiHandler(wifiHandler)) {
        log_error("Nulls in WiFi handler buffer!");
        return;
    }

    // Create the client task
    xTaskCreate(client_wifi_listen_task, "Client_Wifi_Listen", 4096, (void*)wifiHandler, 1, NULL);
    // xTaskCreate(client_wifi_sender_task, "Client_Wifi_Sender", 4096, (void*)wifiHandler, 1, NULL);
}
