#include <communication/read.h>


void receiveData(void *parameter) {
    for (;;) {
        if (Serial2.available() >= 2) {
            uint8_t value1 = Serial2.read(); // Read the first byte
            uint8_t value2 = Serial2.read(); // Read second byte

            Serial.print("Received ");
            Serial.print(value1);
            Serial.println(value2);

            Packet_t* new_packet = (Packet_t*)malloc(sizeof(Packet_t));
            CommandMessage_t command = decode_command(value1);

            new_packet->value = value2;
            new_packet->command = command;

            if (command == GOTO) {
                Serial.print("Commanded to goto ");
                Serial.println(String(value2));
            }

            free(new_packet); // or push it to a queue, somewhere
        }
        vTaskDelay(50 / portTICK_PERIOD_MS); // Check for data every 100ms
    }
}

void begin_uart_read() {
    log_status("Beginning to read UART");
    xTaskCreate(receiveData, "ReceiveUART", 1024, NULL, 1, NULL);
}

void clientTask(void *pvParameters) {
  WiFiConfig_t* config = (WiFiConfig_t*)pvParameters; 

  Serial.println("SSID: " + String(config->ssid));
  Serial.println("Host: " + String(config->host));
  Serial.println("Password: " + String(config->password));

  while (WiFi.status() != WL_CONNECTED) {
    delay(200);
    Serial.println("Connecting to WiFi...");
    Serial.println("Connecting to WiFi...");
  }

  while (true) {
    WiFiClient client;
    if (client.connect(config->host, 80)) {
      Serial.println("Connected to server");

      // Send a request to the server
      client.println("GET / HTTP/1.1");
      client.println("Host: ESP32_Client");
      client.println("Connection: close");
      client.println();

      // Read the response from the server
      while (client.connected() || client.available()) {
        if (client.available()) {
          String line = client.readStringUntil('\n');
          Serial.println(line);
        }
      }

      client.stop();
      Serial.println("Disconnected from server");
    } else {
      Serial.println("Connection to server failed");
    }

    vTaskDelay(5000 / portTICK_PERIOD_MS); // Wait 5 seconds before next request
  }
}


void connect_robot_wifi(const WiFiConfig_t* config)  {
    // Connect to the access point
    Serial.println("Starting WiFi connection on:");
    Serial.println("SSID: " + String(config->ssid));
    Serial.println("Host: " + String(config->host));
    Serial.println("Password: " + String(config->password));

    WiFi.begin(config->ssid, config->password);

    // Create the client task
    xTaskCreate(clientTask, "Client Task", 4096, (void*)config, 1, NULL);
}
