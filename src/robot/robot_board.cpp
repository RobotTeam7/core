#ifdef ROBOT_BOARD

#include <Arduino.h>
#include <WiFi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

const char* ssid = "UniqueESP32_AP";
const char* password = "12345678";
const char* host = "192.168.4.1"; // This should be the IP address of the ESP32 AP

void clientTask(void *pvParameters) {
  while (WiFi.status() != WL_CONNECTED) {
    delay(200);
    Serial.println("Connecting to WiFi...");
    Serial.println("Connecting to WiFi...");
  }

  while (true) {
    WiFiClient client;
    if (client.connect(host, 80)) {
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

void setup() {
  Serial.begin(115200);

  // Connect to the access point
  WiFi.begin(ssid, password);

  // Create the client task
  xTaskCreate(clientTask, "Client Task", 4096, NULL, 1, NULL);
}

void loop() {
  // Nothing to do here, as tasks are handled by FreeRTOS
}

#endif