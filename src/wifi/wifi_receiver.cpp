#include <wifi/wifi_receiver.h>


void clientTask(void *pvParameters) {
  WiFiConfig* config = (WiFiConfig*)pvParameters; 

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


Receiver::Receiver(WiFiConfig* config) {
  this->config = config;
}


void Receiver::begin_wifi() {
  // Connect to the access point
  Serial.println("Starting WiFi connection on:");
  Serial.println("SSID: " + String(config->ssid));
  Serial.println("Host: " + String(config->host));
  Serial.println("Password: " + String(config->password));

  WiFi.begin(config->ssid, config->password);

  // Create the client task
  xTaskCreate(clientTask, "Client Task", 4096, (void*)this->config, 1, NULL);

}