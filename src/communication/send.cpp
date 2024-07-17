#include <communication/send.h>


void send_uart_message(CommandMessage_t message, uint8_t value) {
    log_message("Sending message over UART!");
    Serial2.write(message);
    Serial2.write(value);
}

WiFiServer server(80);

void serverTask(void *pvParameters) {
  WiFiConfig_t* config = (WiFiConfig_t*)pvParameters; 

  Serial.println("SSID: " + String(config->ssid));
  Serial.println("Host: " + String(config->host));
  Serial.println("Password: " + String(config->password));

  server.begin();
  
  while (true) {
    // Listen for incoming connections
    WiFiClient client = server.available();
    
    // If we have a connection
    if (client) {
      Serial.println("New Client.");

      String currentLine = "";
      
      // While the client is connected
      while (client.connected()) {
        // While there is available bytes to be read
        if (client.available()) {
          // Read bytes, and print them to the serial monitor 
          char c = client.read();
          Serial.write(c);
          if (c == '\n') {
            if (currentLine.length() == 0) {
              // HTTP headers end with a blank line
              client.println("HTTP/1.1 200 OK");
              client.println("Content-type:text/html");
              client.println();
              // Return the response
              client.println("Hello from ESP32 server");
              client.println();
              break;
            } else {
              currentLine = "";
            }
          } else if (c != '\r') {
            currentLine += c;
          }
        }
      }
      client.stop();
      Serial.println("Client Disconnected.");
    }
    vTaskDelay(1); // Yield to other tasks
  }
}

void begin_wifi_server(const WiFiConfig_t* config) {
    // Set up the access point
    Serial.println("Starting WiFi AP on:");
    Serial.println("SSID: " + String(config->ssid));
    Serial.println("Host: " + String(config->host));
    Serial.println("Password: " + String(config->password));

    WiFi.softAP(config->ssid, config->password, 1, 0, 1);
    IPAddress IP = WiFi.softAPIP();
    
    Serial.print("AP IP address: ");
    Serial.println(IP);

    // Create the server task
    xTaskCreate(serverTask, "Server Task", 4096, (void*)config, 1, NULL);
}
