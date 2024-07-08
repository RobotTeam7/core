#include <wifi/wifi_sender.h>


WiFiServer server(80);

void serverTask(void *pvParameters) {
  WiFiConfig* config = (WiFiConfig*)pvParameters; 

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

Sender::Sender(WiFiConfig* config) {
  this->config = config;
}

void Sender::begin_server() {
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
  xTaskCreate(serverTask, "Server Task", 4096, (void*)this->config, 1, NULL);
}