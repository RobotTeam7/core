#include <wifi/wifi_sender.h>

void serverTask(void *pvParameters) {
  server.begin();
  
  while (true) {
    WiFiClient client = server.available(); // Listen for incoming clients
    if (client) {
      Serial.println("New Client.");
      String currentLine = "";
      while (client.connected()) {
        if (client.available()) {
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

void wifi_sender::begin_server() {
  // Set up the access point
  WiFi.softAP(ssid, password);
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);

  // Create the server task
  xTaskCreate(serverTask, "Server Task", 4096, NULL, 1, NULL);
}