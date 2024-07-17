#ifndef WIFI_SERVER_H
#define WIFI_SERVER_H

#include <communication/wifi.h>
#include <communication/wifi_client.h>

extern WiFiServer server;

void begin_wifi_as_server(WiFiHandler_t* wifiHandler);


#endif