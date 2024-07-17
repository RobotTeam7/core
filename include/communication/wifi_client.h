#ifndef WIFI_CLIENT_H
#define WIFI_CLIENT_H

#include <communication/wifi.h>

extern WiFiClient client;

void connect_to_wifi_as_client(WiFiHandler_t* wifiHandler);

#endif