#include <communication/uart.h>

void initialize_uart() {
    Serial2.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);
}
