#ifndef DECODE_COMMUNICATION_H
#define DECODE_COMMUNICATION_H

#include <Arduino.h>
#include <communication/uart.h>

/**
 * @brief Decode a byte into a CommandMessage_t
 */
CommandMessage_t decode_command(uint8_t message_identifier);

#endif