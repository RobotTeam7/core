#ifndef COMMUNICATION_PROTOCOL_H
#define COMMUNICATION_PROTOCOL_H

#include <communication/decode.h>
#include "esp_crc.h"


#define START_BYTE      (uint8_t)0x7E
#define STOP_BYTE       (uint8_t)0x7F
#define MESSAGE_SIZE    (sizeof(START_BYTE) + (2 * sizeof(uint8_t)) + sizeof(uint32_t) + sizeof(STOP_BYTE))

typedef struct {
    uint8_t start_byte;
    CommandMessage_t command;
    uint8_t value;
    uint32_t crc_checksum;
    uint8_t stop_byte;
} Message_t;

typedef struct {
    CommandMessage_t command;
    int8_t value;
} Packet_t;

uint8_t* encode_message(CommandMessage_t command, uint8_t value);
void decode_message(const uint8_t* data_pointer, Message_t* destination);


#endif // COMMUNICATION_PROTOCOL_H