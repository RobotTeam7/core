#include <communication/communication.h>


void decode_message(const uint8_t* data_pointer, Message_t* destination) {
    destination->start_byte = data_pointer[0];
    destination->command = (CommandMessage_t)data_pointer[1];
    destination->value = data_pointer[2];
    memcpy(&destination->crc_checksum, data_pointer + sizeof(uint8_t) * 3, sizeof(destination->crc_checksum));
    destination->stop_byte = data_pointer[sizeof(uint8_t) * 3 + sizeof(destination->crc_checksum)];
}

/**
 * @returns Heap-allocated with malloc pointer of size MESSAGE_SIZE
 */
uint8_t* encode_message(CommandMessage_t command, uint8_t value) {
    uint8_t data[] = {command, value};                          // Organized data into a buffer
    uint32_t crc_value = esp_crc32_le(0, data, sizeof(data));   // The CRC checksum of the data
    
    // Compose message
    uint8_t* message_buffer = (uint8_t*)malloc(MESSAGE_SIZE);
    message_buffer[0] = START_BYTE;
    memcpy(message_buffer + sizeof(START_BYTE), data, sizeof(data));
    memcpy(message_buffer + sizeof(START_BYTE) + sizeof(data), &crc_value, sizeof(crc_value));
    message_buffer[sizeof(START_BYTE) + sizeof(data) + sizeof(crc_value)] = STOP_BYTE;

    return message_buffer;
}