#ifndef COMMUNICATION_PROTOCOL_H
#define COMMUNICATION_PROTOCOL_H

#include <Arduino.h>
#include "esp_crc.h"

#include <common/utils.h>


#define START_BYTE      (uint8_t)0x7E
#define STOP_BYTE       (uint8_t)0x7F
#define MESSAGE_SIZE    (sizeof(START_BYTE) + (2 * sizeof(uint8_t)) + sizeof(uint32_t) + sizeof(STOP_BYTE))

/**
 * @attention COMMUNICATION PROTOCOL
 * 
 * Our communication protocol is designed to provide utmost protection against noise and corruption. As such,
 * a start byte and an end byte will enclose the data packet, which will consist of a command byte, data byte, 
 * a CRC checksum calculated from the COMMAND and VALUE bytes. 
 * 
 * If a decoded message indicates corruption (wrong start byte, wrong CRC checksum, wrong stop byte), the receiver
 * should respond with a NACK message after some delay, otherwise some sort of ACK should be sent. Upon receving a NACK,
 * the sender should retransmit the most recently transmitted message after some delay. If multiple NACKs are received 
 * in a row, a software fault should be raised.
 * 
 * LAYOUT:
 *  [0]:     START BYTE     (0x7E)
 *  [1]:     COMMAND BYTE   MSG: (0x00-0x3f) | ACK: (0x40-0x4f) | NACK: (0x50-0x5f)
 *  [2]:     VALUE BYTE     (0x00-0xff)
 *  [3-7]:   CRC CHECKSUM   (0x0000-0xffff)
 *  [8]:     STOP BYTE      (0x7F)
 * 
 * ACK TYPES                (0x40-0x4f):
 *  OCCUPIED  = 0x05,
 *  ACCEPTED  = 0x06
 *  ACK       = 0x0b
 * 
 * NACK TYPES               (0x50-0x5f):
 *  NACK      = 0x0a
 * 
 * All bytes are expected to be uin8_t.
 */

/**
 * @brief Enum discretizing the different kinds of messages that can be sent between the master task and subtasks
 */
typedef enum { ROTATION_DONE, LOST_TAPE, REACHED_POSITION } StatusMessage_t;

/**
 * @brief Enum discretizing the different kinds of messages being passed between boards.
 */
typedef enum { READY = 0x00, GOTO = 0x01, DO_SPIN = 0x02, ABORT = 0x03, TAPE_RETURN = 0x04, COUNTER_DOCK = 0x05, COMPLETED = 0x06, FOLLOW_WALL_TO = 0x07, DO_PIROUETTE = 0x08, MOVE_ASIDE = 0x09, SET_MULTIPLIER = 0x0a, STARTUP_SERVER = 0x0b, NEXT_ACTION = 0x0c, WAIT_FOR_ACTION = 0x0d, OCCUPIED = 0x40, ACCEPTED = 0x41, ACK = 0x42, NACK = 0x50 } CommandMessage_t;

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

/**
 * @brief Encode a `command` and `value` into a byte array representing a valid `Message_t`.
 * 
 * @returns Heap-allocated (created with `malloc()`) pointer. Remember to free!
 */
uint8_t* encode_message(CommandMessage_t command, uint8_t value);

/**
 * @brief Decode a byte array into a valid `Message_t`. 
 * 
 * @param data_pointer pointer to the byte array.
 * @param destination (non-null) pointer to the `Message_t` struct which will be populated.
 */
void decode_message(const uint8_t* data_pointer, Message_t* destination);


#endif // COMMUNICATION_PROTOCOL_H