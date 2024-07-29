#ifndef DECODE_COMMUNICATION_H
#define DECODE_COMMUNICATION_H

#include <Arduino.h>


/**
 * @brief Enum discretizing the different kinds of messages that can be sent between the master task and subtasks
 */
typedef enum { ROTATION_DONE, LOST_TAPE, REACHED_POSITION } StatusMessage_t;

/**
 * @brief Enum discretizing the different kinds of messages being passed between boards
 */
typedef enum { READY = 0x00, GOTO = 0x01, DO_SPIN = 0x02, ABORT = 0x03, TAPE_RETURN = 0x04, COUNTER_DOCK = 0x05, COMPLETED = 0x06, OCCUPIED = 0x07, ACCEPTED = 0x08, ACK = 0x09, NACK = 0x0a } CommandMessage_t;   

// /**
//  * @brief Decode a byte into a CommandMessage_t
//  */
// CommandMessage_t decode_command(uint8_t message_identifier);


#endif // DECODE_COMMUNICATION_H