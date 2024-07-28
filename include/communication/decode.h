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
typedef enum { NONE = 0x00, GOTO = 0x01, DO_SPIN = 0x02, COMPLETED = 0x03, ABORT = 0x04, OCCUPIED = 0x05, ACCEPTED = 0x06, READY = 0x07, COUNTER_DOCK = 0x08, TAPE_RETURN = 0x09, FOLLOW_WALL_TO = 0x10, DO_PIROUETTE = 0x11 } CommandMessage_t;   

/**
 * @brief Decode a byte into a CommandMessage_t
 */
CommandMessage_t decode_command(uint8_t message_identifier);


#endif // DECODE_COMMUNICATION_H