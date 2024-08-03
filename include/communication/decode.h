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

typedef enum { READY = 0x00, GOTO = 0x01, DO_SPIN = 0x02, ABORT = 0x03, TAPE_RETURN = 0x04, COUNTER_DOCK = 0x05, COMPLETED = 0x06, FOLLOW_WALL_TO = 0x07, DO_PIROUETTE = 0x08, SWITCH_COUNTER = 0x09, SET_MULTIPLIER = 0x0a, STARTUP_SERVER = 0x0b, NEXT_ACTION = 0x0c, WAIT_FOR_ACTION = 0x0d, OCCUPIED = 0x40, ACCEPTED = 0x41, ACK = 0x42, NACK = 0x50 } CommandMessage_t;


#endif // DECODE_COMMUNICATION_H