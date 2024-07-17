#ifndef DECODE_COMMUNICATION_H
#define DECODE_COMMUNICATION_H

#include <Arduino.h>


/**
 * @brief Enum discretizing the different kinds of messages that can be sent between the master task and subtasks
 */
typedef enum { ROTATION_DONE, LOST_TAPE } StatusMessage_t;

/**
 * @brief Enum discretizing the different kinds of messages being passed between boards
 */
typedef enum { GOTO, DO_SPIN, COMPLETED, NONE } CommandMessage_t;   


/**
 * @brief Decode a byte into a CommandMessage_t
 */
CommandMessage_t decode_command(uint8_t message_identifier);


#endif // DECODE_COMMUNICATION_H