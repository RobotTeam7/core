#ifndef ROBOT_UART_H
#define ROBOT_UART_H

#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <common/robot_motor.h>
#include <common/pin.h>
#include <common/stepper_motor.h>
#include <common/reflectance_sensor.h>
#include <motion/FreeRTOSConfig.h>
#include <motion/constants.h>
#include <motion/tasks.h>
#include <motion/utils.h>


/**
 * @brief Enum discretizing the different kinds of messages that can be sent between the master task and subtasks
 */
typedef enum { ROTATION_DONE, LOST_TAPE } StatusMessage_t;

/**
 * @brief Enum discretizing the different kinds of messages being passed between boards
 */
typedef enum { GOTO, DO_SPIN, COMPLETED, NONE } CommandMessage_t;   

typedef struct {
    CommandMessage_t command;
    uint8_t value;
} Packet_t;

void initialize_uart();


#endif