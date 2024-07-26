#ifndef ROBOT_UART_H
#define ROBOT_UART_H

#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <common/robot_motor.h>
#include <common/stepper_motor.h>
#include <common/reflectance_sensor.h>

#include <motion/constants.h>
#include <motion/tasks.h>
#include <motion/utils.h>

#include <communication/decode.h>


typedef struct {
    CommandMessage_t command;
    int8_t value;
} Packet_t;

void initialize_uart();
void begin_uart_read(QueueHandle_t* uart_msg_queue);
void send_uart_message(CommandMessage_t message, uint8_t value = 0);


#endif // ROBOT_UART_H