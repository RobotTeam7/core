#ifndef ROBOT_UART_H
#define ROBOT_UART_H

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/uart.h>

#include <communication/communication.h>


/**
 * @brief Initialize the UART communication line.
 * 
 * @param packet_queue the xSharedQueue of type `Packet_t` which will be the destination messages received over UART.
 */
void initialize_uart(QueueHandle_t* packet_queue, uint8_t tx_pin, uint8_t rx_pin);

/**
 * @brief Send a message consisting of a `command` and `value` over UART. 
 * 
 * @param memorize whether this message should be memorized, to be resent if transmission isn't received (ensure always `false` for ACK or NACK messages).
 */
void send_uart_message(CommandMessage_t command, uint8_t value = 0, bool memorize = true);

/**
 * @brief Transmit a NACK message, requesting re-transmission of a corrupted packet.
 */
static void send_nack();

#endif // ROBOT_UART_H