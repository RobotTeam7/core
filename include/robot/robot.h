#ifndef MAIN_ROBOT_H
#define MAIN_ROBOT_H

#include <common/utils.h>
#include <common/hal.h>
#include <common/pwm.h>

#include <communication/wifi.h>
#include <communication/uart.h>


#define SERVO_ACTUATION_DELAY 350

#ifndef use_wifi
    #define use_wifi 1
#endif

#if use_wifi == 1
    #pragma message "Enabling the use of WiFi!"
#elif use_wifi == 0
    #pragma message "Disabling the use of WiFi!"
#endif

typedef enum {
    VERTICAL_UP = 0,
    VERTICAL_HEIGHT_3 = 10, // 3 = second highest position
    VERTICAL_HEIGHT_2 = 35, // 2 = halfway
    VERTICAL_HEIGHT_1 = 60, // 1 = second lowest position
    VERTICAL_DOWN = 100,

    CLAW_CLOSED_FULL = 0,
    CLAW_OPEN = 100,
    CLAW_CLOSED_BUN = 21,
    CLAW_CLOSED_LETTUCE = 0,
    CLAW_CLOSED_TOMATO = 3,
    CLAW_CLOSED_CHEESE = 13,
    CLAW_CLOSED_PATTY = 12,

    PLATE_CLOSED = 0,
    PLATE_OPEN = 100,

    DRAW_BRIDGE_UP = 100,
    DRAW_BRIDGE_DOWN = 0,
} ServoPositionsPercentage_t;

extern QueueHandle_t uart_msg_queue;
extern ServoMotor_t* claw_servo;
extern ServoMotor_t* vertical_servo;
extern bool MOTION_BUSY;
extern bool MOTION_READY;

extern bool wifi_ready;
extern bool action_ready;
extern bool block_action;

void uart_msg_handler(void *parameter);
void wifi_msg_handler(void *parameter);
void TaskMaster(void* pvParameters);

void send_wifi_message(CommandMessage_t command, int8_t value);

void grab_with_claw(ServoPositionsPercentage_t claw_percentage);

void grab_plate();

void open_claw(ServoPositionsPercentage_t percentage);

void wait_for_motion();

void send_command(CommandMessage_t command, int8_t value);

void init_communications(uint8_t tx_pin, uint8_t rx_pin);


#endif // MAIN_ROBOT_H