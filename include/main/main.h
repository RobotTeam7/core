#ifndef MAIN_ROBOT_H
#define MAIN_ROBOT_H

#include <communication/decode.h>
#include <Arduino.h>
#include <common/utils.h>
#include <main/constants.h>
#include <common/servo_motor.h>
#include <common/servo_motor.h>
#include <common/robot_motor.h>
#include <common/pwm.h>
#include <common/limit_switch.h>

#include <communication/wifi_client.h>
#include <communication/uart.h>
#include <communication/decode.h>

extern QueueHandle_t outboundWiFiQueue;
extern QueueHandle_t inboundWiFiQueue;
extern QueueHandle_t uart_msg_queue;
extern WiFiHandler_t wifi_handler;
extern ServoMotor_t* claw_servo;
extern ServoMotor_t* vertical_servo;
extern bool MOTION_BUSY;
extern bool MOTION_READY;

void uart_msg_handler(void *parameter);
void wifi_msg_handler(void *parameter);
void TaskMaster(void* pvParameters);

void grab_with_claw(int claw_percentage);

void grab_plate();

void open_claw(ServoPositionsPercentage_t percentage);

void wait_for_motion();

void send_command(CommandMessage_t command, int8_t value);


#endif // MAIN_ROBOT_H