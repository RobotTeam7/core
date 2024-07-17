#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <master/constants.h>

#include <common/resource_manager.h>
#include <common/servo_motor.h>
#include <common/robot_motor.h>
#include <common/pwm.h>
#include <common/pin.h>

#include <communication/read.h>
#include <communication/send.h>
#include <communication/decode.h>


WiFiConfig_t config;

ServoMotor_t* servoMotor;

void setup() {
    Serial.begin(115200); // Initialize serial monitor

    servoMotor = instantiate_servo_motor(13, 0);

    // initialize_uart();
    // begin_uart_read();

    // send_uart_message(GOTO, 8);
    // delay(1000);
    // send_uart_message(GOTO, 9);
    // delay(1000);
    // send_uart_message(DO_SPIN, 0);
    // delay(1000);

}

// float dutyCycleHigh = 0.06;
// float dutyCycleLow = 0.02;
// int powerValueHigh = dutyCycleHigh * 65536;
// int powerValueLow = dutyCycleLow * 65536;
// int unit_16_number = 65536;

// float granularity = 0.001;
// float dutyCycle;
// int delay_value = 5;

void loop() {
    // motor_lit.set_position_percentage(0);
    // delay(500);
    // motor_lit.set_position_percentage(.50);
    // delay(500);
    // motor_lit.set_position_percentage(1);
    // delay(500);
    set_servo_position_percentage(servoMotor, 0.00);
    delay(1500);
    set_servo_position_percentage(servoMotor, 1.00);
    delay(1500);
}
