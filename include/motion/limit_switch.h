#ifndef LIMIT_SWITCH_H
#define LIMIT_SWITCH_H

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h> 

#include <common/utils.h>


#define MAX_LIMIT_SWITCHES 10

/**
 * @brief This struct contains the data representing a limit switch. 
 */
typedef struct
{
    uint8_t interrupt_pin;          // The pin number that will be registered as an interrupt
    TaskHandle_t* task_to_notify;    // The task that will be notified of a rising edge on `interrupt_pin`.
} LimitSwitch_t;

extern volatile uint8_t limit_switch_count;                 // Counter to keep track of the number of created limit switches
extern volatile LimitSwitch_t* limit_switches[MAX_LIMIT_SWITCHES];   // Array to keep track of all registered limit switches

/**
 * @brief Instantiate a limit switch, registering an interrupt on `interrupt_pin` to notify `task_to_notify`.
 * 
 * @returns Heap-allocated (created with `malloc()`) pointer. Remember to free!
 */
LimitSwitch_t* instantiate_limit_switch(uint8_t interrupt_pin);


#endif // LIMIT_SWITCH_H