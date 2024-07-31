#ifndef LIMIT_SWITCH_H
#define LIMIT_SWITCH_H

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h> 

#include <common/utils.h>

#include <motion/tasks.h>


/**
 * @brief This struct contains the data representing a limit switch. 
 */
typedef struct
{
    uint8_t interrupt_pin;  // The pin number that will be registered as an interrupt
} LimitSwitch_t;

typedef void (*ISR_Function)();

/**
 * @brief Instantiate a limit switch, registering an interrupt on `interrupt_pin` to notify `task_to_notify`.
 * 
 * @returns Heap-allocated (created with `malloc()`) pointer. Remember to free!
 */
LimitSwitch_t* instantiate_limit_switch(uint8_t interrupt_pin, ISR_Function isr_function);


#endif // LIMIT_SWITCH_H