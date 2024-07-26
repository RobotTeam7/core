#include <motion/limit_switch.h>
#include <motion/tasks.h>


void IRAM_ATTR docking_isr() {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (xDockingHandle != NULL) {
        vTaskNotifyGiveFromISR(xDockingHandle, &xHigherPriorityTaskWoken);
    }

    // Request a context switch if giving the notification unblocked a higher priority task
    if (xHigherPriorityTaskWoken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
} 

// void IRAM_ATTR GenericISR() {
//     BaseType_t xHigherPriorityTaskWoken = pdFALSE;
//     Serial.println("ISR!");
//     // Iterate over all registered limit switches, determine which one is currently triggered, and notify that task
//     for (uint8_t i = 0; i < limit_switch_count; i++) {
//         if (digitalRead(limit_switches[i]->interrupt_pin) == HIGH) {
//             TaskHandle_t task_to_notify = *limit_switches[i]->task_to_notify;

//             // Notify the corresponding task, if its a valid 
//             if (task_to_notify != NULL) {
//                 vTaskNotifyGiveFromISR(task_to_notify, &xHigherPriorityTaskWoken);
//             } else {
//                 log_error("Interrupt tried to notify a deleted task!");
//             }
//         }
//     }

//     // Request a context switch if giving the notification unblocked a higher priority task
//     if (xHigherPriorityTaskWoken == pdTRUE) {
//         portYIELD_FROM_ISR();
//     }
// }

LimitSwitch_t* instantiate_limit_switch(uint8_t interrupt_pin) {
    LimitSwitch_t* new_switch = (LimitSwitch_t*)malloc(sizeof(LimitSwitch_t));
    if (new_switch == NULL) {
        log_error("Couldn't instantiate limit switch!");
        return NULL;
    }

    new_switch->interrupt_pin = interrupt_pin;

    // Attach the generic ISR
    pinMode(interrupt_pin, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(interrupt_pin), docking_isr, RISING);

    log_status("Created new limit switch!");

    return new_switch;
}
