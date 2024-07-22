#include <common/limit_switch.h>


volatile uint8_t limit_switch_count = 0;
LimitSwitch_t* limit_switches[MAX_LIMIT_SWITCHES];

void IRAM_ATTR GenericISR() {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // Iterate over all registered limit switches, determine which one is currently triggered, and notify that task
    for (uint8_t i = 0; i < limit_switch_count; i++) {
        if (digitalRead(limit_switches[i]->interrupt_pin) == HIGH) {
            TaskHandle_t task_to_notify = limit_switches[i]->task_to_notify;

            // Notify the corresponding task, if its a valid 
            if (task_to_notify != NULL) {
                vTaskNotifyGiveFromISR(*limit_switches[i]->task_to_notify, &xHigherPriorityTaskWoken);
            } else {
                log_error("Interrupt tried to notify a deleted task!");
            }
        }
    }

    // Request a context switch if giving the notification unblocked a higher priority task
    if (xHigherPriorityTaskWoken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

LimitSwitch_t* instantiate_limit_switch(uint8_t interrupt_pin, TaskHandle_t* task_to_notify) {
    if (limit_switch_count >= MAX_LIMIT_SWITCHES) {
        log_error("Limit switch creation aborted: reached maximum number of limit switches!");
        return NULL;
    }

    LimitSwitch_t* new_switch = (LimitSwitch_t*)malloc(sizeof(LimitSwitch_t));
    if (new_switch == NULL) {
        log_error("Couldn't instantiate limit switch!");
    }

    new_switch->interrupt_pin = interrupt_pin;
    new_switch->task_to_notify = task_to_notify;

    limit_switches[limit_switch_count++] = new_switch;

    // Attach the generic ISR
    attachInterrupt(digitalPinToInterrupt(interrupt_pin), GenericISR, RISING);

    log_status("Created new limit switch!");
}
