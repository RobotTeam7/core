#include <common/limit_switch.h>


LimitSwitch_t* instantiate_limit_switch(uint8_t interrupt_pin, ISR_Function isr_function) {
    LimitSwitch_t* new_switch = (LimitSwitch_t*)malloc(sizeof(LimitSwitch_t));
    if (new_switch == NULL) {
        log_error("Couldn't instantiate limit switch!");
        return NULL;
    }

    new_switch->interrupt_pin = interrupt_pin;

    // Attach the generic ISR
    pinMode(interrupt_pin, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(interrupt_pin), isr_function, RISING);

    log_status("Created new limit switch!");

    return new_switch;
}
