#include <common/reflectance_sensor.h>
#include <motion/constants.h>


TapeSensor_t* instantiate_tape_sensor(uint8_t pin) {
    TapeSensor_t* tapeSensor = (TapeSensor_t*)malloc(sizeof(TapeSensor_t));
    if (NULL == tapeSensor) {
        log_error("Couldn't allocate memory for tape sensor!");
        return NULL;
    }

    pinMode(pin, INPUT);

    tapeSensor->pin = pin;

    tapeSensor->value = 1111;

    log_status("Created tape sensor!");

    return tapeSensor;
}

void read_tape_sensor(TapeSensor_t* tapeSensor) {
    tapeSensor->value = analogRead(tapeSensor->pin);
}