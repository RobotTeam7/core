#include <common/reflectance_sensor.h>
#include <motion/constants.h>


DualTapeSensor_t* instantiate_tape_sensor(uint8_t pin) {
    DualTapeSensor_t* tapeSensor = (DualTapeSensor_t*)malloc(sizeof(DualTapeSensor_t));
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

void read_tape_sensor(DualTapeSensor_t* tapeSensor) {
    tapeSensor->value = analogRead(tapeSensor->pin);
}