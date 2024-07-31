#include <common/reflectance_sensor.h>
#include <motion/constants.h>


DualTapeSensor_t* instantiate_tape_sensor(uint8_t leftPin, uint8_t rightPin) {
    DualTapeSensor_t* tapeSensor = (DualTapeSensor_t*)malloc(sizeof(DualTapeSensor_t));
    if (NULL == tapeSensor) {
        log_error("Couldn't allocate memory for tape sensor!");
        return NULL;
    }

    pinMode(leftPin, INPUT);
    pinMode(rightPin, INPUT);

    tapeSensor->leftPin = leftPin;
    tapeSensor->rightPin = rightPin;

    tapeSensor->leftValue = 1111;
    tapeSensor->rightValue = 1111;

    log_status("Created tape sensor!");

    return tapeSensor;
}

void read_tape_sensor(DualTapeSensor_t* tapeSensor) {
    tapeSensor->leftValue = analogRead(tapeSensor->leftPin);
    tapeSensor->rightValue = analogRead(tapeSensor->rightPin);
}

int is_tape_left_or_right(DualTapeSensor_t* tapeSensor) {
    int left_mean = tapeSensor->leftValue;
    int right_mean = tapeSensor->rightValue;

    if (left_mean - right_mean > TAPE_SENSOR_AFFIRMATIVE_THRESHOLD) {
        return 1;
    } else if (right_mean - left_mean > TAPE_SENSOR_AFFIRMATIVE_THRESHOLD) {
        return -1;
    } else {
        return 0;
    }
}

int is_tape_visible(DualTapeSensor_t* tapeSensor) {
    int left_mean = tapeSensor->leftValue;
    int right_mean = tapeSensor->rightValue;

    if (left_mean > THRESHOLD_SENSOR_SINGLE && right_mean > THRESHOLD_SENSOR_SINGLE) {
        return 1; 
    } else if (left_mean < THRESHOLD_SENSOR_SINGLE && right_mean < THRESHOLD_SENSOR_SINGLE) {
        return -1;
    } else {
        return 0;
    }
}
