#include <common/reflectance_sensor.h>
#include <motion/constants.h>


TapeSensor_t* instantiate_tape_sensor(uint8_t leftPin, uint8_t rightPin) {
    TapeSensor_t* tapeSensor = (TapeSensor_t*)malloc(sizeof(TapeSensor_t));
    if (NULL == tapeSensor) {
        log_error("Couldn't allocate memory for tape sensor!");
        return NULL;
    }
     
    tapeSensor->leftPin = leftPin;
    tapeSensor->rightPin = rightPin;

    tapeSensor->leftValue = 1111;
    tapeSensor->rightValue = 1111;

    log_status("Created tape sensor!");

    return tapeSensor;
}

// perform a read of the reflectance sensors 
void read_tape_sensor(TapeSensor_t* tapeSensor) {
    tapeSensor->leftValue = analogRead(tapeSensor->leftPin);
    tapeSensor->rightValue = analogRead(tapeSensor->rightPin);
}

// return 1 if the left - right > THRESHOLD,  -1 if right - left > THRESHOLD, 0 if neither is true
int is_tape_left_or_right(TapeSensor_t* tapeSensor) {
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

// return 1 if both sensors see tape, -1 if neither sensor sees tape, 0 if only one of them sees tape
int is_tape_visible(TapeSensor_t* tapeSensor) {
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
