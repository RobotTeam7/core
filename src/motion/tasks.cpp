#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <common/reflectance_sensor.h>

#include <communication/decode.h>

#include <motion/constants.h>
#include <motion/tasks.h>
#include <motion/pid.h>
#include <motion/state.h>


// Ensure that a RobotMotorData_t* does not contain null values
int checkRobotMotors(RobotMotorData_t* robotMotors) {
    if (robotMotors == NULL) {
        return 1;
    } else {
        return robotMotors->motorFR == NULL || robotMotors->motorBR == NULL || robotMotors->motorFL == NULL || robotMotors->motorBL == NULL;
    } 
}


// Ensure that a DualTapeSensor_t* does not include null values
int checkTapeSensor(DualTapeSensor_t* tapeSensor) {
    return tapeSensor == NULL;
}

// Ensure that a TapeAwarenessData_t* does not contain null values
int checkTapeAwarenessData(TapeAwarenessData_t* tapeAwarenessData) {
    if (tapeAwarenessData == NULL) {
        return 1;
    } else {
        return checkTapeSensor(tapeAwarenessData->tapeSensor) == 1 || checkRobotMotors(tapeAwarenessData->robotMotors) == 1; 
    }
}

void TaskPollReflectance(void *pvParameters) {
    log_status("Beginning reflectance task initialization...");

    // Acquire and verify pointers
    DualTapeSensor_t* tapeSensor = (DualTapeSensor_t*)pvParameters;
    if (checkTapeSensor(tapeSensor)) {
        log_error("Error: Null pointers in reflectance sensor data!");  
        vTaskDelete(NULL);
        return;
    }

    // Setup reflectance sensor
    TickType_t delay_ticks = pdMS_TO_TICKS(POLL_SENSOR_DELAY_MS);

    log_status("Succesfully initialized reflectance polling task!");

    int lastError = 0;
    while (1) {
        // Read sensor values, pushing them onto the buffer
        read_tape_sensor(tapeSensor);

        // Print direction of less reflectance (higher value -> less reflectance, they are inversly proportional)
        if (VERBOSITY_LEVEL > MOST_VERBOSE) {
            int reflectance_right = tapeSensor->rightValue;
            int reflectance_left = tapeSensor->leftValue;

            // char drive_state[20] = "find tape";  // this could be a memory unsafe situation, as some chars are uninitialized
            // if (reflectance_right - reflectance_left > TAPE_SENSOR_AFFIRMATIVE_THRESHOLD) {
            //     strcpy(drive_state, "---->>");
            // } else if (reflectance_left - reflectance_right > TAPE_SENSOR_AFFIRMATIVE_THRESHOLD) {
            //     strcpy(drive_state, "<<----");
            // } else {
            //     strcpy(drive_state, "^^^^^^");
            // }
            // Serial.println(drive_state);
        }

        vTaskDelay(delay_ticks);
    }
}

void TaskFollowTape(void *pvParameters) {
    log_status("Beginning tape follow task initialization...");
    TickType_t delay_ticks = pdMS_TO_TICKS(MOTOR_ADJUSTMENT_DELAY_TAPE_FOLLOWING_MS);

    TapeAwarenessData_t* tapeAwarenessData = (TapeAwarenessData_t*)pvParameters;
    if (checkTapeAwarenessData(tapeAwarenessData)) {
        log_error("Error: tapeAwarenessData contains nulls!");
        vTaskDelete(NULL);
        return;
    }

    RobotMotorData_t* robotMotors = tapeAwarenessData->robotMotors;

    log_status("Successfully initialized tape follow task!");
    int lastError = 0;
    while (1) {
        int left_mean = tapeAwarenessData->tapeSensor->leftValue;
        int right_mean = tapeAwarenessData->tapeSensor->rightValue;

        int error = left_mean - right_mean;
        int pid_adjustment_value = pid_follow_tape(error, lastError);
        lastError = error;
        int motor_speed_1 = MOTOR_SPEED_FOLLOWING - pid_adjustment_value;
        int motor_speed_2 = MOTOR_SPEED_FOLLOWING + pid_adjustment_value;
        motor_set_drive(robotMotors->motorFL, motor_speed_1, FORWARD_DRIVE); 
        motor_set_drive(robotMotors->motorBL, motor_speed_1, FORWARD_DRIVE);
        motor_set_drive(robotMotors->motorFR, motor_speed_2, FORWARD_DRIVE);
        motor_set_drive(robotMotors->motorBR, motor_speed_2, FORWARD_DRIVE);

        vTaskDelay(delay_ticks);
    }
    Serial.println("Exiting tape follow");
}

int checkRobotControlData(RobotControlData_t* robotControlData) {
    if (robotControlData == NULL) {
        return 1;
    } else {
        return checkTapeAwarenessData(robotControlData->tapeAwarenessData) == 1 || robotControlData->xSharedQueue == NULL; 
    }
}

void TaskRotate(void *pvParameters) {
    log_status("Beginning rotate task initialization...");

    RobotControlData_t* robotControlData = (RobotControlData_t*)pvParameters;
    if (checkRobotControlData(robotControlData))
    {
        log_error("Error: nulls in robotControlData");
        vTaskDelete(NULL);
        return;
    }

    // convert ms delays into ticks
    TickType_t inital_delay_ticks = pdMS_TO_TICKS(ROTATE_INITIAL_DELAY);
    TickType_t poll_rate_ticks = pdMS_TO_TICKS(MOTOR_ADJUSTMENT_DELAY_ROTATING_MS);

    RobotMotorData_t* robotMotors = robotControlData->tapeAwarenessData->robotMotors;
    DualTapeSensor_t* tapeSensor = robotControlData->tapeAwarenessData->tapeSensor;
    bool rotating;

    log_status("Successfully initialized rotation!");

    while (1) {
        // start rotating, then delay to ensure that rotation isn't immediately canceled by tape detection
        motor_set_drive(robotMotors->motorFR, MOTOR_SPEED_ROTATION, REVERSE_DRIVE);
        motor_set_drive(robotMotors->motorBR, MOTOR_SPEED_ROTATION, REVERSE_DRIVE);
        motor_set_drive(robotMotors->motorFL, MOTOR_SPEED_ROTATION, FORWARD_DRIVE);
        motor_set_drive(robotMotors->motorBL, MOTOR_SPEED_ROTATION, FORWARD_DRIVE);
        
        vTaskDelay(inital_delay_ticks);

        // look for tape detection
        rotating = true;
        
        log_message("Looking for tape...");

        while (rotating)
        {
            int left_mean = tapeSensor->leftValue;
            int right_mean = tapeSensor->rightValue;

            if (right_mean > THRESHOLD_SENSOR_SINGLE && left_mean > THRESHOLD_SENSOR_SINGLE)
            {
                log_status("Found tape. Ending rotation...");
                rotating = false;

                motor_stop(robotMotors->motorFR);
                motor_stop(robotMotors->motorBR);
                motor_stop(robotMotors->motorFL);
                motor_stop(robotMotors->motorBL);

                // send message to TaskMaster that rotation has finished
                StatusMessage_t message = ROTATION_DONE;
                if (xQueueSend(*robotControlData->xSharedQueue, &message, portMAX_DELAY) != pdPASS)
                {
                    log_error("Failed to send ROTATION_DONE to xSharedQueue");
                }
            }

            vTaskDelay(poll_rate_ticks);
        }
    }
}

int checkState(State_t* state) {
    return state == NULL;
}

int checkMonoTapeSensor(MonoTapeSensor_t* tapeSensor) {
    return tapeSensor == NULL;
}


void TaskStationTracking(void* pvParameters) {
    MonoTapeSensor_t* tapeSensor = (MonoTapeSensor_t*)pvParameters;
    if (checkMonoTapeSensor(tapeSensor)) {
        log_error("Nulls in mono tape sensor!");
        return;
    }

    TickType_t delay = pdMS_TO_TICKS(STATION_TRACKING_POLL_DELAY_MS);
    uint16_t consecutive_count = 0;
    int value = 0;

    log_status("Initialized TaskStationTracking");
    bool found_tape = false;
    while (1) {
        // Check sensors
        read_tape_sensor(tapeSensor);
        value = tapeSensor->value;
       
        // if (value > THRESHOLD_SENSOR_SINGLE) {
        //     consecutive_count++;
        // } else {
        //     consecutive_count = 0;
        // }

        // if (consecutive_count > DEBOUNCE_TAPE_THRESHOLD) {
        //     consecutive_count = 0;
        //     state.last_station += state.orientation;

        //     log_status("Passed station!");
        // }
        if (value > THRESHOLD_SENSOR_SINGLE) {
            found_tape = true;
        } else if ((value < THRESHOLD_SENSOR_SINGLE) && (found_tape == true)) {
            state.last_station += state.orientation;
            log_status("Passed station!");
            found_tape = false;
        }
        vTaskDelay(delay);
    }
}
