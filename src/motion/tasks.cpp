#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <common/reflectance_sensor.h>
#include <motion/constants.h>
#include <motion/tasks.h>

#define POLL_SENSOR_DELAY_MS 5
#define MOTOR_TASK_DELAY_MS 5


// Ensure that a RobotMotorData_t* does not contain null values
int checkRobotMotors(RobotMotorData_t* robotMotors) {
    if (robotMotors == NULL) {
        return 1;
    } else {
        return robotMotors->motorFR == NULL || robotMotors->motorBR == NULL || robotMotors->motorFL == NULL || robotMotors->motorBL == NULL;
    } 
}


// Ensure that a TapeSensor_t* does not include null values
int checkTapeSensor(TapeSensor_t* tapeSensor) {
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
    TapeSensor_t* tapeSensor = (TapeSensor_t*)pvParameters;
    if (checkTapeSensor(tapeSensor)) {
        log_error("Error: Null pointers in reflectance sensor data!");  
        vTaskDelete(NULL);
        return;
    }

    // Setup reflectance sensor
    TickType_t delay_ticks = pdMS_TO_TICKS(POLL_SENSOR_DELAY_MS);

    log_status("Succesfully initialized reflectance polling task!");

    while (1) {
        // Read sensor values, pushing them onto the buffer
        read_tape_sensor(tapeSensor);

        // Print direction of greater reflectance
        if (PRINT_DRIVE_STATE) {
            int reflectance_right = tapeSensor->rightValue;
            int reflectance_left = tapeSensor->leftValue;

            if (reflectance_right - reflectance_left > THRESHOLD) {
                Serial.println("---->>");
            } else if (reflectance_left - reflectance_right > THRESHOLD) {
                Serial.println("<<----");
            } else {
                Serial.println("^^^^^^");
            }
        }

        vTaskDelay(delay_ticks);
    }
}

void TaskFollowTape(void *pvParameters) {
    log_status("Beginning tape follow task initialization...");

    TapeAwarenessData_t* tapeAwarenessData = (TapeAwarenessData_t*)pvParameters;
    if (checkTapeAwarenessData(tapeAwarenessData)) {
        log_error("Error: tapeAwarenessData contains nulls!");
        vTaskDelete(NULL);
        return;
    }

    RobotMotorData_t* robotMotors = tapeAwarenessData->robotMotors;

    TickType_t delay_ticks = pdMS_TO_TICKS(MOTOR_TASK_DELAY_MS);

    log_status("Successfully initialized tape follow task!");

    while (1) {
        // Read buffers
        
        // If we lost the tape, let master know 
        if (is_tape_visible(tapeAwarenessData->tapeSensor) < 0) {
            // Send message to TaskMaster that we lost the tape
            Message message = LOST_TAPE;
            if (xQueueSend(*tapeAwarenessData->xSharedQueue, &message, portMAX_DELAY) != pdPASS)
            {
                log_error("Failed to send LOST_TAPE to xSharedQueue");
            }
        }

        int left_mean = tapeAwarenessData->tapeSensor->leftValue;
        int right_mean = tapeAwarenessData->tapeSensor->rightValue;
        int tape_visibility = is_tape_left_or_right(tapeAwarenessData->tapeSensor);
        
        if (tape_visibility == 1) {                       // If we are detecting tape on the left, turn left
            // log_message("Driving left...");
            motor_set_drive(robotMotors->motorFL, MOTOR_SPEED_LOW, FORWARD_DRIVE); 
            motor_set_drive(robotMotors->motorBL, MOTOR_SPEED_LOW, FORWARD_DRIVE);
            motor_set_drive(robotMotors->motorFR, MOTOR_SPEED_HIGH, FORWARD_DRIVE);
            motor_set_drive(robotMotors->motorBR, MOTOR_SPEED_HIGH, FORWARD_DRIVE);
        } else if (tape_visibility == -1) {               // If we are detecting tape on the right, turn right
            // log_message("Driving right...");
            motor_set_drive(robotMotors->motorFL, MOTOR_SPEED_HIGH, FORWARD_DRIVE);
            motor_set_drive(robotMotors->motorBL, MOTOR_SPEED_HIGH, FORWARD_DRIVE);
            motor_set_drive(robotMotors->motorFR, MOTOR_SPEED_LOW, FORWARD_DRIVE);
            motor_set_drive(robotMotors->motorBR, MOTOR_SPEED_LOW, FORWARD_DRIVE);
        } else {                                                        // Otherwise, drive forwards
            // log_message("Driving forward...");
            motor_set_drive(robotMotors->motorFL, MOTOR_SPEED_HIGH, FORWARD_DRIVE); 
            motor_set_drive(robotMotors->motorBL, MOTOR_SPEED_HIGH, FORWARD_DRIVE);
            motor_set_drive(robotMotors->motorFR, MOTOR_SPEED_HIGH, FORWARD_DRIVE);
            motor_set_drive(robotMotors->motorBR, MOTOR_SPEED_HIGH, FORWARD_DRIVE);
        }

        vTaskDelay(delay_ticks);
    }
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
    TickType_t inital_delay_ticks = pdMS_TO_TICKS(1000);
    TickType_t poll_rate_ticks = pdMS_TO_TICKS(MOTOR_ADJUSTMENT_DELAY_ROTATING_MS);

    RobotMotorData_t* robotMotors = robotControlData->tapeAwarenessData->robotMotors;
    TapeSensor_t* tapeSensor = robotControlData->tapeAwarenessData->tapeSensor;
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
            // Serial.println("Difference:" + String(left_mean - right_mean));

            if (right_mean > THRESHOLD_SENSOR_SINGLE && left_mean > THRESHOLD_SENSOR_SINGLE)
            {
                log_status("Found tape. Ending rotation...");
                rotating = false;

                motor_stop(robotMotors->motorFR);
                motor_stop(robotMotors->motorBR);
                motor_stop(robotMotors->motorFL);
                motor_stop(robotMotors->motorBL);

                // send message to TaskMaster that rotation has finished
                Message message = ROTATION_DONE;
                if (xQueueSend(*robotControlData->xSharedQueue, &message, portMAX_DELAY) != pdPASS)
                {
                    log_error("Failed to send ROTATION_DONE to xSharedQueue");
                }
            }

            vTaskDelay(poll_rate_ticks);
        }
    }
}