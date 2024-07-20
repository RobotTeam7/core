#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <common/reflectance_sensor.h>

#include <communication/decode.h>

#include <motion/constants.h>
#include <motion/tasks.h>
#include <motion/pid.h>
#include <motion/state.h>
#include <motion/motion.h>


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
    return tapeAwarenessData == NULL || checkTapeSensor(tapeAwarenessData->tapeSensor) == 1;
}

int checkState(State_t* state) {
    return state == NULL;
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

    state.drive_state = DriveState_t::DRIVE;
    state.drive_speed = MOTOR_SPEED_FOLLOWING;

    log_status("Successfully initialized tape follow task!");
    int lastError = 0;
    while (1) {

        read_tape_sensor(tapeAwarenessData->tapeSensor);
        int left_mean = tapeAwarenessData->tapeSensor->leftValue;
        int right_mean = tapeAwarenessData->tapeSensor->rightValue;

        // Serial.println("Left" + String(left_mean));
        // Serial.println("Right" + String(right_mean));

        int error = left_mean - right_mean;

        int pid_adjustment_value = pid_follow_tape(error, lastError);
        state.yaw = pid_adjustment_value;
        lastError = error;

        vTaskDelay(delay_ticks);
    }
}

void TaskRotate(void *pvParameters) {
    log_status("Beginning rotate task initialization...");

    TapeAwarenessData_t* robotControlData = (TapeAwarenessData_t*)pvParameters;
    if (checkTapeAwarenessData(robotControlData))
    {
        log_error("Error: nulls in robotControlData");
        vTaskDelete(NULL);
        return;
    }

    // convert ms delays into ticks
    TickType_t inital_delay_ticks = pdMS_TO_TICKS(ROTATE_INITIAL_DELAY);
    TickType_t poll_rate_ticks = pdMS_TO_TICKS(MOTOR_ADJUSTMENT_DELAY_ROTATING_MS);

    DualTapeSensor_t* tapeSensor = robotControlData->tapeSensor;
    bool rotating;

    log_status("Successfully initialized rotation!");

    state.drive_state = DriveState_t::ROTATE;
    state.drive_speed = MOTOR_SPEED_ROTATION;

    while (1) {
        // start rotating, then delay to ensure that rotation isn't immediately canceled by tape detection
        vTaskDelay(inital_delay_ticks);

        // look for tape detection
        rotating = true;
        
        log_message("Looking for tape...");

        while (rotating)
        {
            read_tape_sensor(tapeSensor);
            int left_mean = tapeSensor->leftValue;
            int right_mean = tapeSensor->rightValue;

            if (right_mean > THRESHOLD_SENSOR_SINGLE && left_mean > THRESHOLD_SENSOR_SINGLE)
            {
                log_status("Found tape. Ending rotation...");
                rotating = false;

                state.drive_state = DriveState_t::STOP;
                state.drive_speed = 0;

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

int checkDualTapeSensor(DualTapeSensor_t* tapeSensor) {
    return tapeSensor == NULL;
}

void TaskStationTracking(void* pvParameters) {
    DualTapeSensor_t* tapeSensor = (DualTapeSensor_t*)pvParameters;
    if (checkDualTapeSensor(tapeSensor)) {
        log_error("Nulls in dual tape sensor!");
        return;
    }

    TickType_t delay = pdMS_TO_TICKS(STATION_TRACKING_POLL_DELAY_MS);
    uint16_t consecutive_count = 0;
    int value_left = 0;
    int value_right;

    log_status("Initialized TaskStationTracking");
    bool found_tape = false;
    while (1) {
        // Check sensors
        read_tape_sensor(tapeSensor);
        value_left = tapeSensor->leftValue;
        value_right = tapeSensor->rightValue;
       
        if (value_left > THRESHOLD_SENSOR_SINGLE || value_right > THRESHOLD_SENSOR_SINGLE) {
            found_tape = true;
        } else if ((value_left < THRESHOLD_SENSOR_SINGLE || value_right < THRESHOLD_SENSOR_SINGLE) && (found_tape == true)) {
            state.last_station += state.orientation;
            log_status("Passed station!");
            found_tape = false;
            vTaskDelay(pdMS_TO_TICKS(300));
        }
        vTaskDelay(delay);
    }
}


void TaskDocking(void* pvParameters) {
    TapeAwarenessData_t* tapeAwarenessData = (TapeAwarenessData_t*)pvParameters;
    if (checkTapeAwarenessData(tapeAwarenessData)) {
        log_error("Error: tapeAwarenessData contains nulls!");
        vTaskDelete(NULL);
        return;
    }
}

void TaskDrive(void* pvParameters) {
    log_status("Initializing drive task...");

    RobotMotorData_t* robot_motors = (RobotMotorData_t*)pvParameters;
    if (checkRobotMotors(robot_motors)) {
        log_error("Nulls in robot motors!");
    }
    log_status("Initialized drive task!");

    while (1) {
        switch (state.drive_state) {
            case STOP:
                stop_all_motors(robot_motors);
                break;

            case DRIVE:
                // set_robot_drive(robot_motors, state.drive_speed * state.direction);
                set_robot_drive(robot_motors, state.drive_speed * state.direction);
                break;

            case DriveState_t::ROTATE:
                rotate_robot(robot_motors, state.drive_speed * state.helicity);
                break;
        }
        vTaskDelay(MOTOR_UPDATE_DELAY);
    }
}
