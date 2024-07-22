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

// Ensure that a NavigationData_t* does not contain null values
int checkNavigationData(NavigationData_t* navigationData) {
    return navigationData == NULL || checkTapeSensor(navigationData->backTapeSensor) || checkTapeSensor(navigationData->fontTapeSensor) == 1;
}

int checkState(State_t* state) {
    return state == NULL;
}

void TaskFollowTape(void *pvParameters) {
    log_status("Beginning tape follow task initialization...");
    TickType_t delay_ticks = pdMS_TO_TICKS(MOTOR_ADJUSTMENT_DELAY_TAPE_FOLLOWING_MS);

    NavigationData_t* navigationData = (NavigationData_t*)pvParameters;
    if (checkNavigationData(navigationData)) {
        log_error("Error: tapeAwarenessData contains nulls!");
        vTaskDelete(NULL);
        return;
    }

    state.drive_state = DriveState_t::DRIVE;
    state.drive_speed = MOTOR_SPEED_FOLLOWING;

    log_status("Successfully initialized tape follow task!");
    int lastError = 0;
    while (1) {
        DualTapeSensor_t* sensor;
        if(state.direction == 1) {
            sensor = navigationData->fontTapeSensor;
        } else {
            sensor = navigationData->backTapeSensor;
        }
        read_tape_sensor(sensor);
        int left_mean = sensor->leftValue;
        int right_mean = sensor->rightValue;

        Serial.println("Left" + String(left_mean));
        Serial.println("Right" + String(right_mean));
        vTaskDelay(pdMS_TO_TICKS(500));

        int error = left_mean - right_mean;

        int pid_adjustment_value = pid_follow_tape(error, lastError);
        state.yaw = pid_adjustment_value;
        lastError = error;

        vTaskDelay(delay_ticks);
    }
}

void TaskRotate(void *pvParameters) {
    log_status("Beginning rotate task initialization...");

    NavigationData_t* navigationData = (NavigationData_t*)pvParameters;
    if (checkNavigationData(navigationData))
    {
        log_error("Error: nulls in robotControlData");
        vTaskDelete(NULL);
        return;
    }

    // convert ms delays into ticks
    TickType_t inital_delay_ticks = pdMS_TO_TICKS(ROTATE_INITIAL_DELAY);
    TickType_t poll_rate_ticks = pdMS_TO_TICKS(MOTOR_ADJUSTMENT_DELAY_ROTATING_MS);

    DualTapeSensor_t* tapeSensor = navigationData->fontTapeSensor;
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
                if (xQueueSend(*navigationData->xSharedQueue, &message, portMAX_DELAY) != pdPASS)
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

    int value_left;
    int value_right;

    log_status("Initialized TaskStationTracking");
    bool found_tape = false;
    while (1) {
        // Check sensors
        read_tape_sensor(tapeSensor);
        value_left = tapeSensor->leftValue;
        value_right = 0; // right tape sensor isn't working rn
        // Serial.println("Right" + String (value_right));
        // Serial.println("Left" + String (value_left));
       
        if (value_left > THRESHOLD_SENSOR_SINGLE || value_right > THRESHOLD_SENSOR_SINGLE) {
            found_tape = true;
        } else if ((value_left < THRESHOLD_SENSOR_SINGLE || value_right < THRESHOLD_SENSOR_SINGLE) && (found_tape == true)) {
            state.last_station += state.orientation * state.direction;
            log_status("Passed station!");
            found_tape = false;

            vTaskDelay(pdMS_TO_TICKS(150));
        }
        vTaskDelay(delay);
    }
}


void TaskDocking(void* pvParameters) {
    NavigationData_t* navigationData = (NavigationData_t*)pvParameters;
    if (checkNavigationData(navigationData)) {
        log_error("Error: Tape Awareness data buffer contains nulls!");
        vTaskDelete(NULL);
        return;
    }

    TickType_t delay = pdMS_TO_TICKS(STATION_TRACKING_POLL_DELAY_MS);

    state.direction = -state.direction; // Invert direction
    state.drive_speed = MOTOR_SPEED_DOCKING;
    state.drive_state = DRIVE;

    int value_left;
    int value_right;
    bool found_tape = false;

    DualTapeSensor_t* sensor;
    if(state.direction == 1) {
        sensor = navigationData->fontTapeSensor;
    } else {
        sensor = navigationData->backTapeSensor;
    }

    while (1) {
        // Check sensors
        read_tape_sensor(sensor);
        value_left = sensor->leftValue;
        value_right = 0; // right tape sensor isn't working rn
       
        if ((value_left > THRESHOLD_SENSOR_SINGLE || value_right > THRESHOLD_SENSOR_SINGLE) && !found_tape) {
            state.last_station += state.orientation * state.direction;
            found_tape = true;
            StatusMessage_t message = REACHED_POSITION;
            xQueueSend(*navigationData->xSharedQueue, &message, portMAX_DELAY);
        } 

        vTaskDelay(delay);
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
                set_robot_drive(robot_motors, state.drive_speed * state.direction);
                break;

            case DriveState_t::ROTATE:
                rotate_robot(robot_motors, state.drive_speed * state.helicity);
                break;
        }
        vTaskDelay(MOTOR_UPDATE_DELAY);
    }
}
