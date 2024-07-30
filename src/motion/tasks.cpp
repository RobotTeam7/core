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




TaskHandle_t xHandleRotating = NULL;
TaskHandle_t xDriveHandle = NULL;
TaskHandle_t xHandleFollowing = NULL;
TaskHandle_t xStationTrackingHandle = NULL;
TaskHandle_t xDockingHandle = NULL;
TaskHandle_t xCounterDockingHandle = NULL;
TaskHandle_t xReturnToTapeHandle = NULL;
TaskHandle_t xFollowWallHandle = NULL;

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

// Ensure that a NavigationData_t* does not contain null values
int checkTapeReturn(ReturnToTapeData_t* navigationData) {
    return navigationData == NULL || checkTapeSensor(navigationData->backTapeSensor) || checkTapeSensor(navigationData->fontTapeSensor) == 1 || navigationData->masterHandle == NULL;
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

        vTaskDelete(xHandleFollowing);
        xHandleFollowing = NULL;

        return;
    }

    state.drive_state = DriveState_t::DRIVE;
    state.drive_speed = MOTOR_SPEED_FOLLOWING;

    int lastError = 0;
    DualTapeSensor_t* sensor;
    if (state.direction == 1) {
        sensor = navigationData->fontTapeSensor;
    } else {
        sensor = navigationData->backTapeSensor;
    }

    log_status("Successfully initialized tape follow task!");

    while (1) {
        read_tape_sensor(sensor);
        int left_mean = sensor->leftValue;
        int right_mean = sensor->rightValue;

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

        vTaskDelete(xHandleRotating);
        xHandleRotating = NULL;

        return;
    }

    // convert ms delays into ticks
    TickType_t inital_delay_ticks = pdMS_TO_TICKS(ROTATE_INITIAL_DELAY);
    TickType_t poll_rate_ticks = pdMS_TO_TICKS(MOTOR_ADJUSTMENT_DELAY_ROTATING_MS);

    DualTapeSensor_t* tapeSensor = state.direction == 1 ? navigationData->fontTapeSensor : navigationData->backTapeSensor;
    
    int count = 0;
    bool on_tape = false;
    log_status("Successfully initialized rotation!");

    state.drive_state = DriveState_t::ROTATE;
    state.drive_speed = MOTOR_SPEED_ROTATION;

    vTaskDelay(pdMS_TO_TICKS(ROTATE_INITIAL_DELAY));

    while (1) {
        // start rotating, then delay to ensure that rotation isn't immediately canceled by tape detection
        vTaskDelay(inital_delay_ticks);
        
        log_message("Looking for tape...");

        while (count < 2)
        {
            read_tape_sensor(tapeSensor);
            int left_mean = tapeSensor->leftValue;
            int right_mean = tapeSensor->rightValue;

            if ((right_mean > THRESHOLD_SENSOR_SINGLE || left_mean > THRESHOLD_SENSOR_SINGLE)) {
                // vTaskDelay(pdMS_TO_TICKS(25));
                log_status("Found tape. Ending rotation...");
                state.drive_state = DriveState_t::STOP;
                state.drive_speed = 0;
                state.orientation = -state.orientation;

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

    TickType_t delay = pdMS_TO_TICKS(DELAY_STATION_TRACKING_POLL);

    int value_left;
    int value_right;

    log_status("Initialized TaskStationTracking");
    bool found_tape = false;
    while (1) {
        // Check sensors
        read_tape_sensor(tapeSensor);
        value_left = tapeSensor->leftValue;
        value_right = tapeSensor->rightValue;

        // Serial.println("left sensor: " + String(value_left));
        // Serial.println("right sensor: " + String(value_right));
       
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

// Ensure that a NavigationData_t* does not contain null values
int checkDockingData(DockingData_t* dockingData) {
    return dockingData == NULL || checkTapeSensor(dockingData->wingSensor);
}

void TaskDocking(void* pvParameters) {
    log_status("Initializing docking...");
    
    DockingData_t* dockingData = (DockingData_t*)pvParameters;

    if (checkDockingData(dockingData)) {
        log_error("Error: Tape Awareness data buffer contains nulls!");

        vTaskDelete(xDockingHandle);
        xDockingHandle = NULL;

        return;
    }

    DualTapeSensor_t* sensor = dockingData->wingSensor;
    TickType_t delay = pdMS_TO_TICKS(DELAY_STATION_TRACKING_POLL);

    int value_left;
    int value_right;

    log_status("Initialized docking!");

    while (1) {
        // Check sensors
        read_tape_sensor(sensor);
        value_left = sensor->leftValue;
        value_right = sensor->rightValue; // right tape sensor isn't working rn
        // Serial.println("Right Sensor: " + String(value_right));
        // Serial.println("Left Sensor: " + String(value_left));
       
        if ((value_left > THRESHOLD_SENSOR_SINGLE || value_right > THRESHOLD_SENSOR_SINGLE)) {
            // state.last_station += state.orientation * state.direction;
            StatusMessage_t message = REACHED_POSITION;
            xQueueSend(*dockingData->xSharedQueue, &message, portMAX_DELAY);
            log_status("Finished docking!");
            state.drive_state = STOP;
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

            case DriveState_t::TRANSLATE:
                translate_robot(robot_motors, state.drive_speed * -state.y_direction * state.orientation);
                break;

            case DriveState_t::ROTATE_AND_TRANSLATE:
                // translational velocity accepts the opposite of the y-direction
                pirouette_robot(robot_motors, MOTOR_SPEED_PIROUETTE_ROTATION * state.helicity, state.drive_speed * -state.y_direction * state.orientation, state.pirouette_angle);
                // pirouette_robot(robot_motors, 0 * state.helicity, state.drive_speed * -state.y_direction * state.orientation, state.pirouette_angle);
                break;
        }
        vTaskDelay(MOTOR_UPDATE_DELAY);
    }
}

void TaskCounterDocking(void* pvParameters) {
    log_status("Initializing counter docking task...");

    TaskHandle_t* xMasterHandle = (TaskHandle_t*)pvParameters;
    if (xMasterHandle == NULL || *xMasterHandle == NULL) {
        log_error("Nulls in master task handle!");
        vTaskDelete(NULL);
        return;
    }

    uint32_t ulNotificationValue;
    while (1) {
        // Wait to be notified that limit switch hit the counter
        vTaskDelay(pdMS_TO_TICKS(950));
        log_status("Hit counter!");

        // Stop moving
        state.drive_speed = 0;
        state.drive_state = STOP;

        // Notify master that we reached the counter
        xTaskNotifyGive(*xMasterHandle);

        // Task is over
        vTaskDelete(xCounterDockingHandle);
        xCounterDockingHandle = NULL;
    }
}

void TaskReturnToTape(void* pvParameters) {
    log_status("Beginning tape return task initialization...");

    ReturnToTapeData_t* navigationData = (ReturnToTapeData_t*)pvParameters;
    if (checkTapeReturn(navigationData))
    {
        log_error("Error: nulls in navigationData");

        vTaskDelete(xReturnToTapeHandle);
        xReturnToTapeHandle = NULL;

        return;
    }

    // convert ms delays into ticks
    TickType_t poll_rate_ticks = pdMS_TO_TICKS(DELAY_RETURN_TO_TAPE_POLL);
    DualTapeSensor_t* tapeSensor = state.direction == 1 ? navigationData->fontTapeSensor : navigationData->backTapeSensor;

    log_status("Successfully initialized tape return!");

    state.drive_state = DriveState_t::TRANSLATE;
    state.drive_speed = MOTOR_SPEED_TRANSLATION;

    log_message("Looking for tape...");

    while (1) {
        // look for tape detection
        read_tape_sensor(tapeSensor);
        int left_mean = tapeSensor->leftValue;
        int right_mean = tapeSensor->rightValue;
        // Serial.println("Left" + String(left_mean));
        // Serial.println("Right" + String(right_mean));

        if (right_mean > THRESHOLD_SENSOR_SINGLE || left_mean > THRESHOLD_SENSOR_SINGLE)
        {
            log_status("Found tape. Ending return to tape...");

            // send message to TaskMaster that return to tape has finished
            xTaskNotifyGive(*navigationData->masterHandle);

            state.drive_state = DriveState_t::STOP;
            state.drive_speed = 0;

            vTaskDelete(NULL);
            xReturnToTapeHandle = NULL;
            taskYIELD();

            break;
        }

        vTaskDelay(poll_rate_ticks);
    }
}

int checkFullSensorData(FullSensorData_t* fullSensorData) {
    return fullSensorData == NULL || checkTapeSensor(fullSensorData->wingSensor) || checkTapeSensor(fullSensorData->fontTapeSensor) || checkTapeSensor(fullSensorData->backTapeSensor);
}

void TaskFollowWall(void* pvParameters) {
    FullSensorData_t* fullSensorData = (FullSensorData_t*)pvParameters;

    if (checkFullSensorData(fullSensorData)) {
        log_error("Error: Full Sensor Data in Follow Wall contains nulls!");
        vTaskDelete(xFollowWallHandle);
        xFollowWallHandle = NULL;
        return;
    }

    DualTapeSensor_t* wingSensor = fullSensorData->wingSensor;
    DualTapeSensor_t* sensor = state.direction == 1 ? fullSensorData->fontTapeSensor : fullSensorData->backTapeSensor;

    int value_left;  // these are wing sensor values
    int value_right;
    bool found_tape = false;

    log_status("Successfully initialized TaskFollowWall");

    while (1) {
        //
        read_tape_sensor(sensor);
        value_left = sensor->leftValue;
        value_right = sensor->rightValue;
        // value_left = 99;
        // value_right = 99;

        Serial.println("left sensor: " + String(value_left));
        Serial.println("right sensor: " + String(value_right));

        if (value_left > THRESHOLD_SENSOR_SINGLE || value_right > THRESHOLD_SENSOR_SINGLE) {
            found_tape = true;
        } else if ((value_left < THRESHOLD_SENSOR_SINGLE || value_right < THRESHOLD_SENSOR_SINGLE) && (found_tape == true)) {
            state.last_side_station += state.orientation * state.direction;
            log_status("Passed station while wall following!");
            found_tape = false;
            // vTaskDelay(pdMS_TO_TICKS(150));
        }

        vTaskDelay(pdMS_TO_TICKS(DELAY_WALL_SLAMMING_POLL));
    }
    Serial.println("Exited loop!");
}
