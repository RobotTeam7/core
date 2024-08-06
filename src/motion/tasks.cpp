#include <common/hal.h>

#include <communication/communication.h>

#include <motion/constants.h>
#include <motion/tasks.h>
#include <motion/state.h>
#include <motion/drive.h>


TaskHandle_t xDriveHandle = NULL;
TaskHandle_t xCounterDockingHandle = NULL;
TaskHandle_t xFollowWallHandle = NULL;
TaskHandle_t xHomingHandle = NULL;

// Ensure that a RobotMotorData_t* does not contain null values
int checkRobotMotors(RobotMotorData_t* robotMotors) {
    if (robotMotors == NULL) {
        return 1;
    } else {
        return robotMotors->motorFR == NULL || robotMotors->motorBR == NULL || robotMotors->motorFL == NULL || robotMotors->motorBL == NULL;
    } 
}

// Ensure that a DualTapeSensor_t* does not include null values
int checkTapeSensor(TapeSensor_t* tapeSensor) {
    return tapeSensor == NULL;
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
                pirouette_robot(robot_motors, MOTOR_SPEED_PIROUETTE_ROTATION * state.helicity, state.drive_speed * -state.y_direction * state.orientation, state.pirouette_angle);
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
        vTaskDelay(pdMS_TO_TICKS(DELAY_TRANSLATE_TO_WALL));
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

int checkFullSensorData(FullSensorData_t* fullSensorData) {
    return fullSensorData == NULL || checkTapeSensor(fullSensorData->frontTapeSensor) || checkTapeSensor(fullSensorData->backTapeSensor);
}

void TaskFollowWall(void* pvParameters) {
    FullSensorData_t* fullSensorData = (FullSensorData_t*)pvParameters;

    if (checkFullSensorData(fullSensorData)) {
        log_error("Error: Full Sensor Data in Follow Wall contains nulls!");
        vTaskDelete(xFollowWallHandle);
        xFollowWallHandle = NULL;
        return;
    }

    TapeSensor_t* sensor = state.direction == 1 ? fullSensorData->frontTapeSensor : fullSensorData->backTapeSensor;

    int value;  // these are wing sensor values
    bool found_tape = false;

    log_status("Successfully initialized TaskFollowWall");

    while (1) {
        read_tape_sensor(sensor);

        value = sensor->value;

        // Serial.println("sensor: " + String(value));

        if (value > THRESHOLD_SENSOR_SINGLE) {
            found_tape = true;
        } else if ((value < THRESHOLD_SENSOR_SINGLE) && (found_tape == true)) {
            state.last_side_station += state.orientation * state.direction;
            log_status("Passed station while wall following!");
            found_tape = false;
            // vTaskDelay(pdMS_TO_TICKS(150));
        }

        vTaskDelay(pdMS_TO_TICKS(DELAY_WALL_SLAMMING_POLL));
    }
    Serial.println("Exited loop!");
}
void TaskHoming(void* pvParameters) {
    ReturnToTapeData_t* returnToTapeData = (ReturnToTapeData_t*)pvParameters;

    if(checkTapeSensor(returnToTapeData->middleTapeSensor)) {
        log_error("middle tape sensor is null");
        vTaskDelete(xHomingHandle);
        xHomingHandle = NULL;
        return;
    }
    
    TapeSensor_t* sensor = returnToTapeData->middleTapeSensor;

    state.drive_speed = MOTOR_SPEED_HOMING;
    state.drive_state = DRIVE;
    state.yaw = 0;
    
    // delay is initially high since we are initially fast, but lowers after the first detection
    int delay_ms = 300;

    while(1) {
        read_tape_sensor(sensor);
        Serial.println(String(sensor->value));

        if(sensor->value >= 2000) {
            Serial.println("I see tape!" + String(sensor->value));

            state.drive_state = STOP;
            vTaskDelay(pdMS_TO_TICKS(delay_ms));

            read_tape_sensor(sensor);
            if(sensor->value >= 2000) {
                // while(1) {
                //     log_status("IM ON TAPE WOOP WOOP");
                //     vTaskDelay(1000);
                // }
                xTaskNotifyGive(*returnToTapeData->masterHandle);
                log_status("finished homing!");
                vTaskDelete(NULL);
                xHomingHandle = NULL;
            }else {
                delay_ms = 50;
                // we must have passed the tape, so we look for it in the opposite direction
                state.direction = -state.direction;
                // not too sure if we should just set yaw to zero for this function
                state.yaw = -state.yaw;
                state.drive_state = DRIVE;

                // for each oscillation we kill speed slightly
            }
        }
        vTaskDelay(pdMS_TO_TICKS(DELAY_HOMING_POLL));
    }


}