#include <Arduino.h>
#include <FreeRTOS.h>
#include <task.h>
#include <slaveboard/constants.h>
#include <slaveboard/tasks.h>

#define POLL_SENSOR_DELAY_MS 1
#define MOTOR_TASK_DELAY_MS 10

#define THRESHOLD 300
#define REFLECTANCE_ONE PA5
#define REFLECTANCE_TWO PA4

// returns the mean value in the buffer
int get_buffer_average(CircularBuffer<int, REFLECTANCE_SENSOR_BUFFER_SIZE> &sensor_buffer)
{
  int sum = 0;
  for (int i = 0; i < sensor_buffer.size(); ++i)
  {
    sum += sensor_buffer[i];
  }
  // truncates integer
  return sum / sensor_buffer.size();
}

int checkRobotMotors(RobotMotorData_t* robotMotors) {
    if (robotMotors == NULL) {
        return 1;
    } else {
        return robotMotors->motorFR == NULL || robotMotors->motorBR == NULL || robotMotors->motorFL == NULL || robotMotors->motorBL == NULL;
    } 
}

int checkReflectanceSensorData(ReflectanceSensorData_t* reflectanceData) {
    if (reflectanceData == NULL) {
        return 1;
    } else {
        return reflectanceData->leftSensorBuffer == NULL || reflectanceData->rightSensorBuffer == NULL; 
    }
}

int checkTapeAwarenessData(TapeAwarenessData_t* tapeAwarenessData) {
    if (tapeAwarenessData == NULL) {
        return 1;
    } else {
        return checkReflectanceSensorData(tapeAwarenessData->reflectanceSensorData) == 1 || checkRobotMotors(tapeAwarenessData->robotMotors) == 1; 
    }
}

void TaskPollReflectance(void *pvParameters) {
    log_status("Beginning reflectance task initialization...");
    
    // Acquire and verify pointers
    ReflectanceSensorData_t* reflectanceData = (ReflectanceSensorData_t*)pvParameters;
    if (checkReflectanceSensorData(reflectanceData)) {
        log_error("Error: Null pointers in reflectance sensor data!");  
        vTaskDelete(NULL);
        return;
    }

    // Setup reflectance sensor
    TickType_t delay_ticks = pdMS_TO_TICKS(POLL_SENSOR_DELAY_MS);

    pinMode(REFLECTANCE_ONE, INPUT);
    pinMode(REFLECTANCE_TWO, INPUT);

    int reflectance_right;
    int reflectance_left;

    log_status("Succesfully initialized reflectance polling task!");

    while (1) {
        // Read sensor values, pushing them onto the buffer
        reflectance_right = analogRead(REFLECTANCE_ONE);
        reflectance_left = analogRead(REFLECTANCE_TWO);

        reflectanceData->rightSensorBuffer->push(reflectance_right);
        reflectanceData->leftSensorBuffer->push(reflectance_left);

        // Print direction of less reflectance (higher value -> less reflectance, they are inversly proportional)
        if (VERBOSITY_LEVEL > MOST_VERBOSE) {
            char drive_state[20] = "find tape";  // this could be a memory unsafe situation, as some chars are uninitialized
            if (reflectance_right - reflectance_left > THRESHOLD) {
                strcpy(drive_state, "---->>");
            } else if (reflectance_left - reflectance_right > THRESHOLD) {
                strcpy(drive_state, "<<----");
            } else {
                strcpy(drive_state, "^^^^^^");
            }
            Serial.println(drive_state);
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
        int left_mean = get_buffer_average(*tapeAwarenessData->reflectanceSensorData->leftSensorBuffer);
        int right_mean = get_buffer_average(*tapeAwarenessData->reflectanceSensorData->rightSensorBuffer);
        
        if (right_mean - left_mean > THRESHOLD) {                       // If we are detecting tape on the left, turn left
            log_message("Driving left...");
            robotMotors->motorFL->set_drive(MOTOR_SPEED_LOW, forward); 
            robotMotors->motorBL->set_drive(MOTOR_SPEED_LOW, forward);
            robotMotors->motorFR->set_drive(MOTOR_SPEED_HIGH, forward);
            robotMotors->motorBR->set_drive(MOTOR_SPEED_HIGH, forward);
        } else if (left_mean - right_mean > THRESHOLD) {               // If we are detecting tape on the right, turn right
            log_message("Driving right...");
            robotMotors->motorFL->set_drive(MOTOR_SPEED_HIGH, forward);
            robotMotors->motorBL->set_drive(MOTOR_SPEED_HIGH, forward);
            robotMotors->motorFR->set_drive(MOTOR_SPEED_LOW, forward);
            robotMotors->motorBR->set_drive(MOTOR_SPEED_LOW, forward);
        } else {                                                        // Otherwise, drive forwards
            log_message("Driving forward...");
            robotMotors->motorFL->set_drive(MOTOR_SPEED_HIGH, forward); 
            robotMotors->motorBL->set_drive(MOTOR_SPEED_HIGH, forward);
            robotMotors->motorFR->set_drive(MOTOR_SPEED_HIGH, forward);
            robotMotors->motorBR->set_drive(MOTOR_SPEED_HIGH, forward);
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
    ReflectanceSensorData_t* reflectanceSensorData = robotControlData->tapeAwarenessData->reflectanceSensorData;
    bool rotating;

    log_status("Successfully initialized rotation!");

    while (1) {
        // start rotating, then delay to ensure that rotation isn't immediately canceled by tape detection
        robotMotors->motorFR->set_drive(MOTOR_SPEED_ROTATION, forward);
        robotMotors->motorBR->set_drive(MOTOR_SPEED_ROTATION, forward);
        robotMotors->motorFL->set_drive(MOTOR_SPEED_ROTATION, reverse);
        robotMotors->motorBL->set_drive(MOTOR_SPEED_ROTATION, reverse);
        
        vTaskDelay(inital_delay_ticks);

        // look for tape detection
        rotating = true;
        
        log_message("Looking for tape...");

        while (rotating)
        {
            int left_mean = get_buffer_average(*(reflectanceSensorData->leftSensorBuffer));
            int right_mean = get_buffer_average(*(reflectanceSensorData->rightSensorBuffer));

            if (right_mean > THRESHOLD_SENSOR_SINGLE && left_mean > THRESHOLD_SENSOR_SINGLE)
            {
                log_status("Found tape. Ending rotation...");
                rotating = false;

                robotMotors->motorFR->stop();
                robotMotors->motorBR->stop();
                robotMotors->motorFL->stop();
                robotMotors->motorBL->stop();

                // send message to TaskMaster that rotation has finished
                Message message = ROTATION_DONE;
                if (xQueueSend(*robotControlData->xSharedQueue, &message, portMAX_DELAY) != pdPASS)
                {
                    log_error("Failed to send to xSharedQueue");
                }
            }

            vTaskDelay(poll_rate_ticks);
        }
    }
}