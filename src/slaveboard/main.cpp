#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <string.h>
#include <FreeRTOS.h>
#include <task.h>

#include <slaveboard/constants.h>
#include <slaveboard/FreeRTOSConfig.h>
#include <slaveboard/tasks.h>
#include <common/robot_motor.h>
#include <common/pin.h>
#include <slaveboard/utils.h>

RobotMotor motor_front_left;
RobotMotor motor_front_right;
RobotMotor motor_back_left;
RobotMotor motor_back_right;

RobotMotorData_t robotMotors;
ReflectanceSensorData_t config_reflectance;
TapeAwarenessData_t config_following;
RobotControlData_t config_rotate;

CircularBuffer<int, REFLECTANCE_SENSOR_BUFFER_SIZE> leftBuffer;
CircularBuffer<int, REFLECTANCE_SENSOR_BUFFER_SIZE> rightBuffer;

TaskHandle_t xHandleRotating = NULL;
TaskHandle_t xReflectanceHandle = NULL;
TaskHandle_t xHandleFollowing = NULL;
TaskHandle_t xMasterHandle = NULL;

QueueHandle_t xSharedQueue = xQueueCreate(10, sizeof(Message));

void TaskMaster(void *pvParameters);

void setup() {
    Serial.begin(9600);

    checkResetCause();

    motor_front_left = RobotMotor(MOTOR_FRONT_LEFT_FORWARD, MOTOR_FRONT_LEFT_REVERSE);
    motor_front_right = RobotMotor(MOTOR_FRONT_RIGHT_FORWARD, MOTOR_FRONT_RIGHT_REVERSE);
    motor_back_left = RobotMotor(MOTOR_BACK_LEFT_FORWARD, MOTOR_BACK_LEFT_REVERSE);
    motor_back_right = RobotMotor(MOTOR_BACK_RIGHT_FORWARD, MOTOR_BACK_RIGHT_REVERSE);

    robotMotors = { &motor_front_right, &motor_front_left, &motor_back_right, &motor_back_left};

    config_reflectance = { &leftBuffer, &rightBuffer };
    config_following = { &robotMotors, &config_reflectance };
    config_rotate = { &config_following, &xSharedQueue};

    // check if reflectance polling task was created
    if (xTaskCreate(TaskPollReflectance, "ReflectancePolling", 72, &config_reflectance, PRIORITY_REFLECTANCE_POLLING, &xReflectanceHandle) == pdPASS) {
        Serial.println("Reflectance polling task was created successfully."); 
    } else {
        Serial.println("Reflectance polling task was not created successfully!");
    }

    // check if tape following task was created
    if (xTaskCreate(TaskFollowTape, "Tape Following", 64, &config_following, PRIORITY_FOLLOW_TAPE, &xHandleFollowing) == pdPASS) {
        Serial.println("Tape following task was created successfully.");
    } else {
        Serial.println("Tape following task was not created successfully!");
    }

    // check if tape following task was created
    if (xTaskCreate(TaskRotate, "TapeRotate", 64, &config_rotate, PRIORITY_FOLLOW_TAPE, &xHandleRotating) == pdPASS) {
        Serial.println("Rotate task was created successfully.");
    } else {
        Serial.println("Rotate task was not created successfully!");
    }

    // check if tape following task was created
    if (xTaskCreate(TaskMaster, "MasterTask", 64, NULL, 1, &xMasterHandle) == pdPASS) {
        Serial.println("Master task was created successfully.");
    } else {
        Serial.println("Master task was not created successfully!");
    }

    vTaskSuspend(xHandleRotating);
    vTaskSuspend(xHandleFollowing);

    size_t freeHeap = xPortGetFreeHeapSize();
    Serial.println("Free Heap: " + String(freeHeap));

    vTaskStartScheduler();
}

void loop()
{
    monitorStackUsage(&xHandleRotating, &xReflectanceHandle, &xHandleFollowing, &xMasterHandle); // Monitor stack usage periodically
    delay(500);
}

void TaskMaster(void *pvParameters)
{
    log_status("Beginning master task...");
    Message receivedMessage;
    while (1) {
        log_status("State 0");
        // rotate until completion
        vTaskSuspend(xHandleFollowing);

        vTaskDelay(pdMS_TO_TICKS(1000));

        vTaskResume(xHandleRotating);

        // // wait until a message is recieved on the shared queue
        // // this saves the value of the message into "receivedMessage"
        // this operation is blocking since we passed "portMAX_DELAY"
        log_status("State 1");

        if (xQueueReceive(xSharedQueue, &receivedMessage, portMAX_DELAY) == pdPASS) {
            if(receivedMessage == ROTATION_DONE) {
                log_status("State 2");
                
                // tape follow for 1 second
                vTaskResume(xHandleFollowing);
                vTaskSuspend(xHandleRotating);
                vTaskDelay(pdMS_TO_TICKS(1500));

                log_status("State 3");
            }
        }
    }
}
