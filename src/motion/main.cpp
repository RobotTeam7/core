#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <common/robot_motor.h>
#include <common/pin.h>
#include <common/stepper_motor.h>
#include <motion/FreeRTOSConfig.h>
#include <motion/constants.h>
#include <motion/tasks.h>
#include <motion/utils.h>

RobotMotor_t* motor_front_left;
RobotMotor_t* motor_front_right;
RobotMotor_t* motor_back_left;
RobotMotor_t* motor_back_right;

RobotMotorData_t robotMotors;
ReflectanceSensorData_t config_reflectance;
TapeAwarenessData_t config_following;
RobotControlData_t config_rotate;

// CircularBuffer<int, REFLECTANCE_SENSOR_BUFFER_SIZE> leftBuffer;
// CircularBuffer<int, REFLECTANCE_SENSOR_BUFFER_SIZE> rightBuffer;

TaskHandle_t xHandleRotating = NULL;
TaskHandle_t xReflectanceHandle = NULL;
TaskHandle_t xHandleFollowing = NULL;
TaskHandle_t xMasterHandle = NULL;

QueueHandle_t xSharedQueue = xQueueCreate(2, sizeof(Message));

void TaskMaster(void *pvParameters);

void begin_rotating();
void begin_following();

void setup() {
    Serial.begin(115200);

    checkResetCause();

    size_t freeHeap = xPortGetFreeHeapSize();
    Serial.println("Free Heap: " + String(freeHeap));

    motor_front_left = instantiate_robot_motor(MOTOR_FRONT_LEFT_FORWARD, MOTOR_FRONT_LEFT_REVERSE);
    motor_front_right = instantiate_robot_motor(MOTOR_FRONT_RIGHT_FORWARD, MOTOR_FRONT_RIGHT_REVERSE);
    motor_back_left = instantiate_robot_motor(MOTOR_BACK_LEFT_FORWARD, MOTOR_BACK_LEFT_REVERSE);
    motor_back_right = instantiate_robot_motor(MOTOR_BACK_RIGHT_FORWARD, MOTOR_BACK_RIGHT_REVERSE);

    robotMotors = { motor_front_right, motor_front_left, motor_back_right, motor_back_left };

    config_reflectance = { 10, 10 };
    config_following = { &robotMotors, &config_reflectance };
    config_rotate = { &config_following, &xSharedQueue };

    // check if reflectance polling task was created
    if (xTaskCreate(TaskPollReflectance, "ReflectancePolling", 2048, &config_reflectance, PRIORITY_REFLECTANCE_POLLING, &xReflectanceHandle) == pdPASS) {
        log_status("Reflectance polling task was created successfully."); 
    } else {
        log_error("Reflectance polling task was not created successfully!");
    }

    // check if tape following task was created
    if (xTaskCreate(TaskMaster, "MasterTask", 2048, NULL, 1, &xMasterHandle) == pdPASS) {
        log_status("Master task was created successfully.");
    } else {
        log_error("Master task was not created successfully!");
    }
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

        size_t freeHeap = xPortGetFreeHeapSize();
        Serial.println("Free Heap: " + String(freeHeap));

        begin_rotating();

        // // wait until a message is recieved on the shared queue
        // // this saves the value of the message into "receivedMessage"
        // this operation is blocking since we passed "portMAX_DELAY"
        log_status("State 1");

        if (xQueueReceive(xSharedQueue, &receivedMessage, portMAX_DELAY) == pdPASS) {
            if(receivedMessage == ROTATION_DONE) {
                log_status("State 2");
                
                // tape follow for 1 second
                vTaskDelete(xHandleRotating);
                begin_following();

                vTaskDelay(pdMS_TO_TICKS(1500));
                vTaskDelete(xHandleFollowing);           

                log_status("State 3");
            }
        }
    }
}

void begin_rotating() {
    // check if tape following task was created
    if (xTaskCreate(TaskRotate, "TapeRotate", 2048, &config_rotate, PRIORITY_ROTATE, &xHandleRotating) == pdPASS) {
        log_status("Rotate task was created successfully.");
    } else {
        log_error("Rotate task was not created successfully!");
    }
}

void begin_following() {
    // check if tape following task was created
    if (xTaskCreate(TaskFollowTape, "Tape Following", 2048, &config_following, PRIORITY_FOLLOW_TAPE, &xHandleFollowing) == pdPASS) {
        log_status("Tape following task was created successfully.");
    } else {
        log_error("Tape following task was not created successfully!");
    }
}
