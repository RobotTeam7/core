#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <common/robot_motor.h>
#include <common/pin.h>
#include <common/stepper_motor.h>
#include <common/reflectance_sensor.h>
#include <communication/uart.h>
#include <communication/read.h>
#include <communication/decode.h>
#include <communication/send.h>
#include <motion/FreeRTOSConfig.h>
#include <motion/constants.h>
#include <motion/tasks.h>
#include <motion/utils.h>

RobotMotor_t* motor_front_left;
RobotMotor_t* motor_front_right;
RobotMotor_t* motor_back_left;
RobotMotor_t* motor_back_right;

TapeSensor_t* frontTapeSensor;

RobotMotorData_t robotMotors;
TapeAwarenessData_t config_following;
RobotControlData_t config_rotate;

TaskHandle_t xHandleRotating = NULL;
TaskHandle_t xReflectanceHandle = NULL;
TaskHandle_t xHandleFollowing = NULL;
TaskHandle_t xMasterHandle = NULL;

QueueHandle_t xSharedQueue = xQueueCreate(10, sizeof(StatusMessage_t));

typedef enum { TAPE_FOLLOW, ROTATE } ActionType_t;
ActionType_t currentAction = TAPE_FOLLOW;

void TaskMaster(void *pvParameters);
void begin_rotating();
void begin_following();


void setup() {
    Serial.begin(115200);

    initialize_uart();
    begin_uart_read();

    checkResetCause();

    motor_front_left = instantiate_robot_motor(MOTOR_FRONT_LEFT_FORWARD, MOTOR_FRONT_LEFT_REVERSE);
    motor_front_right = instantiate_robot_motor(MOTOR_FRONT_RIGHT_FORWARD, MOTOR_FRONT_RIGHT_REVERSE);
    motor_back_left = instantiate_robot_motor(MOTOR_BACK_LEFT_FORWARD, MOTOR_BACK_LEFT_REVERSE);
    motor_back_right = instantiate_robot_motor(MOTOR_BACK_RIGHT_FORWARD, MOTOR_BACK_RIGHT_REVERSE);

    robotMotors = { motor_front_right, motor_front_left, motor_back_right, motor_back_left };
    frontTapeSensor = instantiate_tape_sensor(13, 15);

    config_following = { &robotMotors, frontTapeSensor, &xSharedQueue };
    config_rotate = { &config_following, &xSharedQueue };

    // check if reflectance polling task was created
    if (xTaskCreate(TaskPollReflectance, "ReflectancePolling", 2048, frontTapeSensor, PRIORITY_REFLECTANCE_POLLING, &xReflectanceHandle) == pdPASS) {
        log_status("Reflectance polling task was created successfully."); 
    } else {
        log_error("Reflectance polling task was not created successfully!");
    }

    // check if task master was created
    if (xTaskCreate(TaskMaster, "MasterTask", 2048, NULL, 3, &xMasterHandle) == pdPASS) {
        log_status("Master task was created successfully.");
    } else {
        log_error("Master task was not created successfully!");
    }

}

void loop()
{
    // monitorStackUsage(&xHandleRotating, &xReflectanceHandle, &xHandleFollowing, &xMasterHandle); // Monitor stack usage periodically
    // delay(500);
}

void TaskMaster(void *pvParameters)
{
    log_status("Beginning master task...");

    while (1) {
        StatusMessage_t receivedMessage;
        switch (currentAction) {
            case ROTATE:
                // Begin rotating and wait for a message that we see the tape
                begin_rotating();
                if (xQueueReceive(xSharedQueue, &receivedMessage, portMAX_DELAY) == pdPASS) {
                    if (receivedMessage == ROTATION_DONE) {
                        log_status("Found tape!");                        
                        vTaskDelete(xHandleRotating);

                        vTaskDelay(pdMS_TO_TICKS(ROTATE_INTO_TAPE_FOLLOW_DELAY));
                    
                        log_status("Moving to tape following...");
                        currentAction = TAPE_FOLLOW;
                    }
                }
                break;

            case TAPE_FOLLOW:
                log_status("Tape following!");
                begin_following();
                if (xQueueReceive(xSharedQueue, &receivedMessage, portMAX_DELAY) == pdPASS) {
                    if (receivedMessage == LOST_TAPE) {
                        log_status("Lost tape!");                        
                        vTaskDelete(xHandleFollowing);
                    
                        log_status("Moving to rotating...");
                        currentAction = ROTATE;
                    }
                } 
                break;
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
    if (xTaskCreate(TaskFollowTape, "Tape Following", 1024, &config_following, PRIORITY_FOLLOW_TAPE, &xHandleFollowing) == pdPASS) {
        log_status("Tape following task was created successfully.");
    } else {
        log_error("Tape following task was not created successfully!");
    }
}


void begin_following_pid() {
    Serial.print("hey whats good");
}
