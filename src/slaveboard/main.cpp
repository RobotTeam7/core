#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <common/robot_motor.h>
#include <common/pin.h>
#include <string.h>
#include <FreeRTOS.h>
#include <task.h>
#include <FreeRTOS/Source/include/queue.h>

// tape following imports
#include <tape/task_follow_tape.h>
#include <reflectance/task_poll_reflectance.h>
#include <task_rotate.h>

#define MOTOR_BACK_RIGHT_FORWARD PB1
#define MOTOR_BACK_RIGHT_REVERSE PB0
#define MOTOR_BACK_LEFT_FORWARD PA7
#define MOTOR_BACK_LEFT_REVERSE PA6
#define MOTOR_FRONT_LEFT_FORWARD PA1
#define MOTOR_FRONT_LEFT_REVERSE PA0
#define MOTOR_FRONT_RIGHT_FORWARD PA3
#define MOTOR_FRONT_RIGHT_REVERSE PA2

#define PRIORITY_REFLECTANCE_POLLING 3
#define PRIORITY_FOLLOW_TAPE 1
#define PRIORITY_ROTATE 1

RobotMotor motor_front_left;
RobotMotor motor_front_right;
RobotMotor motor_back_left;
RobotMotor motor_back_right;

ReflectancePollingConfig config_reflectance;
MotorReflectanceConfig config_motor_reflectance;

CircularBuffer<int, BUFFER_SIZE> leftBuffer;
CircularBuffer<int, BUFFER_SIZE> rightBuffer;

TaskHandle_t xHandleReflectance = NULL;
TaskHandle_t xHandleRotating = NULL;
TaskHandle_t xHandleFollowing = NULL;

// Enum for defining message types, will be changed in the future
typedef enum {
  ROTATION_DONE,
} Message;

QueueHandle_t xSharedQueue = xQueueCreate(10, sizeof(Message));
void TaskMaster(void *pvParameters);

void setup() {
  Serial.begin(9600);
  Serial.println("Got serial!");
  
  motor_front_left = RobotMotor(MOTOR_FRONT_LEFT_FORWARD, MOTOR_FRONT_LEFT_REVERSE);
  motor_front_right = RobotMotor(MOTOR_FRONT_RIGHT_FORWARD, MOTOR_FRONT_RIGHT_REVERSE);
  motor_back_left = RobotMotor(MOTOR_BACK_LEFT_FORWARD, MOTOR_BACK_LEFT_REVERSE);
  motor_back_right = RobotMotor(MOTOR_BACK_RIGHT_FORWARD, MOTOR_BACK_RIGHT_REVERSE);

  config_reflectance = {
    &leftBuffer, &rightBuffer, &xSharedQueue
  };

  config_motor_reflectance = {
    &motor_front_right, &motor_front_left, &motor_back_right, &motor_back_left, &config_reflectance
  };

  BaseType_t xReturnedReflectance = xTaskCreate(TaskPollReflectance, "Reflectance Polling", 200, &config_reflectance, PRIORITY_REFLECTANCE_POLLING, &xHandleReflectance);
  BaseType_t xReturnedRotating = xTaskCreate(TaskRotate, "Rotating", 200, &config_motor_reflectance, PRIORITY_ROTATE, &xHandleRotating);
  BaseType_t xReturnedFollowing = xTaskCreate(TaskFollowTape, "Tape Following", 200, &config_motor_reflectance, PRIORITY_FOLLOW_TAPE, &xHandleFollowing);
  BaseType_t xReturnedMaster = xTaskCreate(TaskMaster, "Master", 200, NULL, 1, NULL);

  // suspend driving tasks initially
  vTaskSuspend(xHandleRotating);
  vTaskSuspend(xHandleFollowing);

  // check if reflectance polling task was created
  if (xReturnedReflectance == pdPASS) {
    Serial.println("Reflectance polling task was created successfully.");
  } else {
    Serial.println("Reflectance polling task was not created successfully!");
  }

  // check if tape following task was created
  if (xReturnedFollowing == pdPASS) {
    Serial.println("Tape following task was created successfully.");
  } else {
    Serial.println("Tape following task was not created successfully!");
  }

  // // check if rotating task was created
  if (xReturnedRotating == pdPASS) {
    Serial.println("Rotating task was created successfully.");
  } else {
    Serial.println("Rotating task was not created successfully!");
  }

  // // check if rotating task was created
  if (xReturnedMaster == pdPASS) {
    Serial.println("Rotating task was created successfully.");
  } else {
    Serial.println("Rotating task was not created successfully!");
  }

  vTaskStartScheduler();
}

void TaskMaster(void *pvParameters) {
  Message receivedMessage;
  for(;;) {
    // rotate until completion
    vTaskSuspend(xHandleFollowing);
    Serial.println("chill for a sec!");
    vTaskDelay(pdMS_TO_TICKS(1000));
    Serial.println("Rotating!");
    vTaskResume(xHandleRotating);

    // wait until a message is recieved on the shared queue
    // this saves the value of the message into "receivedMessage"
    // this operation is blocking since we passed "portMAX_DELAY"
    if (xQueueReceive(xSharedQueue, &receivedMessage, portMAX_DELAY) == pdPASS) {
      if(receivedMessage == ROTATION_DONE) {
        Serial.println("Tape Following!");
        // tape follow for 1 second
        vTaskResume(xHandleFollowing);
        vTaskSuspend(xHandleRotating);
        vTaskDelay(pdMS_TO_TICKS(1500));
      }
    }
  }
}

void loop() {
}