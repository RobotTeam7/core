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

ReflectancePollingConfig config_reflectance;
TapeFollowingConfig config_following;
MotorReflectanceConfig config_rotate;

CircularBuffer<int, BUFFER_SIZE> leftBuffer;
CircularBuffer<int, BUFFER_SIZE> rightBuffer;

TaskHandle_t xHandleRotating = NULL;
TaskHandle_t xReflectanceHandle = NULL;
TaskHandle_t xHandleFollowing = NULL;
TaskHandle_t xMasterHandle = NULL;

QueueHandle_t xSharedQueue = xQueueCreate(10, sizeof(Message));

void TaskMaster(void *pvParameters);

void setup()
{
  Serial.begin(9600);

  checkResetCause();

  motor_front_left = RobotMotor(MOTOR_FRONT_LEFT_FORWARD, MOTOR_FRONT_LEFT_REVERSE);
  motor_front_right = RobotMotor(MOTOR_FRONT_RIGHT_FORWARD, MOTOR_FRONT_RIGHT_REVERSE);
  motor_back_left = RobotMotor(MOTOR_BACK_LEFT_FORWARD, MOTOR_BACK_LEFT_REVERSE);
  motor_back_right = RobotMotor(MOTOR_BACK_RIGHT_FORWARD, MOTOR_BACK_RIGHT_REVERSE);

  config_reflectance = { &leftBuffer, &rightBuffer };
  config_following = { &motor_front_right, &motor_front_left, &motor_back_right, &motor_back_left, &config_reflectance };
  config_rotate = { &motor_front_right, &motor_front_left, &motor_back_right, &motor_back_left, &config_reflectance, &xSharedQueue};

  BaseType_t xReturnedReflectance = xTaskCreate(TaskPollReflectance, "ReflectancePolling", 72, &config_reflectance, PRIORITY_REFLECTANCE_POLLING, &xReflectanceHandle);
  BaseType_t xReturnedFollowing = xTaskCreate(TaskFollowTape, "Tape Following", 64, &config_following, PRIORITY_FOLLOW_TAPE, &xHandleFollowing);
  BaseType_t xReturnedRotate = xTaskCreate(TaskRotate, "TapeRotate", 64, &config_rotate, PRIORITY_FOLLOW_TAPE, &xHandleRotating);
  BaseType_t xMasterReturned = xTaskCreate(TaskMaster, "MasterTask", 64, NULL, 1, &xMasterHandle);

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

  // check if tape following task was created
  if (xReturnedRotate == pdPASS) {
    Serial.println("Rotate task was created successfully.");
  } else {
    Serial.println("Rotate task was not created successfully!");
  }

  // check if tape following task was created
  if (xMasterReturned == pdPASS) {
    Serial.println("Master task was created successfully.");
  } else {
    Serial.println("Master task was not created successfully!");
  }

  size_t freeHeap = xPortGetFreeHeapSize();
  Serial.println("Free Heap: " + String(freeHeap));

  vTaskStartScheduler();
}

void loop()
{
  monitorStackUsage(&xHandleRotating, &xReflectanceHandle, &xHandleFollowing, &xMasterHandle); // Monitor stack usage periodically
  delay(100);
}

void TaskMaster(void *pvParameters)
{
  Message receivedMessage;
  while (1)
  {
    // rotate until completion
    vTaskSuspend(xHandleFollowing);
    Serial.println("chill for a sec!");
    vTaskDelay(pdMS_TO_TICKS(1000));
    Serial.println("Rotating!");
    vTaskResume(xHandleRotating);

    // // wait until a message is recieved on the shared queue
    // // this saves the value of the message into "receivedMessage"
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
