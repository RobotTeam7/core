#ifndef SLAVEBOARD_TASKS_H
#define SLAVEBOARD_TASKS_H

#include <Arduino.h>
#include <common/robot_motor.h>
#include <CircularBuffer.h>
#include <FreeRTOS/Source/include/queue.h>
#include <slaveboard/constants.h>
#include <slaveboard/utils.h>

/**
 * @brief Enum discretizing the different kinds of messages that can be sent between the master task and subtasks
 */
enum Message { ROTATION_DONE };

/**
 * @brief Obtain the average of the values contained within the buffer, as an integer
 */
int get_buffer_average(CircularBuffer<int, REFLECTANCE_SENSOR_BUFFER_SIZE> &sensorBuffer);

typedef struct {
    RobotMotor* motorFR;
    RobotMotor* motorFL;
    RobotMotor* motorBR;
    RobotMotor* motorBL;
} RobotMotorData_t;

typedef struct {
    CircularBuffer<int, REFLECTANCE_SENSOR_BUFFER_SIZE>* leftSensorBuffer;
    CircularBuffer<int, REFLECTANCE_SENSOR_BUFFER_SIZE>* rightSensorBuffer;
} ReflectanceSensorData_t;

typedef struct {
    RobotMotorData_t* robotMotors;
    ReflectanceSensorData_t* reflectanceSensorData;
} TapeAwarenessData_t;

typedef struct {
    TapeAwarenessData_t* tapeAwarenessData;
    QueueHandle_t* xSharedQueue;
} RobotControlData_t;

void TaskRotate(void *pvParameters);
void TaskFollowTape(void *pvParameters);
void TaskPollReflectance(void *pvParameters);

#endif 