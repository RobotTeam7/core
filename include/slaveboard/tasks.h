#ifndef SLAVEBOARD_TASKS_H
#define SLAVEBOARD_TASKS_H

#include <Arduino.h>
#include <common/robot_motor.h>
#include <CircularBuffer.h>
#include <FreeRTOS/Source/include/queue.h>

#define BUFFER_SIZE 1

/**
 * @brief Enum discretizing the different kinds of messages that can be sent between the master task and subtasks
 */
enum Message { ROTATION_DONE };

/**
 * @brief Obtain the average of the values contained within the buffer, as an integer
 */
int get_buffer_average(CircularBuffer<int, BUFFER_SIZE> &sensor_buffer);

struct ReflectancePollingConfig {
    CircularBuffer<int, BUFFER_SIZE>* left_sensor_buffer;
    CircularBuffer<int, BUFFER_SIZE>* right_sensor_buffer;
};

struct TapeFollowingConfig {
    RobotMotor* motor_front_right;
    RobotMotor* motor_front_left;
    RobotMotor* motor_back_right;
    RobotMotor* motor_back_left;
    
    ReflectancePollingConfig* reflectancePollingConfig;
};

struct MotorReflectanceConfig {
    RobotMotor* motor_front_right;
    RobotMotor* motor_front_left;
    RobotMotor* motor_back_right;
    RobotMotor* motor_back_left;
    
    ReflectancePollingConfig* reflectancePollingConfig;
    QueueHandle_t* xSharedQueue;
};

void TaskRotate(void *pvParameters);
void TaskFollowTape(void *pvParameters);
void TaskPollReflectance(void *pvParameters);

#endif 