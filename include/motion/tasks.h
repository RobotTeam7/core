#ifndef MOTION_TASKS_H
#define MOTION_TASKS_H

#include <Arduino.h>
#include <common/robot_motor.h>
#include <CircularBuffer.h>
#include <motion/constants.h>
#include <motion/utils.h>

/**
 * @brief Enum discretizing the different kinds of messages that can be sent between the master task and subtasks
 */
enum Message { ROTATION_DONE, LOST_TAPE };

typedef struct {
    RobotMotor_t* motorFR;
    RobotMotor_t* motorFL;
    RobotMotor_t* motorBR;
    RobotMotor_t* motorBL;
} RobotMotorData_t;

typedef struct {
    RobotMotorData_t* robotMotors;
    TapeSensor_t* tapeSensor;
    QueueHandle_t* xSharedQueue;
} TapeAwarenessData_t;

typedef struct {
    TapeAwarenessData_t* tapeAwarenessData;
    QueueHandle_t* xSharedQueue;
} RobotControlData_t;

void TaskRotate(void *pvParameters);
void TaskFollowTape(void *pvParameters);
void TaskPollReflectance(void *pvParameters);
void TaskFollowPID(void *pvParameters);


#endif 