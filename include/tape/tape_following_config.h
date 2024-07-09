#ifndef TAPE_FOLLOWING_CONFIG_H
#define TAPE_FOLLOWING_CONFIG_H

#include <common/robot_motor.h>
#include <Arduino.h>
#include <CircularBuffer.h>

#define BUFFER_SIZE 5

struct TapeFollowingConfig {
    const RobotMotor* motor_front_right;
    const RobotMotor* motor_front_left;
    const RobotMotor* motor_back_right;
    const RobotMotor* motor_back_left;

    CircularBuffer<int, BUFFER_SIZE>* left_sensor_buffer;
    CircularBuffer<int, BUFFER_SIZE>* right_sensor_buffer;
};

#endif