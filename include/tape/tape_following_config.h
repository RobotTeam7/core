#ifndef TAPE_FOLLOWING_CONFIG_H
#define TAPE_FOLLOWING_CONFIG_H

#include <common/robot_motor.h>
#include <Arduino.h>
#include <CircularBuffer.h>

#define BUFFER_SIZE 5

struct TapeFollowingConfig {
    RobotMotor* motor_front_right;
    RobotMotor* motor_front_left;
    RobotMotor* motor_back_right;
    RobotMotor* motor_back_left;

    CircularBuffer<int, BUFFER_SIZE>* left_sensor_buffer;
    CircularBuffer<int, BUFFER_SIZE>* right_sensor_buffer;
};

#endif