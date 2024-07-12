#ifndef TAPE_FOLLOWING_CONFIG_H
#define TAPE_FOLLOWING_CONFIG_H

#include <common/robot_motor.h>
#include <Arduino.h>
#include <CircularBuffer.h>
#include <tape/reflectance_polling_config.h>


struct TapeFollowingConfig {
    RobotMotor* motor_front_right;
    RobotMotor* motor_front_left;
    RobotMotor* motor_back_right;
    RobotMotor* motor_back_left;
    
    ReflectancePollingConfig* reflectancePollingConfig;
};

#endif