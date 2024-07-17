#ifndef MOTION_CONSTANTS_H
#define MOTION_CONSTANTS_H

#include <common/constants.h>


// Motor Speeds

#define MOTOR_SPEED_FOLLOWING   8000    // Base motor speed whilst tape following
#define MOTOR_SPEED_ROTATION    8000    // Base motor speed whilst rotating

// Delays

#define MOTOR_ADJUSTMENT_DELAY_TAPE_FOLLOWING_MS    5       // delay between motor PID adjustment (ms)
#define MOTOR_ADJUSTMENT_DELAY_ROTATING_MS          5       // delay between checks for tape during rotation (ms)
#define ROTATE_INTO_TAPE_FOLLOW_DELAY               300     // delay between rotation task end and tape following beginning (ms)
#define ROTATE_INITIAL_DELAY                        1000    // delay between rotation task beginning and checking for tape (ms)               
#define POLL_SENSOR_DELAY_MS                        5       // Tape reflectance sensor polling delay (ms)

// Sensor Thresholds

#define THRESHOLD_SENSOR_SINGLE     1500 //defines the minimum  reflectance sensor reading to determine a sensor is seeing tape

// Pin Definitions
#define MOTOR_BACK_RIGHT_FORWARD    32
#define MOTOR_BACK_RIGHT_REVERSE    26
#define MOTOR_BACK_LEFT_FORWARD     7
#define MOTOR_BACK_LEFT_REVERSE     5
#define MOTOR_FRONT_LEFT_FORWARD    10
#define MOTOR_FRONT_LEFT_REVERSE    9 
#define MOTOR_FRONT_RIGHT_FORWARD   4
#define MOTOR_FRONT_RIGHT_REVERSE   2

#define STEPPER_MOTOR_STEP  7
#define STEPPER_MOTOR_DIR   14

#define LEFT_TAPE_SENSOR    13
#define RIGHT_TAPE_SENSOR   15

// Motion Task Priorities
#define PRIORITY_REFLECTANCE_POLLING    3
#define PRIORITY_FOLLOW_TAPE            1
#define PRIORITY_ROTATE                 2

// Motion PID
#define kp 1.1
#define kd 1.8


#endif // MOTION_CONSTANTS_H