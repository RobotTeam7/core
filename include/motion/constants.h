#ifndef MOTION_CONSTANTS_H
#define MOTION_CONSTANTS_H

#include <common/constants.h>


// ______________ SPEEDS ______________
#define MOTOR_SPEED_FOLLOWING 8000
#define MOTOR_SPEED_ROTATION 8000



// ______________ DELAYS ______________

// this delay defines how often motor processes check buffer values for cetain tasks
#define MOTOR_ADJUSTMENT_DELAY_TAPE_FOLLOWING_MS 5
#define MOTOR_ADJUSTMENT_DELAY_ROTATING_MS 5

#define POLL_SENSOR_DELAY_MS 1

#define ROTATE_INTO_TAPE_FOLLOW_DELAY 300

#define ROTATE_INITIAL_DELAY 1000



// ______________ THRESHOLDS ______________

// this defines the minimum  reflectance sensor reading to determine a sensor is seeing tape
// black housing 1000
#define THRESHOLD_SENSOR_SINGLE 1500

#define MOTOR_BACK_RIGHT_FORWARD 32
#define MOTOR_BACK_RIGHT_REVERSE 26
#define MOTOR_BACK_LEFT_FORWARD 7
#define MOTOR_BACK_LEFT_REVERSE 5
#define MOTOR_FRONT_LEFT_FORWARD 10
#define MOTOR_FRONT_LEFT_REVERSE 9 
#define MOTOR_FRONT_RIGHT_FORWARD 4
#define MOTOR_FRONT_RIGHT_REVERSE 2

#define STEPPER_MOTOR_STEP 7
#define STEPPER_MOTOR_DIR 14

#define REFLECTANCE_ONE 13
#define REFLECTANCE_TWO 15

#define SPEED 12000
#define TIME_DELAY_MOTOR 500

#define PRIORITY_REFLECTANCE_POLLING 3
#define PRIORITY_FOLLOW_TAPE 1
#define PRIORITY_ROTATE 2

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET -1    // This display does not have a reset pin accessible

#endif