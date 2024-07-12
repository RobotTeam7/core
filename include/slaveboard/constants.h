#ifndef CONSTANTS_H
#define CONSTANTS_H


// ______________ SPEEDS ______________
#define MOTOR_SPEED_HIGH 8500
#define MOTOR_SPEED_LOW 5500
#define MOTOR_SPEED_ROTATION 8000



// ______________ DELAYS ______________

// this delay defines how often motor processes check buffer values for cetain tasks
#define MOTOR_ADJUSTMENT_DELAY_TAPE_FOLLOWING_MS 2
#define MOTOR_ADJUSTMENT_DELAY_ROTATING_MS 2

#define POLL_RATE_REFLECTANCE_MS 1



// ______________ THRESHOLDS ______________
// this defines the minimum difference between two reflectance sensor readings to determine that one is seeing tape
// and the other is not
#define THRESHOLD_SENSOR_DIFFERENCE 300

// this defines the minimum  reflectance sensor reading to determine a sensor is seeing tape
#define THRESHOLD_SENSOR_SINGLE 500

#define MOTOR_BACK_RIGHT_FORWARD PB1
#define MOTOR_BACK_RIGHT_REVERSE PB0
#define MOTOR_BACK_LEFT_FORWARD PA7
#define MOTOR_BACK_LEFT_REVERSE PA6
#define MOTOR_FRONT_LEFT_FORWARD PA1
#define MOTOR_FRONT_LEFT_REVERSE PA0
#define MOTOR_FRONT_RIGHT_FORWARD PA3
#define MOTOR_FRONT_RIGHT_REVERSE PA2
#define configCHECK_FOR_STACK_OVERFLOW 2

#define SPEED 12000
#define TIME_DELAY_MOTOR 500

#define PRIORITY_REFLECTANCE_POLLING 3
#define PRIORITY_FOLLOW_TAPE 1

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET -1    // This display does not have a reset pin accessible

#endif