#ifndef MOTION_CONSTANTS_H
#define MOTION_CONSTANTS_H

#include <common/constants.h>


// Motor Speeds

#define MOTOR_SPEED_FOLLOWING   8000    // Base motor speed whilst tape following
#define MOTOR_SPEED_ROTATION    8000    // Base motor speed whilst rotating

// Delays

#define MOTOR_ADJUSTMENT_DELAY_TAPE_FOLLOWING_MS    1       // delay between motor PID adjustment (ms)
#define MOTOR_ADJUSTMENT_DELAY_ROTATING_MS          1       // delay between checks for tape during rotation (ms)
#define ROTATE_INTO_TAPE_FOLLOW_DELAY               300     // delay between rotation task end and tape following beginning (ms)
#define ROTATE_INITIAL_DELAY                        1000    // delay between rotation task beginning and checking for tape (ms)               
#define STATION_TRACKING_POLL_DELAY_MS              20      // delay between polls for if we see station tape
#define MOTOR_UPDATE_DELAY                          1       // delay between motor drive state updates 

// Sensor Thresholds

#define THRESHOLD_SENSOR_SINGLE     1500 //defines the minimum  reflectance sensor reading to determine a sensor is seeing tape
#define DEBOUNCE_TAPE_THRESHOLD     10 // Consecutive polls in which we need to see tape to affirm that we are on a station
 
// Pin Definitions
#define MOTOR_BACK_RIGHT_FORWARD    25
#define MOTOR_BACK_RIGHT_REVERSE    26
#define MOTOR_BACK_LEFT_FORWARD     13
#define MOTOR_BACK_LEFT_REVERSE     4
#define MOTOR_FRONT_LEFT_FORWARD    8
#define MOTOR_FRONT_LEFT_REVERSE    7
#define MOTOR_FRONT_RIGHT_FORWARD   10
#define MOTOR_FRONT_RIGHT_REVERSE   9

#define FRONT_TAPE_SENSOR_LEFT      34
#define FRONT_TAPE_SENSOR_RIGHT     35
#define BACK_TAPE_SENSOR_LEFT       38
#define BACK_TAPE_SENSOR_RIGHT      37

#define LEFT_WING_TAPE_SENSOR       32 // yellow cables right
#define RIGHT_WING_TAPE_SENSOR      33 // green cables left

#define RX_PIN                      21
#define TX_PIN                      22

// Stepper  
#define STEPPER_DIR                 15
#define STEPPER_STEP                12

// Motion Task Priorities
#define PRIORITY_DRIVE_UPDATE           3
#define PRIORITY_FOLLOW_TAPE            1
#define PRIORITY_ROTATE                 2
#define PRIORITY_STATION_TRACKING       1

// Motion PID
#define kp 1.1
#define kd 1.8


#endif // MOTION_CONSTANTS_H