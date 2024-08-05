#ifndef MOTION_CONSTANTS_H
#define MOTION_CONSTANTS_H

#include <common/constants.h>

#if robot == 0 // Midnight Rambler

#define MOTOR_SPEED_PIROUETTE_ROTATION              (14000)    // Motor speed for the rotation moment vector component of a pirouette

#endif 

#if robot == 1 // Fiddler

#define MOTOR_SPEED_PIROUETTE_ROTATION              (12500)    // Motor speed for the rotation moment vector component of a pirouette

#endif

// Motor Speeds

#define MOTOR_SPEED_FOLLOWING                       (9000)    // Base motor speed whilst tape following
#define MOTOR_SPEED_ROTATION                        (9000)    // Base motor speed whilst rotating
#define MOTOR_SPEED_DOCKING                         (6000)    // Base speed for when we are trying to dock
#define MOTOR_SPEED_TRANSLATION                     (15000)   // Base speed for when we are trying to dock
#define MOTOR_SPEED_BREAKING                        (12000)   // Break speed of the motors
#define MOTOR_SPEED_WALL_SLAMMING                   (18000)    // Base motor sped whilst wall slamming
#define MOTOR_SPEED_WALL_SLAMMING_CRAWL             (7800)    // Base motor sped whilst approaching desired station and wall slamming
#define MOTOR_SPEED_PIROUETTE_TRANSLATION           (13000)   // Base speed for when we are trying to dock


// Delays

#define MOTOR_ADJUSTMENT_DELAY_TAPE_FOLLOWING_MS    (1)       // delay between motor PID adjustment (ms)
#define MOTOR_ADJUSTMENT_DELAY_ROTATING_MS          (1)       // delay between checks for tape during rotation (ms)
#define ROTATE_INTO_TAPE_FOLLOW_DELAY               (300)     // delay between rotation task end and tape following beginning (ms)
#define ROTATE_INITIAL_DELAY                        (1200)    // delay between rotation task beginning and checking for tape (ms)               
#define DELAY_STATION_TRACKING_INTITAL              (20)      // delay between polls for if we see station tape
#define MOTOR_UPDATE_DELAY                          (1)       // delay between motor drive state updates 
#define TAPE_TRACKING_INTITAL_DELAY                 (500)     // delay between when we start driving and when we start tracking tape (ms)
#define DELAY_BREAKING                              (0)       // delay between running motors in reverse and stopping for sharp robot breaking
#define DELAY_RETURN_TO_TAPE_POLL                   (1)       // delay between polls to see if we see tape while returning to tape
#define DELAY_STATION_TRACKING_POLL                 (5)       // delay between polls to see if we see station tape
#define DELAY_WALL_SLAMMING_STOP_POLL               (1)          
#define DELAY_WALL_SLAMMING_POLL                    (5)       // delay between polls to check if we are approaching tape during wall slamming
#define DELAY_HOMING_POLL                           (1)
#define DELAY_ROTATION_DURATION                     (3000)    // delay used to time when to stop rotating during a pirouette
#define DELAY_START_PIROUETTE                       (500)
#define DELAY_FINISH_PIROUETTE                      (2300)    // delay used to time when to stop translating after rotating during a pirouette
#define DELAY_TRANSLATE_TO_WALL                     (1100)
#define DELAY_TRANSLATE_SIDE_SWAP                   (1900)


// Sensor Thresholds

#define THRESHOLD_SENSOR_SINGLE                     (1500)    //defines the minimum  reflectance sensor reading to determine a sensor is seeing tape
#define TAPE_SENSOR_AFFIRMATIVE_MIDDLE_THRESHOLD    (2500)
#define DEBOUNCE_TAPE_THRESHOLD                     (10)      // Consecutive polls in which we need to see tape to affirm that we are on a station
 
// Pin Definitions
#define MOTOR_BACK_RIGHT_FORWARD    (25)
#define MOTOR_BACK_RIGHT_REVERSE    (26)
#define MOTOR_BACK_LEFT_FORWARD     (13)
#define MOTOR_BACK_LEFT_REVERSE     (4)
#define MOTOR_FRONT_LEFT_FORWARD    (8)
#define MOTOR_FRONT_LEFT_REVERSE    (7)
#define MOTOR_FRONT_RIGHT_FORWARD   (10)
#define MOTOR_FRONT_RIGHT_REVERSE   (9)

// #define FRONT_TAPE_SENSOR_LEFT      38
#define FRONT_TAPE_SENSOR_RIGHT     (38)
#define BACK_TAPE_SENSOR_LEFT       (37)
// #define BACK_TAPE_SENSOR_RIGHT      35
#define MIDDLE_TAPE_SENSOR          (35) // or 34

#define LEFT_WING_TAPE_SENSOR       (32) 
#define RIGHT_WING_TAPE_SENSOR      (33) 

#define IR_1                        (36) // VP
#define IR_2                        (39) // VN

#define SWITCH_COUNTER_1            (22)
#define SWITCH_COUNTER_2            (19)
#define SWITCH_COUNTER_3            (20) // front left
#define SWITCH_COUNTER_4            (21) // back left

#define RX_PIN                      (27)
#define TX_PIN                      (14)

// Motion Task Priorities
#define PRIORITY_DRIVE_UPDATE       (4)
#define PRIORITY_FOLLOW_TAPE        (2)
#define PRIORITY_ROTATE             (5)
#define PRIORITY_STATION_TRACKING   (3)
#define PRIORITY_RETURN_TO_TAPE     (5)
#define PRIORITY_FOLLOW_WALL        (5)
#define PRIORITY_HOMING             (5)

// Motion PID
#define kp                          (1.1)
#define kd                          (1.8)

#define EFFICIENCY_FRONT_RIGHT      (0.8)
#define EFFICIENCY_FRONT_LEFT       (0.8)
#define EFFICIENCY_BACK_RIGHT       (1)
#define EFFICIENCY_BACK_LEFT        (1)

#define YAW_WALL_SLAMMING           (1500) // the yaw (motor difference) introduced during wall slamming to ensure the robot stays on the wall.

#endif // MOTION_CONSTANTS_H