#ifndef MOTION_STATE_H
#define MOTION_STATE_H

#include <Arduino.h>


// Coordinate System Orientation: serving station is down (enemy gate is down). 

typedef enum { GOTO_STATION, SPIN, IDLE, DOCK_AT_STATION, RETURN_TO_TAPE, WALL_SLAM_TO, PIROUETTE, SIDE_SWAP, STARTUP } ActionType_t;

typedef enum { DRIVE, ROTATE, STOP, TRANSLATE, ROTATE_AND_TRANSLATE } DriveState_t;

typedef struct {
    // Localization
    int orientation;                // Which direction is the robot facing -> 1: right, -1: left
    int16_t pirouette_angle;
    int8_t last_side_station;       // When hugging a counter, which station were we last at
    int y_direction;                // Which direction are we displaced from tape -> 1: up, 0: on tape, -1: down                
    
    // Drive
    int helicity;                   // Which direction should the robot rotate -> 1: counterclockwise, -1: clockwise
    int direction;                  // Which direction should the robot drive in -> 1: forward, -1: reverse
    int yaw;                        // pid-updated drive state
    int16_t drive_speed;            // base speed in which motors should be engaged
    float speed_modifier;           // multiplier to ALL drive speeds (should be a percentage, 0.0-1.0)

    // Command
    int8_t desired_side_station;    // this value isn't a uint because sometimes we will send pirouette commands to return to the same side
    DriveState_t drive_state;
    ActionType_t current_action;
} State_t;

extern State_t state;


#endif // MOTION_STATE_H