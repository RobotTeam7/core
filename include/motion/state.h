#ifndef MOTION_STATE_H
#define MOTION_STATE_H

#include <Arduino.h>


// Coordinate System Orientation: serving station is down (enemy gate is down). 

typedef enum { GOTO_STATION, SPIN, IDLE, DOCK_AT_STATION, RETURN_TO_TAPE, WALL_SLAM_TO } ActionType_t;

typedef enum { DRIVE, ROTATE, STOP, TRANSLATE, HUG_WALL } DriveState_t;

typedef struct {
    // Memory
    int orientation;                    // Which direction is the robot facing -> 1: right, -1: left
    uint8_t last_station;               // Which station are we at (or was at before moving)
    int tape_displacement_direction;    // Which direction is back to the tape -> 1: up, -1: down

    // Control
    uint8_t desired_station;
    int direction;                      // Which direction should the robot drive in -> 1: forward, -1: reverse
    int y_direction;                    // Which direction to translate -> 1: up, 0: on tape, -1: down                
    int helicity;                       // Which direction should the robot rotate -> 1: counterclockwise, -1: clockwise
    int yaw;                            // pid-updated drive state
    uint16_t drive_speed;               // base speed in which motors should be engaged
    DriveState_t drive_state;
    ActionType_t current_action;
} State_t;

extern State_t state;


#endif // MOTION_STATE_H