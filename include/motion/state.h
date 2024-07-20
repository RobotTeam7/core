#ifndef MOTION_STATE_H
#define MOTION_STATE_H

#include <Arduino.h>


// Coordinate System Orientation: serving station is south (enemy gate is down). 

typedef enum { GOTO_STATION, SPIN, IDLE } ActionType_t;

typedef enum { DRIVE, ROTATE, STOP } DriveState_t;

typedef struct {
    uint8_t last_station;
    uint8_t desired_station;
    int orientation;                // Which direction is the robot facing -> 1: right, -1: left
    int direction;                  // Which direction should the robot drive in -> 1: forward, -1: reverse               
    int helicity;                   // Which direction should the robot rotate -> 1: counterclockwise, -1: clockwise
    int yaw;                        // pid-updated drive state
    uint16_t drive_speed;           // base speed in which motors should be engaged
    DriveState_t drive_state;
    ActionType_t current_action;
} State_t;

extern State_t state;


#endif // MOTION_STATE_H