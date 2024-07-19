#ifndef MOTION_STATE_H
#define MOTION_STATE_H

#include <Arduino.h>


// Coordinate System: serving station is south

typedef enum { GOTO_STATION, ROTATE, IDLE } ActionType_t;

typedef struct {
    uint8_t last_station;
    uint8_t desired_station;
    int orientation;                // 1: right, -1, left
    int helicity;                   // 1: counterclockwise, -1: clockwise
    ActionType_t current_action;
} State_t;

extern State_t state;


#endif // MOTION_STATE_H