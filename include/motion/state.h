#ifndef MOTION_STATE_H
#define MOTION_STATE_H

#include <common/utils.h>


// Coordinate System Orientation: serving station is down (enemy gate is down). 

typedef enum { GOTO_STATION, SPIN, IDLE, DOCK_AT_STATION, RETURN_TO_TAPE, WALL_SLAM_TO, PIROUETTE, SIDE_SWAP, STARTUP } ActionType_t;

typedef enum { DRIVE, ROTATE, STOP, TRANSLATE, ROTATE_AND_TRANSLATE } DriveState_t;

typedef struct {
    // Localization
    int orientation;                // Which direction is the robot facing -> 1: right, -1: left
    int8_t last_side_station;       // When hugging a counter, which station were we last at
    int y_direction;                // Which direction are we displaced from tape -> 1: up, 0: on tape, -1: down
    SemaphoreHandle_t memory_lock;  // Acquire before calling `update_localization`.                
    
    // Drive
    int helicity;                   // Which direction should the robot rotate -> 1: counterclockwise, -1: clockwise
    int direction;                  // Which direction should the robot drive in -> 1: forward, -1: reverse
    int16_t drive_speed;            // base speed in which motors should be engaged
    DriveState_t drive_state;
    SemaphoreHandle_t drive_lock;   // Acquire before calling `update_drive`. 

    // Control    
    int yaw;                        // pid-updated drive state
    double speed_modifier;          // multiplier to ALL drive speeds (should be a percentage, 0.0-1.0)
    int16_t pirouette_angle;
    SemaphoreHandle_t control_lock; // Acquire before calling `update_control`. 

    // Command
    int8_t desired_side_station;    // this value isn't a uint because sometimes we will send pirouette commands to return to the same side
    ActionType_t current_action;
    SemaphoreHandle_t command_lock; // Acquire before calling `update_command`.     
} State_t;

void init_state();
void update_localization(int orientation, int8_t last_side_station, int y_direction);
void update_drive(int direction, int16_t drive_speed, DriveState_t drive_state);
void update_control(int yaw, double speed_modifier, int16_t pirouette_angle);
void update_command(int8_t desired_side_station, ActionType_t new_action);

extern State_t state;


#endif // MOTION_STATE_H