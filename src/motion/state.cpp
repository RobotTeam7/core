#include <motion/state.h>

State_t state = {
    .orientation = 1,
    .last_station = 1,
    .last_side_station = 0,
    .pirouette_angle = 0,
    
    .desired_station = 0,
    .desired_side_station = 0,
    .direction = 1,
    .y_direction = 0,
    .helicity = 1,
    .yaw = 0,
    .drive_speed = 0,
    .drive_state = STOP,
    .current_action = IDLE,
};