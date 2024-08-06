#include <motion/state.h>

State_t state = {
    .orientation = -1,
    .pirouette_angle = 0,
    .last_side_station = 0,
    .y_direction = 0,
    .helicity = 1,
    .direction = 1,
    .yaw = 0,
    .drive_speed = 0,
    .speed_modifier = 1.0,
    .desired_side_station = 0,
    .drive_state = STOP,
    .current_action = IDLE,
};