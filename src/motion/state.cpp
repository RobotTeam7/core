#include <motion/state.h>

State_t state = {
    .orientation = 1,
    .last_station = 2,
    .desired_station = 0,
    .direction = 1,
    .y_direction = -1,
    .helicity = -1,
    .yaw = 0,
    .drive_speed = 0,
    .drive_state = STOP,
    .current_action = IDLE,
};
