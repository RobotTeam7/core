#include <motion/state.h>

State_t state = {
    .orientation = 1,
    .last_station = 0,
    .tape_displacement_direction = 0,
    .desired_station = 0,
    .direction = 1,
    .y_direction = -1,
    .helicity = 1,
    .yaw = 0,
    .drive_speed = 12000,
    .drive_state = TRANSLATE,
    .current_action = IDLE,
};
