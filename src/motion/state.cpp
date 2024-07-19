#include <motion/state.h>

State_t state = {
    .last_station = 0,
    .desired_station = 2,
    .orientation = 1,
    .direction = 1,
    .helicity = 1,
    .yaw = 0,
    .drive_speed = 0,
    .drive_state = STOP,
    .current_action = GOTO_STATION,
};
