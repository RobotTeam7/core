#include <motion/state.h>

State_t state = {
    .last_station = 0,
    .desired_station = 2,
    .orientation = 1,
    .helicity = 1,
    .current_action = GOTO_STATION,
};
