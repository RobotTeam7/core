#include <motion/state.h>


State_t state;

void init_state() {
    state = {
        .orientation = -1,
        .last_side_station = 0,
        .y_direction = 0,
        .memory_lock = xSemaphoreCreateMutex(),
        .helicity = 1,
        .direction = 1,
        .drive_speed = 0,
        .drive_state = STOP,
        .drive_lock = xSemaphoreCreateMutex(),
        .yaw = 0,
        .speed_modifier = 1.0,
        .pirouette_angle = 0,
        .desired_side_station = 0,
        .current_action = IDLE,
        .command_lock = xSemaphoreCreateMutex(),
    };

    if (state.memory_lock == NULL || state.drive_lock == NULL || state.command_lock == NULL) {
        log_error("Failed to create state mutexes!");
        return;
    }
}

void update_localization(int orientation, int8_t last_side_station, int y_direction) {
    state.orientation = orientation;
    state.last_side_station = last_side_station;
    state.y_direction = y_direction;
}

void update_drive(int direction, int16_t drive_speed, DriveState_t drive_state) {
    state.direction = direction;
    state.drive_speed = drive_speed;
    state.drive_state = drive_state;
}

void update_control(int yaw, double speed_modifier, int16_t pirouette_angle) {
    state.yaw = yaw;
    state.speed_modifier = speed_modifier;
    state.pirouette_angle = pirouette_angle;
}

void update_command(int8_t desired_side_station, ActionType_t new_action) {
    state.desired_side_station = desired_side_station;
    state.current_action = new_action;
}