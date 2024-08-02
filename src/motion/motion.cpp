#include <motion/motion.h>
#include <motion/constants.h>


void stop_all_motors(RobotMotorData_t* robot_motors) {
    motor_stop(robot_motors->motorBL);
    motor_stop(robot_motors->motorFL);
    motor_stop(robot_motors->motorFR);
    motor_stop(robot_motors->motorBR);
}

void set_robot_drive(RobotMotorData_t* robot_motors, int16_t drive_value) {
    motor_set_drive(robot_motors->motorFL, (int16_t)((drive_value - state.yaw) * state.speed_modifier));
    motor_set_drive(robot_motors->motorBL, (int16_t)((drive_value - state.yaw) * state.speed_modifier));
    motor_set_drive(robot_motors->motorFR, (int16_t)((drive_value + state.yaw) * state.speed_modifier));
    motor_set_drive(robot_motors->motorBR, (int16_t)((drive_value + state.yaw) * state.speed_modifier));
}

void rotate_robot(RobotMotorData_t* robot_motors, int16_t drive_value) {
    motor_set_drive(robot_motors->motorFR, (int16_t)((-drive_value) * state.speed_modifier));
    motor_set_drive(robot_motors->motorBR, (int16_t)((-drive_value) * state.speed_modifier));
    motor_set_drive(robot_motors->motorFL, (int16_t)( (drive_value) * state.speed_modifier));
    motor_set_drive(robot_motors->motorBL, (int16_t)( (drive_value) * state.speed_modifier));
}

// positive drive value translates robot toward it's right side
void translate_robot(RobotMotorData_t* robot_motors, int16_t drive_value) {
    motor_set_drive(robot_motors->motorFL, (int16_t)((-drive_value) * state.speed_modifier));
    motor_set_drive(robot_motors->motorFR, (int16_t)( (drive_value) * state.speed_modifier));
    motor_set_drive(robot_motors->motorBL, (int16_t)( (drive_value) * state.speed_modifier));
    motor_set_drive(robot_motors->motorBR, (int16_t)((-drive_value) * state.speed_modifier));
}

void pirouette_robot(RobotMotorData_t* robot_motors, int16_t drive_value_rotate, int16_t drive_value_translate, int degrees) {
    // Convert theta from degrees to radians
    float theta = degrees * (M_PI / 180.0);

    // Calculate the x and y components of the desired movement vector
    float vx = drive_value_translate * cos(theta);
    float vy = drive_value_translate * sin(theta);

    // Calculate the motor speeds needed to achieve the desired movement vector
    float motorFL_speed_translation = vy - vx;
    float motorFR_speed_translation = vy + vx;
    float motorBL_speed_translation = vy + vx;
    float motorBR_speed_translation = vy - vx;

    // Combine rotation and translation
    motor_set_drive(robot_motors->motorFR, (int16_t)((motorFR_speed_translation - drive_value_rotate) * state.speed_modifier));
    motor_set_drive(robot_motors->motorBR, (int16_t)((motorBR_speed_translation - drive_value_rotate) * state.speed_modifier));
    motor_set_drive(robot_motors->motorFL, (int16_t)((motorFL_speed_translation + drive_value_rotate) * state.speed_modifier));
    motor_set_drive(robot_motors->motorBL, (int16_t)((motorBL_speed_translation + drive_value_rotate) * state.speed_modifier));
}
