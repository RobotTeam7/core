#include <motion/motion.h>
#include <motion/constants.h>


void stop_all_motors(RobotMotorData_t* robot_motors) {
    motor_stop(robot_motors->motorBL);
    motor_stop(robot_motors->motorFL);
    motor_stop(robot_motors->motorFR);
    motor_stop(robot_motors->motorBR);
}

void set_robot_drive(RobotMotorData_t* robot_motors, int16_t drive_value) {
    motor_set_drive(robot_motors->motorFL, (drive_value) - state.yaw);
    motor_set_drive(robot_motors->motorBL, (drive_value) - state.yaw);
    motor_set_drive(robot_motors->motorFR, (drive_value) + state.yaw);
    motor_set_drive(robot_motors->motorBR, (drive_value) + state.yaw);
}

void rotate_robot(RobotMotorData_t* robot_motors, uint16_t drive_value) {
    motor_set_drive(robot_motors->motorFR, -drive_value);
    motor_set_drive(robot_motors->motorBR, -drive_value);
    motor_set_drive(robot_motors->motorFL, drive_value);
    motor_set_drive(robot_motors->motorBL, drive_value);
   
}

void translate_robot(RobotMotorData_t* robot_motors, int16_t drive_value) {
    motor_set_drive(robot_motors->motorFL, drive_value);
    motor_set_drive(robot_motors->motorFR, -drive_value * 0.9);
    motor_set_drive(robot_motors->motorBL, -drive_value);
    motor_set_drive(robot_motors->motorBR, drive_value);
}
