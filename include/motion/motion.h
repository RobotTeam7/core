#ifndef MOTION_H
#define MOTION_H


#include <common/robot_motor.h>
#include <common/utils.h>
#include <motion/state.h>

typedef struct {
    RobotMotor_t* motorFR;
    RobotMotor_t* motorFL;
    RobotMotor_t* motorBR;
    RobotMotor_t* motorBL;
} RobotMotorData_t;

/**
 * @brief Stop all motors. 
 */
void stop_all_motors(RobotMotorData_t* robot_motors);

/**
 * @brief Drive all motors with base speed `drive_value`. Individual motors will be set to a speed dependent on `state.pid_action`.
 */
void set_robot_drive(RobotMotorData_t* robot_motors, int16_t drive_value);

/**
 * @brief Rotate the robot with power `drive_value`.
 */
void rotate_robot(RobotMotorData_t* robot_motors, int16_t drive_value);

/**
 * @brief Order the robot to translate horizontally.
 * @param drive_value Positive value will indicate upwards, negative value will indicate downwards
 */
void translate_robot(RobotMotorData_t* robot_motors, int16_t drive_value);

void pirouette_robot(RobotMotorData_t* robot_motors, int16_t drive_value_rotate, int16_t drive_value_translate);



#endif // MOTION_H