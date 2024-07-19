#include <common/robot_motor.h>


RobotMotor_t* instantiate_robot_motor(uint8_t forwardPin, uint8_t reversePin) {
    RobotMotor_t* robotMotor = (RobotMotor_t*)malloc(sizeof(RobotMotor_t));
    if (robotMotor == NULL) {
        log_error("Failed to allocate memory for robot motor!");
        return NULL;
    }

    bind_pwm(forwardPin);
    bind_pwm(reversePin);
    
    robotMotor->boundForwardPin = forwardPin;
    robotMotor->boundReversePin = reversePin;
    robotMotor->currentState = FORWARD_DRIVE;
    robotMotor->currentDrive = 0;

    log_status("Instantiated motor!");
    
    return robotMotor;
}

int get_output_pin(RobotMotor_t* robotMotor, int robot_state) {
    return robot_state == FORWARD_DRIVE ? robotMotor->boundForwardPin : robotMotor->boundReversePin;
}

void motor_set_drive(RobotMotor_t* robotMotor, int16_t driveValue) {
    int direction = driveValue > 0 ? FORWARD_DRIVE : REVERSE_DRIVE;

    int newPin = get_output_pin(robotMotor, direction);
    int oldPin = get_output_pin(robotMotor, robotMotor->currentState);

    if (newPin != oldPin) {
        set_pwm(oldPin, 0);
    }

    set_pwm(newPin, driveValue);

    robotMotor->currentState = direction;
    robotMotor->currentDrive = driveValue;
}

void motor_stop(RobotMotor_t* robotMotor) {
    motor_set_drive(robotMotor, 0, FORWARD_DRIVE);
}

void stop_robot_motors(RobotMotorData_t* robot_motors) {
    log_message("Stopping all motors...");

    motor_stop(robot_motors->motorBL);
    motor_stop(robot_motors->motorFL);
    motor_stop(robot_motors->motorFR);
    motor_stop(robot_motors->motorBR);
}

void drive_robot_motors(RobotMotorData_t* robot_motors, uint16_t drive_value, uint8_t direction) {
    motor_set_drive(robot_motors->motorFR, drive_value, direction);
    motor_set_drive(robot_motors->motorBR, drive_value, direction);
    motor_set_drive(robot_motors->motorFL, drive_value, direction);
    motor_set_drive(robot_motors->motorBL, drive_value, direction);
}

void rotate_robot(RobotMotorData_t* robot_motors, uint16_t drive_value) {
    motor_set_drive(robotMotors->motorFR, -MOTOR_SPEED_ROTATION * state->helicity);
    motor_set_drive(robotMotors->motorBR, -MOTOR_SPEED_ROTATION);
    motor_set_drive(robotMotors->motorFL, MOTOR_SPEED_ROTATION);
    motor_set_drive(robotMotors->motorBL, MOTOR_SPEED_ROTATION);
}

