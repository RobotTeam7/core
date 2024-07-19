#include <common/robot_motor.h>
#include <common/utils.h>


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
    robotMotor->currentState = NO_DRIVE;
    robotMotor->currentDrive = 0;

    log_status("Instantiated motor!");
    
    return robotMotor;
}

void motor_set_drive(RobotMotor_t* robotMotor, uint16_t driveValue, uint8_t direction) {
    int oldPin = direction != FORWARD_DRIVE ? robotMotor->boundForwardPin : robotMotor->boundReversePin;
    int newPin = direction == FORWARD_DRIVE ? robotMotor->boundForwardPin : robotMotor->boundReversePin;

    if (direction != robotMotor->currentState) {
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
    motor_stop(robot_motors->motorBL);
    motor_stop(robot_motors->motorFL);
    motor_stop(robot_motors->motorFR);
    motor_stop(robot_motors->motorBR);
}

void drive_robot_motors(RobotMotorData_t* robot_motors, uint16_t drive_value, uint8_t direction) {

}

void rotate_robot(RobotMotorData_t* robot_motors, uint16_t drive_value, uint8_t direction) {
    
}

