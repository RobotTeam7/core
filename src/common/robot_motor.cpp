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
    int direction = FORWARD_DRIVE;

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
    motor_set_drive(robotMotor, 0);
}
