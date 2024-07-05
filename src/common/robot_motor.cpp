#include <common/robot_motor.h>

RobotMotor::RobotMotor(int forwardPin, int reversePin) {
    pwm::bind_pwm(forwardPin);
    pwm::bind_pwm(reversePin);

    this->boundForwardPin = forwardPin;
    this->boundReversePin = reversePin;
    this->currentState = none;
    this->currentDrive = 0;
}

RobotMotor::RobotMotor() {
    this->currentState = none;
    this->currentDrive = 0;
}

void RobotMotor::set_drive(uint16_t driveValue, driveMode direction) {
    int oldPin = direction != forward ? boundForwardPin : boundReversePin;
    int newPin = direction == forward ? boundForwardPin : boundReversePin;

    if (direction != currentState) {
        pwm::set_pwm(oldPin, 0);
    }

    pwm::set_pwm(newPin, driveValue);

    currentState = direction;
    currentDrive = driveValue;
}

void RobotMotor::stop() {
    set_drive(0, none);
}

driveMode RobotMotor::report_drive() {
    return currentState;
}