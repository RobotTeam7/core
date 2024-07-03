#include <robot_motor.h>

RobotMotor::RobotMotor(PinName forwardPin, PinName reversePin) {
    pinMode(forwardPin, OUTPUT);
    pinMode(reversePin, OUTPUT);

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
    PinName oldPin = direction != forward ? boundForwardPin : boundReversePin;
    PinName newPin = direction == forward ? boundForwardPin : boundReversePin;

    if (direction != currentState) {
    pwm_start(oldPin, DRIVE_MOTOR_PWM_FREQUENCY, 0, TimerCompareFormat_t::RESOLUTION_16B_COMPARE_FORMAT);
    }

    pwm_start(newPin, DRIVE_MOTOR_PWM_FREQUENCY, driveValue, TimerCompareFormat_t::RESOLUTION_16B_COMPARE_FORMAT);

    currentState = direction;
    currentDrive = driveValue;
}

void RobotMotor::stop() {
    set_drive(0, none);
}

driveMode RobotMotor::report_drive() {
    return currentState;
}
