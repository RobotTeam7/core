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

void RobotMotor::set_drive(uint16_t driveValue, driveMode newState) {
    PinName oldPin = newState != forward ? boundForwardPin : boundReversePin;
    PinName newPin = newState == forward ? boundForwardPin : boundReversePin;

    if (newState != currentState) {
    pwm_start(oldPin, DRIVE_MOTOR_PWM_FREQUENCY, 0, TimerCompareFormat_t::RESOLUTION_16B_COMPARE_FORMAT);
    }

    pwm_start(newPin, DRIVE_MOTOR_PWM_FREQUENCY, driveValue, TimerCompareFormat_t::RESOLUTION_16B_COMPARE_FORMAT);

    currentState = newState;
    currentDrive = driveValue;
}

void RobotMotor::stop() {
    set_drive(0, forward);
}

driveMode RobotMotor::report_drive() {
    return currentState;
}
