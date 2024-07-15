#include <common/servo_motor.h>


ServoMotor::ServoMotor() {
    this->boundControlPin = NULL;
    this->position = NULL;
}

ServoMotor::ServoMotor(uint8_t controlPin, uint16_t position) {
    this->position = position;
    this->boundControlPin = controlPin;

    pwm::bind_pwm(this->boundControlPin);
    pwm::set_pwm(this->boundControlPin, this->position);
}

void ServoMotor::set_position(uint16_t newPosition) {
    this->position = newPosition;
    // Serial.println("Setting servo motor position to " + String(this->position));
    pwm::set_pwm(this->boundControlPin, this->position);
}

uint16_t ServoMotor::get_position() {
    return this->position;
}