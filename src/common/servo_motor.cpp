#include <common/servo_motor.h>
#include <common/constants.h>

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

// sets the position of the claw based on a percentage value
// 0 is closed
// 1 is open
void ServoMotor::set_position_percentage(float percentange) {
    if(percentange < 0 || percentange > 1) {
        Serial.println("invalid percentage, must be between 0 and 1");
        return;
    }

    int range = (SERVO_MAX_DUTY_CYCLE - SERVO_MIN_DUTY_CYCLE) * PWM_MAX_POWER;
    int min_power = SERVO_MIN_DUTY_CYCLE * PWM_MAX_POWER;
    int power = min_power + range * percentange;
    this->set_position(power);
}

uint16_t ServoMotor::get_position() {
    return this->position;
}