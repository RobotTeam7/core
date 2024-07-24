#include <common/servo_motor.h>


ServoMotor_t* instantiate_servo_motor(uint8_t boundControlPin, float max_duty_cycle, float min_duty_cycle) {
    ServoMotor_t* servoMotor = (ServoMotor_t*)malloc(sizeof(ServoMotor_t));
    if (servoMotor == NULL) {
        log_error("Failed to allocate memory for servo motor!");
        return NULL;
    }

    uint16_t closed_position = max_duty_cycle * UINT16_MAX;
    servoMotor->position = closed_position;
    servoMotor->boundControlPin = boundControlPin;
    servoMotor->max_duty_cycle = max_duty_cycle;
    servoMotor->min_duty_cycle = min_duty_cycle;

    bind_pwm(servoMotor->boundControlPin, 50);
    set_pwm(servoMotor->boundControlPin, servoMotor->position);    

    log_status("Created servo motor!");

    return servoMotor;
}

// sets the position of the claw based on a percentage value
// 0 is closed
// 1 is open
void set_servo_position_percentage(ServoMotor_t* servoMotor, float percentange) {
    if (percentange < 0 || percentange > 1) {
        log_error("Invalid percentage. Must be between 0 and 1!");
        return;
    }

    int range = (servoMotor->max_duty_cycle - servoMotor->min_duty_cycle) * UINT16_MAX;
    int min_power = servoMotor->min_duty_cycle * UINT16_MAX;
    int power = min_power + range * percentange;

    set_servo_position(servoMotor, power);
}

void set_servo_position(ServoMotor_t* servoMotor, uint16_t newPosition) {
    servoMotor->position = newPosition;
    set_pwm(servoMotor->boundControlPin, servoMotor->position);
}
