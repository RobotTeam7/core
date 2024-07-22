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

void motor_set_drive(RobotMotor_t* robot_motor, int16_t drive_value) {
    // Get the target direction by looking at the sign of drive_value
    int target_direction = drive_value > 0 ? FORWARD_DRIVE : REVERSE_DRIVE;
    
    // Determine 
    int old_pin = robot_motor->currentState == FORWARD_DRIVE ? robot_motor->boundForwardPin : robot_motor->boundReversePin;
    int new_pin = target_direction == FORWARD_DRIVE ? robot_motor->boundForwardPin : robot_motor->boundReversePin;

    if (new_pin != old_pin) {
        set_pwm(old_pin, 0);
    }

    int target_drive = drive_value * target_direction;

    set_pwm(new_pin, target_drive);

    robot_motor->currentState = target_direction;
    robot_motor->currentDrive = target_drive;
}

void motor_stop(RobotMotor_t* robotMotor) {
    motor_set_drive(robotMotor, 0);
}
