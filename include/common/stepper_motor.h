#ifndef STEPPER_MOTOR_H
#define STEPPER_MOTOR_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <math.h>
#include <common/pwm.h>
#include <common/pin.h>
#include <FreeRTOS.h>
#include <task.h>


#define MAX_STEPPER_FREQENCY 100000
#define TEST_FREQUENCY 500
#define PRIORITY_STEPPER_TASK 1

#define UP HIGH
#define DOWN LOW

/**
 * \brief This class represents a robot's stepper motor.
 * 
 * \details Use this class to bind forward and reverse pins, and control the motor abstractly through simple commands, from single steps to multiple steps.
 */
class StepperMotor {
private:
    int position;
    uint8_t boundStepPin;
    uint8_t boundDirPin;

public:
    StepperMotor(uint8_t stepPin, uint8_t dirPin, int position);
    StepperMotor();

    void step(int direction, int numSteps);
    void single_step(int direction);  
};

struct StepperMotorCommandData_t {
    int numSteps;
    int frequency;
    uint8_t stepPin;
    uint8_t dirPin;
    int direction;
};

#endif