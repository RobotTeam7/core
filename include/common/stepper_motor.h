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
#include <common/utils.h>


#define MAX_STEPPER_FREQENCY 100000
#define TEST_FREQUENCY 500
#define PRIORITY_STEPPER_TASK 1

#define UP HIGH
#define DOWN LOW


/**
 * \brief This struct represents a robot's stepper motor.
 * 
 * \details Use this class to bind forward and reverse pins, and control the motor abstractly through simple commands, from single steps to multiple steps.
 * NOTE: Multiple stepper commands could cause issues with  `position`, which is not thread safe. 
 */
typedef struct {
    int position;
    uint8_t stepPin;
    uint8_t directionPin;
} StepperMotor_t;

typedef struct {
    int numSteps;
    int frequency;
    int direction;
    StepperMotor_t* stepperMotor;
} StepperMotorCommandBuffer_t;

StepperMotor_t* instantiateStepperMotor(uint8_t stepPin, uint8_t dirPin, int position);
void actuateStepperMotor(StepperMotor_t* stepperMotor, int direction, int numSteps, uint16_t motorFrequency);


#endif