#ifndef STEPPER_MOTOR_H
#define STEPPER_MOTOR_H

#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <common/pwm.h>
#include <common/utils.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"


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

/**
 * @brief Create a new stepper motor.
 * 
 * @param stepPin Pin delivering the step pulse to the stepper motor.
 * @param dirPin Pin indicating the step direction to the stepper motor.
 * @param position Initial stepper motor position
 * 
 * @returns Heap-allocated (created with `malloc()`) pointer. Remember to free!
 */
StepperMotor_t* instantiateStepperMotor(uint8_t stepPin, uint8_t dirPin, int position);

/**
 * @brief Actuate the stepper motor to make `numSteps` steps in `direction`, at a given frequency.
 * 
 * @param direction `UP` or `DOWN`
 * @param numSteps integer number of steps to make
 * @param motorFrequency frequency in Hz that the steps will be made in. Must not exceed MAX_STEPPER_FREQENCY.
 */
void actuateStepperMotor(StepperMotor_t* stepperMotor, int direction, int numSteps, uint16_t motorFrequency);


#endif // STEPPER_MOTOR_H