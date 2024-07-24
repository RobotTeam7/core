#ifndef STEPPER_MOTOR_H
#define STEPPER_MOTOR_H

#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

#include <common/pwm.h>
#include <common/utils.h>


/**
 * \brief This struct represents a robot's stepper motor.
 * 
 * \details Use this class to bind forward and reverse pins, and control the motor abstractly through simple commands, from single steps to multiple steps.
 */
typedef struct {
    int position;
    uint8_t stepPin;
    uint8_t directionPin;
    uint8_t sleep_pin;
    SemaphoreHandle_t xMutex; // This mutex is used to ensure that multiple stepper motor commands run sequentially (never at the same time!).
    int speed;
} StepperMotor_t;


/**
 * @brief This buffer contains the data necessary to perform a stepper motor actuation command.
 */
typedef struct {
    int numSteps;
    int direction;
    StepperMotor_t* stepperMotor;
} StepperMotorCommandBuffer_t;

/**
 * @brief Create a new stepper motor.
 * 
 * @param stepPin Pin delivering the step pulse to the stepper motor.
 * @param dirPin Pin indicating the step direction to the stepper motor.
 * @param position Initial stepper motor position
 * @param speed number of steps per second (in Hz)
 * 
 * @returns Heap-allocated (created with `malloc()`) pointer. Remember to free!
 */
StepperMotor_t* instantiate_stepper_motor(uint8_t stepPin, uint8_t dirPin, uint8_t sleep_pin, int position, int speed);

/**
 * @brief Actuate the stepper motor to make `numSteps` steps in `direction`, at a given frequency.
 * 
 * @details This command is thread-safe. Multiple commands may be issued to the same stepper motor and they will execute in the order they will sent without overlap: it will not cause `position` to become incorrect. 
 * 
 * @param direction `UP` or `DOWN`
 * @param numSteps integer number of steps to make
 */
void actuate_stepper_motor(StepperMotor_t* stepperMotor, int direction, int numSteps);


#endif // STEPPER_MOTOR_H