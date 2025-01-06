#ifndef ROBOT_HAL_H
#define ROBOT_HAL_H

#include <Arduino.h>
#include <stdint.h>
#include <math.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h> 
#include <freertos/semphr.h>

#include <common/constants.h>
#include <common/pwm.h>
#include <common/utils.h>



//////////////////////////
// < --- ROBOT MOTOR --- > 
//////////////////////////

#define DRIVE_MOTOR_PWM_FREQUENCY   (50)  // Motor PWM signal frequency in Hz

/**
 * @brief Enum discretizing the options for timers to control robot motots. A timer may support only up to four motors!
 */
typedef enum {
    timer_1 = MOTOR_TIMER_0, timer_2 = MOTOR_TIMER_1
} MotorTimers_t;

/**
 * @brief Enum discretizing the different directions that a motor can drive in
 */
typedef enum {
    FORWARD_DRIVE = 1, REVERSE_DRIVE = -1
} MotorDirection_t;

/**
 * @brief This struct encapsulates the data of a robot's motor.
 * 
 * @details Use this class to bind forward and reverse pins, and control the motor abstractly through simple commands.
 */
typedef struct {
    uint8_t forward_pin;             // Pin responsible for delivering a "drive forward" pulse train
    uint8_t reverse_pin;             // Pin responsible for delivering a "drive reverse" pulse train
    MotorDirection_t current_state;  // Current drive direction of the motor 
    uint16_t current_drive;          // Current motor power
} RobotMotor_t;

/**
 * @brief Instantiate a motor, binding `forward_pin` and `reverse_pin` to it.
 * 
 * @param timer the timer which will be used to generate the signal which controls this motor. 
 * 
 * @returns Heap-allocated (created with `malloc()`) pointer. Remember to free!
 */
RobotMotor_t* instantiate_robot_motor(uint8_t forward_pin, uint8_t reverse_pin, MotorTimers_t timer);

/**
 * @brief Set the drive state of this motor.
 * 
 * @param drive_value Drive power, where 0 is stopped and 65535 is full power. Negative value interpreted as reverse direction.
 */
void motor_set_drive(RobotMotor_t* robot_motor, int16_t drive_value);

/**
 * @brief Stop the motor.
 */
void motor_stop(RobotMotor_t* robot_motor); 



///////////////////////////
// < --- LIMIT SWITCH --- > 
///////////////////////////

/**
 * @brief This struct contains the data representing a limit switch. 
 */
typedef struct
{
    uint8_t interrupt_pin;  // The pin number that will be registered as an interrupt
} LimitSwitch_t;

typedef void (*ISR_Function)(); // Signature of a function which can act as a limit switch ISR

/**
 * @brief Instantiate a limit switch, registering an interrupt on `interrupt_pin` to activate `isr_function`.
 * 
 * @returns Heap-allocated (created with `malloc()`) pointer. Remember to free!
 */
LimitSwitch_t* instantiate_limit_switch(uint8_t interrupt_pin, ISR_Function isr_function);



//////////////////////////////
// < --- RACK AND PINION --- > 
//////////////////////////////

#define RACK_AND_PINION_SPEED 18000

extern RobotMotor_t* rack_and_pinion;
typedef enum {
    CLAW_FORWARDS = 1, CENTERED = 0, FORKLIFT_FORWARDS = -1
} RackAndPinionPosition_t;

/**
 * @brief Initialize the rack and pinion.
 */
void init_rack_and_pinion(uint8_t fowards_pin, uint8_t backwards_pin, RackAndPinionPosition_t initial_position, uint8_t claw_limit_switch_pin, uint8_t forklift_limit_switch_pin);

void actuate_claw_forwards();

void actuate_forklift_forwards();

void set_rack_zero();



//////////////////////////
// < --- TAPE SENSOR --- > 
//////////////////////////

#define TAPE_SENSOR_AFFIRMATIVE_THRESHOLD   1000  // Threshold for the sensor to report that it is seeing tape

/**
 * @brief This struct encapsulates the data contained by an analog tape sensor.
 */
typedef struct {
    uint8_t pin;
    uint16_t value;
} TapeSensor_t;

/**
 * @brief Instantiate a new analog tape sensor.
 * 
 * @param leftSensorPin Pin connected to the left tape sensor's output
 * @param rightSensorPin Pin connected to the right tape sensor's output
 * 
 * @returns Heap-allocated (created with `malloc()`) pointer. Remember to free!
 */
TapeSensor_t* instantiate_tape_sensor(uint8_t pin);

/**
 * @brief Perform a read of `tape_sensor`, updating its internal data.
 */
void read_tape_sensor(TapeSensor_t* tape_sensor);



//////////////////////////
// < --- SERVO MOTOR --- > 
//////////////////////////

#define SERVO_MOTOR_CONTROL_FREQUENCY (50)  // Servo motor control PWM signal frequency in Hz

/**
 * @brief This struct encapsulates the data for a servo motor 
 * 
 * @details Use this struct and its methods to control a servo motor connected to this robot
 */
typedef struct {
    uint8_t control_pin;     // Pin which will be used to control this servo motor
    uint16_t position;       // Current position, , of the servo motor
    double max_duty_cycle;   // Maximum allowed duty cycle of the control signal
    double min_duty_cycle;   // Minimum allowed duty cycle of the control signal
} ServoMotor_t;

/**
 * @brief Create a new servo motor, binding `control_pin` to it with `position` as its initial position.
 * 
 * @param controlPin The pin that will be used to control the servo motor's position.
 * @param position The servo motor's initial position.
 * 
 * @returns Heap-allocated (created with `malloc()`) pointer. Remember to free!
 */
ServoMotor_t* instantiate_servo_motor(uint8_t control_pin, double max_duty_cycle, double min_duty_cycle);

/**
 * @brief Set the position of this servo motor by passing a percentage where 0% is delivers a minimum duty cycle and 100% delivers a maximum duty cycle to the control pin.
 * 
 * @param percentage value within the range 0 – 100.
 */
int set_servo_position_percentage(ServoMotor_t* servoMotor, int percentage);



////////////////////////////
// < --- STEPPER MOTOR --- > 
////////////////////////////

#define TIMER_FREQUENCY 80000   // Default frequency of the timer, in Hz, that will be used to drive the stepper motor's PWM signal

/**
 * \brief This struct represents a robot's stepper motor.
 * 
 * \details Use this class to bind forward and reverse pins, and control the motor abstractly through simple commands, from single steps to multiple steps.
 */
typedef struct {
    volatile int position;      // Current position of the stepper motor, as an offset from its initial position
    int8_t direction;           // The direction that this stepper motor is actuating in
    uint8_t step_pin;           // The pin that will deliver the "STEP" pulse train to the driver 
    uint8_t direction_pin;      // The pin that will configure the driver's STEP direction
    uint8_t sleep_pin;          // The pin that will enable or disable the driver's SLEEP mode
    SemaphoreHandle_t xMutex;   // This mutex is used to ensure that multiple stepper motor commands run sequentially (never at the same time!).
    int speed;                  // The speed, in Hz, of this stepper motor
    volatile bool running;      // Flag indicating whether this stepper motor is currently actuating
} StepperMotor_t;

extern StepperMotor_t* stepper_motor; // This robot's stepper motor. 

/**
 * @brief This buffer contains the data necessary to perform a stepper motor actuation command.
 */
typedef struct {
    int new_position;  // The new position of the stepper motor, after actuation
    int direction;     // The direction of actuation
} StepperMotorCommandBuffer_t;

/**
 * @brief Configure this robot's stepper motor.
 * 
 * @param step_pin  Pin delivering the step pulse to the stepper motor.
 * @param direction_pin Pin indicating the step direction to the stepper motor.
 * @param sleep_pin  Pin controlling the driver's SLEEP mode (to minimize power consumption)
 * @param position Initial stepper motor position
 * @param speed number of steps per second (in Hz) that the stepper motor should perform 
 */
void init_stepper_motor(uint8_t step_pin, uint8_t direction_pin, uint8_t sleep_pin, int position, int speed);

/**
 * @brief Actuate the stepper motor goto the position `new_position`.
 * 
 * @details This command is thread-safe. Multiple commands may be issued to the same stepper motor and they will execute in the order they will sent without overlap: it will not cause `position` to become incorrect. 
 * 
 * @param direction `UP` or `DOWN`
 * @param new_position the new position of the platform, relative to the instantiation initial position
 */
void actuate_stepper_motor(int direction, int new_position);

/**
 * @brief Returns if this robot's stepper_motor is currently actuating (`true` if so, and `false` if not and is ready).
 */
bool is_running();


#endif