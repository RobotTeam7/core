#ifndef COMMON_CONSTANTS_H
#define COMMON_CONSTANTS_H

// Verbosity Levels
#define NO_PRINTING         0 
#define ERRORS_ONLY         1
#define STATUS_MESSAGES     2
#define MOST_VERBOSE        3 
#define EVERYTHING          4

#ifndef VERBOSITY_LEVEL     // Manual debug level
    #define VERBOSITY_LEVEL EVERYTHING
#endif

#define PRINT_DRIVE_STATE       0

// Servo Motor Constants
#define SERVO_MAX_DUTY_CYCLE    0.06
#define SERVO_MIN_DUTY_CYCLE    0.02

// Thresholds
#define TAPE_SENSOR_AFFIRMATIVE_THRESHOLD 1000  // threshold for the sensor to report that it is seeing tape

// Buffer Sizes
#define REFLECTANCE_SENSOR_BUFFER_SIZE 1

// UART Constants
#define RX_PIN 21
#define TX_PIN 22

// PWM Constants
#define TIMER_RESOLUTION   16   // Indicate 16-bit resolution such that 0 – 65535 is valid
#define LEDC_PWM_FREQUENCY 50   // 50 Hz PWM pulse train

// Stepper Motor Constants
#define MAX_STEPPER_FREQENCY 100000
#define TEST_FREQUENCY 500
#define PRIORITY_STEPPER_TASK 1
#define UP HIGH
#define DOWN LOW

// ESP32 Constants
#define MAX_CHANNELS 16 /// Maximum number of ledc PWM channels that can be created for the ESP32

// Robot Motor Constants
#define DRIVE_MOTOR_PWM_FREQUENCY 50
#define NO_DRIVE 0
#define FORWARD_DRIVE 1
#define REVERSE_DRIVE 2


#endif // COMMON_CONSTANTS_H