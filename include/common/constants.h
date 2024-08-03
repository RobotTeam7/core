#ifndef COMMON_CONSTANTS_H
#define COMMON_CONSTANTS_H

// Verbosity Levels
#define NO_PRINTING         0 
#define ERRORS_ONLY         1
#define STATUS_MESSAGES     2
#define MOST_VERBOSE        3 
#define EVERYTHING          4

#ifndef VERBOSITY_LEVEL     // Manual debug level
    #define VERBOSITY_LEVEL     EVERYTHING
#endif

// Thresholds
#define TAPE_SENSOR_AFFIRMATIVE_THRESHOLD   1000  // threshold for the sensor to report that it is seeing tape

// Buffer Sizes
#define REFLECTANCE_SENSOR_BUFFER_SIZE      1

// PWM Constants
#define TIMER_RESOLUTION    (ledc_timer_bit_t)16   // Indicate 16-bit resolution such that 0 – 65535 is valid
#define LEDC_PWM_FREQUENCY  (uint32_t)50           // 50 Hz PWM pulse train
#define MOTOR_TIMER_0       (ledc_timer_t)0
#define MOTOR_TIMER_1       (ledc_timer_t)1
#define STEPPER_TIMER       (ledc_timer_t)2

// Stepper Motor Constants
#define MAX_STEPPER_FREQENCY    100000
#define TEST_FREQUENCY          500
#define PRIORITY_STEPPER_TASK   5
#define UP                      1
#define DOWN                    -1

// ESP32 Constants
#define MAX_CHANNELS 16 /// Maximum number of ledc PWM channels that can be created for the ESP32

// UART ID Table
#define UART_CMD_BEGIN      0x00
#define UART_CMD_END        0x3f
#define UART_ACK_BEGIN      0x40
#define UART_ACK_END        0x4f
#define UART_NACK_BEGIN     0x50
#define UART_NACK_END       0x5f


#endif // COMMON_CONSTANTS_H