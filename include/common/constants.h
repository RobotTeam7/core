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

// PWM Constants
#define TIMER_RESOLUTION        (ledc_timer_bit_t)16   // Indicate 16-bit resolution such that 0 – 65535 is valid
#define LEDC_PWM_FREQUENCY      (uint32_t)50           // Define standard frequency to generate a 50 Hz PWM signal

// Timer Indices 
#define MOTOR_TIMER_0           (ledc_timer_t)0
#define MOTOR_TIMER_1           (ledc_timer_t)1
#define STEPPER_TIMER           (ledc_timer_t)2

// Stepper Motor Constants
#define MAX_STEPPER_FREQENCY    (100000)    // Corresponds to timing limitations of A397 stepper driver chip
#define PRIORITY_STEPPER_TASK   (5)
#define UP                      (1)         
#define DOWN                    (-1)

// ESP32 Constants
#define MAX_CHANNELS            (16)        // Maximum number of ledc PWM channels that can be created for the ESP32

// Communciation ID Range Table    
#define UART_CMD_BEGIN          (0x00)      // Beginning of ID range for commands
#define UART_CMD_END            (0x3f)      // End of ID range for commands
#define UART_ACK_BEGIN          (0x40)      // Beginning of ID range for positive acknowledgements
#define UART_ACK_END            (0x4f)      // End of ID range for positive acknowledgements
#define UART_NACK_BEGIN         (0x50)      // Beginning of ID range for negative acknowledgements
#define UART_NACK_END           (0x5f)      // End of ID range for negative acknowledgements


#endif // COMMON_CONSTANTS_H