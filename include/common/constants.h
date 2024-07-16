#ifndef UTILS_CONSTANTS_H
#define UTILS_CONSTANTS_H

// Verbosity Levels
#define NO_PRINTING 0 
#define ERRORS_ONLY 1
#define STATUS_MESSAGES 2
#define MOST_VERBOSE 3 
#define EVERYTHING 4

#ifndef VERBOSITY_LEVEL     // Manual debug level

#define VERBOSITY_LEVEL EVERYTHING

#define SERVO_MAX_DUTY_CYCLE 0.06
#define SERVO_MIN_DUTY_CYCLE 0.02

#define THRESHOLD 1000

#define REFLECTANCE_SENSOR_BUFFER_SIZE 1


#endif

#endif