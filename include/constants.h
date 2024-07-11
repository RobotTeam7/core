
// ______________ SPEEDS ______________
#define MOTOR_SPEED_HIGH 16000
#define MOTOR_SPEED_LOW 14000
#define MOTOR_SPEED_ROTATION 13000



// ______________ DELAYS ______________

// this delay defines how often motor processes check buffer values for cetain tasks
#define MOTOR_ADJUSTMENT_DELAY_TAPE_FOLLOWING_MS 2
#define MOTOR_ADJUSTMENT_DELAY_ROTATING_MS 2

#define POLL_RATE_REFLECTANCE_MS 1



// ______________ THRESHOLDS ______________
// this defines the minimum difference between two reflectance sensor readings to determine that one is seeing tape
// and the other is not
#define THRESHOLD_SENSOR_DIFFERENCE 300

// this defines the minimum  reflectance sensor reading to determine a sensor is seeing tape
#define THRESHOLD_SENSOR_SINGLE 500
