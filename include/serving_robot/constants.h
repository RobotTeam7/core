#ifndef SERVING_ROBOT_CONSTANTS_H
#define SERVING_ROBOT_CONSTANTS_H

// Pin Definitions
#define SERVO_VERTICAL_PIN              4
#define SERVO_VERTICAL_UP               0.028
#define SERVO_VERTICAL_DOWN             0.1

#define RACK_FORWARD_PIN                7
#define RACK_REVERSE_PIN                8

#define CLAW_ACTUATE_PIN_1              32
#define CLAW_ACTUATE_PIN_2              33

#define SERVO_CLAW_PIN                  13
#define SERVO_CLAW_OPEN                 0.07
#define SERVO_CLAW_CLOSED               0.05

#define SERVO_DRAW_BRIDGE_PIN           22
#define SERVO_DRAW_BRIDGE_DOWN          0.06
#define SERVO_DRAW_BRIDGE_UP            0.10

#define SERVO_PLATE_PIN                 19
#define SERVO_PLATE_OPEN                0.055
#define SERVO_PLATE_CLOSED              0.032

#define SWITCH_RACK_CLAWSIDE            10
#define SWITCH_RACK_PLATESIDE           9

typedef enum {
    VERTICAL_DOWN = 100,
    CLAW_CLOSED_FULL = 20,
    VERTICAL_UP = 0,
    CLAW_OPEN = 100,

} ServoPositionsPercentage_t;

#endif // SERVING_ROBOT_CONSTANTS_H