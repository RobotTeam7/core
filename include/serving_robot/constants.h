#ifndef SERVING_ROBOT_CONSTANTS_H
#define SERVING_ROBOT_CONSTANTS_H

// Pin Definitions
#define SERVO_VERTICAL_PIN              4
#define SERVO_VERTICAL_UP               0.0375
#define SERVO_VERTICAL_DOWN             0.1

#define RACK_FORWARD_PIN                8
#define RACK_REVERSE_PIN                7

#define CLAW_ACTUATE_PIN_1              32
#define CLAW_ACTUATE_PIN_2              33

#define SERVO_CLAW_PIN                  13
#define SERVO_CLAW_OPEN                 0.0635
#define SERVO_CLAW_CLOSED               0.05

// BUNS 0.0535
// LETTUCE 5.0
// TOMATO 5.2
// CHEESE 5.3
// PATTY 5.3


#define SERVO_DRAW_BRIDGE_PIN           22
#define SERVO_DRAW_BRIDGE_DOWN          0.06
#define SERVO_DRAW_BRIDGE_UP            0.10

#define SERVO_PLATE_PIN                 19
#define SERVO_PLATE_OPEN                0.055
#define SERVO_PLATE_CLOSED              0.032

#define SWITCH_RACK_CLAWSIDE            9
#define SWITCH_RACK_PLATESIDE           10

typedef enum {
    VERTICAL_UP = 0,
    VERTICAL_HEIGHT_3 = 10, // 3 = second highest position
    VERTICAL_HEIGHT_2 = 35, // 2 = halfway
    VERTICAL_HEIGHT_1 = 60, // 1 = second lowest position
    VERTICAL_DOWN = 100,

    CLAW_CLOSED_FULL = 0,
    CLAW_OPEN = 100,
    CLAW_CLOSED_BUN = 32,
    CLAW_CLOSED_LETTUCE = 0,
    CLAW_CLOSED_TOMATO = 18,
    CLAW_CLOSED_CHEESE = 27,
    CLAW_CLOSED_PATTY = 27,

    PLATE_CLOSED = 0,
    PLATE_OPEN = 100,

    DRAW_BRIDGE_UP = 100,
    DRAW_BRIDGE_DOWN = 0,
} ServoPositionsPercentage_t;

#endif // SERVING_ROBOT_CONSTANTS_H