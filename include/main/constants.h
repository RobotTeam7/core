#ifndef MAIN_CONSTANTS_H
#define MAIN_CONSTANTS_H


typedef enum {
    VERTICAL_UP = 0,
    VERTICAL_HEIGHT_3 = 10, // 3 = second highest position
    VERTICAL_HEIGHT_2 = 35, // 2 = halfway
    VERTICAL_HEIGHT_1 = 60, // 1 = second lowest position
    VERTICAL_DOWN = 100,

    CLAW_CLOSED_FULL = 0,
    CLAW_OPEN = 100,
    CLAW_CLOSED_BUN = 21,
    CLAW_CLOSED_LETTUCE = 0,
    CLAW_CLOSED_TOMATO = 7,
    CLAW_CLOSED_CHEESE = 16,
    CLAW_CLOSED_PATTY = 16,

    PLATE_CLOSED = 0,
    PLATE_OPEN = 100,

    DRAW_BRIDGE_UP = 100,
    DRAW_BRIDGE_DOWN = 0,
} ServoPositionsPercentage_t;

#define SERVO_ACTUATION_DELAY 500


#endif // MAIN_CONSTANTS_H