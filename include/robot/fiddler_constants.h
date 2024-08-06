#if robot == 1 // FIDDLER

#ifndef FIDDLER_CONSTANTS_H
#define FIDDLER_CONSTANTS_H


// Pin Definitions
#define SERVO_VERTICAL_PIN              (4)
#define RACK_FORWARD_PIN                (8)
#define RACK_REVERSE_PIN                (7)
#define SERVO_CLAW_PIN                  (13)
#define SERVO_DRAW_BRIDGE_PIN           (22)
#define SWITCH_RACK_CLAWSIDE            (9)
#define SWITCH_RACK_PLATESIDE           (10)
#define SERVO_PLATE_PIN                 (19)
#define RX_PIN                          (27)
#define TX_PIN                          (14)


// Servo Positions
#define SERVO_PLATE_OPEN                (0.065)
#define SERVO_PLATE_CLOSED              (0.032)
#define SERVO_DRAW_BRIDGE_DOWN          (0.06)
#define SERVO_DRAW_BRIDGE_UP            (0.10)
#define SERVO_CLAW_OPEN                 (0.110)
#define SERVO_CLAW_CLOSED               (0.070)
#define SERVO_VERTICAL_UP               (0.0275)
#define SERVO_VERTICAL_DOWN             (0.110)


#endif // FIDDLER_CONSTANTS_H
#endif // FIDDLER