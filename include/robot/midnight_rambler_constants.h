#if robot == 0 // MIDNIGHT RAMBLER

#ifndef MIDNIGHT_RAMBLER_CONSTANTS_H
#define MIDNIGHT_RAMBLER_CONSTANTS_H

// Pin Definitions
#define SERVO_VERTICAL_PIN              (4)
#define SERVO_VERTICAL_UP               (0.05)
#define SERVO_VERTICAL_DOWN             (0.1175)

#define RACK_FORWARD_PIN                (8)
#define RACK_REVERSE_PIN                (7)

#define SERVO_CLAW_PIN                  (13)
#define SERVO_CLAW_OPEN                 (0.052)
#define SERVO_CLAW_CLOSED               (0.028)

#define SERVO_DRAW_BRIDGE_PIN           (22)
#define SERVO_DRAW_BRIDGE_DOWN          (0.090)
#define SERVO_DRAW_BRIDGE_UP            (0.130)

#define SERVO_PLATE_PIN                 (19)
#define SERVO_PLATE_OPEN                (0.065)
#define SERVO_PLATE_CLOSED              (0.030)

#define SWITCH_RACK_CLAWSIDE            (9)
#define SWITCH_RACK_PLATESIDE           (10)

#define TX_PIN                          (14)
#define RX_PIN                          (27)

#endif // MIDNIGHT_RAMBLER_CONSTANTS_H
#endif // sMIDNIGHT RAMBLER