#ifndef RACK_AND_PINION_H
#define RACK_AND_PINION_H

#include <stdint.h>
#include <common/robot_motor.h>
#include <common/limit_switch.h>


extern RobotMotor_t* rack_and_pinion;

/**
 * @brief Initialize the rack and pinion.
 * @param initial_position the initial position, where 1: claw forwards and -1: forklift forwards
 */
void init_rack_and_pinion(uint8_t fowards_pin, uint8_t backwards_pin, int initial_position, uint8_t claw_limit_switch_pin, uint8_t forklift_limit_switch_pin);
void actuate_claw_forwards();
void actuate_forklift_forwards();


#endif // RACK_AND_PINION_H