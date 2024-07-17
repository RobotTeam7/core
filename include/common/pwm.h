#ifndef INDEPENDENT_PWM_H
#define INDEPENDENT_PWM_H


#include <Arduino.h>
#include <map>

#include <common/resource_manager.h>
#include <common/constants.h>
#include <common/pin.h>


/// @brief Namespace containing methods for PWM signal control
namespace pwm {
    /**
     * @brief Bind a pin to become a PWM signal emitter
     * @param pin The pin that will be bound.
     */
    void bind_pwm(int pin);

    /**
     * @brief Set the drive power of a PWM emitting pin.
     * @param pin The pin whose output will be modified.
     * @param power Drive power, must be 0 – 65535 (range of uint16_t).
     */
    void set_pwm(int pin, uint32_t power);
}


#endif // INDEPENDENT_PWM_H