#ifndef INDEPENDENT_PWM_H
#define INDEPENDENT_PWM_H

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
    void set_pwm(int pin, int power);
}

#endif