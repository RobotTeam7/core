#ifndef INDEPENDENT_PWM_H
#define INDEPENDENT_PWM_H

namespace pwm {
    void bind_pwm(int pin);
    void set_pwm(int pin, int power);
}

#endif