#ifndef ROBOT_PWM_H
#define ROBOT_PWM_H

#include <Arduino.h>
#include <driver/ledc.h>
#include <esp_err.h>
#include <esp32-hal-ledc.h>

#include <common/resource_manager.h>
#include <common/constants.h>
#include <common/utils.h>


#define MAX_AVAILABLE_PWM_CHANNELS  (SOC_LEDC_CHANNEL_NUM<<1)
#define MAX_AVAILABLE_TIMERS        SOC_TIMER_GROUP_TOTAL_TIMERS
#define MAX_CHANNELS_PER_TIMER      4
#define LEDC_PWM_SPEED_MODE         LEDC_LOW_SPEED_MODE
#define NUM_GPIO_PINS               39
#define PWM_LEDC_GROUP              0
#define CHANNEL_IS_AVAILABLE        -1
#define UNSPECIFIED_CHANNEL         -1

/**
 * @brief Perform initialization of PWM hardware and custom software peripherals. Required before using other 
 * functions supplied by this module.
 */
void init_pwm();

/**
 * @brief Bind a pin to become a PWM signal emitter.
 * @param pin The pin that will be bound.
 * @param frequency frequency in Hz. (default is 50 Hz)
 * @param timer the timer index that will be used (note: max `MAX_CHANNELS_PER_TIMER` channels are allowed per timer). Default is timer 0.
 * @param bit_num the resolution of the PWM duty cycle in bits. Default is 16 bits (0-65355 resolution).
 */
void bind_pwm(uint8_t pin, uint32_t frequency = LEDC_PWM_FREQUENCY, ledc_timer_t timer = LEDC_TIMER_0, ledc_timer_bit_t bit_num = LEDC_TIMER_16_BIT);

/**
 * @brief Set the drive power of a PWM emitting pin.
 * @param pin The pin whose output will be modified.
 * @param power Drive power, must be 0 – 65535 (range of uint16_t).
 */
void set_pwm(int pin, uint32_t power);

/**
 * @brief Allocate a PWM channel to be used, or -1 if none are available.
 * 
 * @param pin The pin that will be mapped to the channel, if it can be allocated
 * @param channel optionally, specify the channel that you would like to allocate to `pin`.
 * 
 * @returns The allocated PWM channel number.
 */
int allocate_channel(uint8_t pin, int channel = UNSPECIFIED_CHANNEL);


#endif // ROBOT_PWM_H