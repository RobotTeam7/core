#include <common/pwm.h>
#include <Arduino.h>
#include <common/channel_manager.h>
#include <map>

#define TIMER_RESOLUTION   16  // 16-bit resolution such that 0 – 65535 is valid
#define LEDC_PWM_FREQUENCY 50  // 50 Hz PWM pulse train

#if USING_ESP32
    #pragma message "Compiling for ESP32"

    // ESP32 PWM works by binding a pin to a channel, and then we configure the PWM signal by 
    // specifying the channel, not pin. But, as BP PWMs is configured solely through pin and we want 
    // to keep a consistent interface, we will keep a map of pins to channels.
    std::map<int, int> pin_to_channel {};

    void pwm::bind_pwm(int pin) {
        int channel = ChannelManager::getInstance().requestChannel();

        if (channel == -1) {
            return;
        }

        pin_to_channel[pin] = channel;

        ledcSetup(channel, LEDC_PWM_FREQUENCY, TIMER_RESOLUTION);
        
        pinMode(pin, OUTPUT);
        ledcAttachPin(pin, channel);
    }

    void pwm::set_pwm(int pin, int power) {
        int channel = pin_to_channel[pin];
        ledcWrite(channel, power);

        Serial.print("Setting power of channel ");
        Serial.print(channel);
        Serial.print(" on pin ");
        Serial.print(pin);
        Serial.print(" to ");
        Serial.println(power);
    }

#elif USING_BLUE_PILL
    #pragma message "Compiling for BluePill"

    std::map<int, PinName> pinMap = {
        {PA0, PA_0},
        {PA1, PA_1},
        {PA2, PA_2},
        {PA3, PA_3},
        {PA4, PA_4},
        {PA5, PA_5},
        {PA6, PA_6},
        {PA7, PA_7},
        {PA8, PA_8},
        {PA9, PA_9},
        {PA10, PA_10},
        {PA11, PA_11},
        {PA12, PA_12},
        {PA13, PA_13},
        {PA14, PA_14},
        {PA15, PA_15},
        {PB0, PB_0},
        {PB1, PB_1},
        {PB2, PB_2},
        {PB3, PB_3},
        {PB4, PB_4},
        {PB5, PB_5},
        {PB6, PB_6},
        {PB7, PB_7},
        {PB8, PB_8},
        {PB9, PB_9},
        {PB10, PB_10},
        {PB11, PB_11},
        {PB12, PB_12},
        {PB13, PB_13},
        {PB14, PB_14},
        {PB15, PB_15},
    };

    void pwm::bind_pwm(int pin) {
        pinMode(pin, OUTPUT);
        pwm_start(pinMap[pin], LEDC_PWM_FREQUENCY, 0, TimerCompareFormat_t::RESOLUTION_16B_COMPARE_FORMAT);
    }

    void pwm::set_pwm(int pin, int power) {
        pwm_start(pinMap[pin], LEDC_PWM_FREQUENCY, power, TimerCompareFormat_t::RESOLUTION_16B_COMPARE_FORMAT);
    }

#else
    #error "Couldn't identify board!"
#endif