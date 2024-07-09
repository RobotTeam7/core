#include <common/pwm.h>

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

        Serial.println("Bound to pin: " + String(pin) + " on channel: " + channel);
    }

    void pwm::set_pwm(int pin, uint32_t power) {
        int channel = pin_to_channel[pin];
        ledcWrite(channel, power);

        Serial.println("Setting power of channel " + String(channel) + " on pin " + pin + " to " + power);
    }

#elif USING_BLUE_PILL
    #pragma message "Compiling for BluePill"

    inline PinName getPin(int pin) {
        return PinConvert::pin_number_to_pin_name[pin];
    }

    void pwm::bind_pwm(int pin) {
        pinMode(pin, OUTPUT);
        PinName motorPin = getPin(pin);
        pwm_start(motorPin, LEDC_PWM_FREQUENCY, 0, TimerCompareFormat_t::RESOLUTION_16B_COMPARE_FORMAT);
        Serial.println("Bound to pin: " + String(motorPin));
    }

    void pwm::set_pwm(int pin, uint32_t power) {
        PinName motorPin = getPin(pin);
        pwm_start(motorPin, LEDC_PWM_FREQUENCY, power, TimerCompareFormat_t::RESOLUTION_16B_COMPARE_FORMAT);
        Serial.println("Setting pin: " + String(motorPin) + " to " + String(power));
    }

#else
    #error "Couldn't identify board! Please set a build flag for which board you are trying to flash in platformio.ini!"
#endif