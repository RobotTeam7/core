#include <common/pwm.h>


// ESP32 PWM works by binding a pin to a channel, and then we configure the PWM signal by 
// specifying the channel, not pin. But, as BP PWMs is configured solely through pin and we want 
// to keep a consistent interface, we will keep a map of pins to channels.
std::map<int, int> pin_to_channel {};

void bind_pwm(int pin) {
    int channel = ChannelManager::getInstance().requestChannel();

    if (channel == -1) {
        return;
    }

    pin_to_channel[pin] = channel;

    ledcSetup(channel, LEDC_PWM_FREQUENCY, TIMER_RESOLUTION);

    pinMode(pin, OUTPUT);
    ledcAttachPin(pin, channel);

    // Serial.println("Bound to pin: " + String(pin) + " on channel: " + channel);
}

void set_pwm(int pin, uint32_t power) {
    int channel = pin_to_channel[pin];
    ledcWrite(channel, power);

    // Serial.println("Setting power of channel " + String(channel) + " on pin " + pin + " to " + power);
}
