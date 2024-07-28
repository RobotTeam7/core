#include <common/pwm.h>


// Consult https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/ledc.html for info on ESP32 function calls

// ESP32 PWM works by binding a pin to a channel, and then we configure the PWM signal by 
// specifying the channel, not pin. So, we need to keep track of which pin corresponds to which channel.
int8_t pin_to_channel[NUM_GPIO_PINS]; 
int8_t channel_to_pin[MAX_AVAILABLE_PWM_CHANNELS];  // Keep track of channel availability
uint8_t timer_allocations[MAX_AVAILABLE_TIMERS];    // Timers have a physical maximum number of channels that can be allocated to them
int current_channel = 0;

void init_pwm() {
    memset(channel_to_pin, CHANNEL_IS_AVAILABLE, sizeof(channel_to_pin));   // Initialize channel numbers to be available
    memset(pin_to_channel, UNSPECIFIED_CHANNEL, sizeof(pin_to_channel));    // Initialize pins to be unbound
    memset(timer_allocations, 0, sizeof(timer_allocations));                // Initialize timer allocations to 0

    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .duty_resolution  = TIMER_RESOLUTION,
        .timer_num        = MOTOR_TIMER,
        .freq_hz          = LEDC_PWM_FREQUENCY,
        .clk_cfg          = LEDC_AUTO_CLK
    };

    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));
}

int allocate_channel(uint8_t pin, int channel) {
    int allocated_channel_num = UNSPECIFIED_CHANNEL;

    if (channel != UNSPECIFIED_CHANNEL) { // If we specified a channel we want, check if its available
        if (channel_to_pin[channel] == CHANNEL_IS_AVAILABLE) {
            allocated_channel_num = channel;
        }
    } else { // Otherwise, loop until an available channel can be found 
        for (int i = 0; i < MAX_AVAILABLE_PWM_CHANNELS; i++) {
            if (channel_to_pin[i] == CHANNEL_IS_AVAILABLE) {
                allocated_channel_num = i;
                break;
            }
        }
    }
    
    if (allocated_channel_num == UNSPECIFIED_CHANNEL) { // If we couldn't allocate a channel, say so
        log_error("Couldn't allocate channel!");
    } else {
        pin_to_channel[pin] = allocated_channel_num;    // Otherwise, map the pin to the allocated channel 
        channel_to_pin[allocated_channel_num] = pin;    // Mark the channel as allocated
    }

    return allocated_channel_num;
}

void bind_pwm(uint8_t pin, uint32_t frequency, ledc_timer_t timer, ledc_timer_bit_t bit_num) {
    // Allocate a channel
    int channel = allocate_channel(pin);
    if (channel == UNSPECIFIED_CHANNEL) {
        log_error("PWM binding failed!");
        return;
    }
    
    // Ensure the timer can handle another channel
    if (timer_allocations[timer] < MAX_AVAILABLE_TIMERS) {
        timer_allocations[timer] += 1;
    } else {
        log_error("PWM binding failed! Timer would exceed max channel allocations.");
        return;
    }

    pinMode(pin, OUTPUT);

    ledc_channel_config_t ledc_channel = {
        .gpio_num       = pin,
        .speed_mode     = LEDC_PWM_SPEED_MODE,
        .channel        = (ledc_channel_t)channel,
        .intr_type      = LEDC_INTR_DISABLE,
        .timer_sel      = timer,
        .duty           = 0,    // Initialize duty cycle to off
        .hpoint         = 0     // Honestly, no clue what this does, but 0 seems to make this have no effect
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
}

void set_pwm(int pin, uint32_t power) {
    int8_t channel = pin_to_channel[pin];

    if (channel == UNSPECIFIED_CHANNEL) {
        log_error("Pin has not been bound to a PWM channel!");
        return;
    }

    if (power > UINT16_MAX) {
        log_error("Given duty cycle power exceeds max value for PWM resolution! Must be 0-65355.");
    }

    ledc_set_duty(LEDC_PWM_SPEED_MODE, (ledc_channel_t)channel, power);
    ledc_update_duty(LEDC_PWM_SPEED_MODE, (ledc_channel_t)channel);
}
