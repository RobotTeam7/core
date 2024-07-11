#include <Arduino.h>
#include <FreeRTOS.h>
#include <task.h>

#include <constants.h>
#include <reflectance/reflectance_polling_config.h>

#define REFLECTANCE_ONE PA5
#define REFLECTANCE_TWO PA4


void TaskPollReflectance(void *pvParameters) {
    TickType_t delay_ticks = pdMS_TO_TICKS(POLL_RATE_REFLECTANCE_MS);

    pinMode(REFLECTANCE_ONE, INPUT);
    pinMode(REFLECTANCE_TWO, INPUT);
    int reflectance_right;
    int reflectance_left;
    ReflectancePollingConfig* config = static_cast<ReflectancePollingConfig*>(pvParameters);

    // Ensure config is not null
    if (config == nullptr) {
        Serial.println("Error: ReflectancePollingConfig is null");
        // This deletes the task
        vTaskDelete(NULL);
        return;
    }

    // Ensure left and right sensor buffers are not null
    if (config->left_sensor_buffer == nullptr || config->right_sensor_buffer == nullptr) {
        Serial.println("Error: Sensor buffers are null");
        vTaskDelete(NULL); 
        return;
    }

    while (1) {
        reflectance_right = analogRead(REFLECTANCE_ONE);
        reflectance_left = analogRead(REFLECTANCE_TWO);

        //prints reflectance values
        // Serial.println("reflectance right:");
        // Serial.println(reflectance_right);
        // Serial.println("reflectance left:");
        // Serial.println(reflectance_left);

        config->right_sensor_buffer->push(reflectance_right);
        config->left_sensor_buffer->push(reflectance_left);

        vTaskDelay(delay_ticks);
    }
}


