#include <Arduino.h>
#include <tape/tape_following_config.h>
#include <tape/reflectance_polling_config.h>
#include <FreeRTOS.h>
#include <task.h>

#define THRESHOLD 150
#define REFLECTANCE_ONE PA5
#define REFLECTANCE_TWO PA4


void TaskPollReflectance(void *pvParameters) {
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

    for(;;) {
        reflectance_right = analogRead(REFLECTANCE_ONE);
        reflectance_left = analogRead(REFLECTANCE_TWO);

        config->right_sensor_buffer->push(reflectance_right);
        config->left_sensor_buffer->push(reflectance_left);

        Serial.print("Left buffer contents: ");
    for (int i = 0; i < config->left_sensor_buffer->size(); ++i) {
        Serial.print((*config->left_sensor_buffer)[i]);
        Serial.print(" ");
    }
        Serial.println();

        Serial.print("Right buffer contents: ");
    for (int i = 0; i < config->right_sensor_buffer->size(); ++i) {
        Serial.print((*config->right_sensor_buffer)[i]);
        Serial.print(" ");
    }
    Serial.println();
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}