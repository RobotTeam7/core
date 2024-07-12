#include <Arduino.h>
#include <tape/tape_following_config.h>
#include <tape/reflectance_polling_config.h>
#include <FreeRTOS.h>
#include <task.h>

#define POLL_SENSOR_DELAY_MS 1
#define MOTOR_TASK_DELAY_MS 10

#define THRESHOLD 300
#define REFLECTANCE_ONE PA5
#define REFLECTANCE_TWO PA4
#define MOTOR_SPEED 8000


void TaskPollReflectance(void *pvParameters) {
    // TickType_t delay_ticks = pdMS_TO_TICKS(POLL_SENSOR_DELAY_MS);

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
        Serial.println("POLLING?!");
        reflectance_right = analogRead(REFLECTANCE_ONE);
        reflectance_left = analogRead(REFLECTANCE_TWO);

        config->right_sensor_buffer->push(reflectance_right);
        config->left_sensor_buffer->push(reflectance_left);

        vTaskDelay(5);
    }
}

void TaskFollowTape(void *pvParameters) {
    // TickType_t delay_ticks = pdMS_TO_TICKS(MOTOR_TASK_DELAY_MS);
    TapeFollowingConfig* config = static_cast<TapeFollowingConfig*>(pvParameters);

    // Ensure config is not null
    if (config == nullptr) {
        Serial.println("Error: TapeFollowingConfig is null");
        // This deletes the task
        vTaskDelete(NULL);
        return;
    }

    // Ensure contents of config are not null
    if (config->reflectancePollingConfig->left_sensor_buffer == nullptr || config->reflectancePollingConfig->right_sensor_buffer == nullptr) {
        Serial.println("Error: TapeFollowingConfig buffer is null");
        vTaskDelete(NULL); 
        return;
    }

    if (config->motor_front_right == nullptr || config->motor_back_right == nullptr || config->motor_front_left == nullptr || config->motor_back_left == nullptr) {
        Serial.println("Error: TapeFollowingConfig motor is null");
        vTaskDelete(NULL); 
        return;
    } else {
        Serial.println("Acquired TapeFollowingConfig");
    }

    while (1) {
        int left_mean;
        int right_mean;
        int right_sum = 0;
        int left_sum = 0;

        for (int i = 0; i < config->reflectancePollingConfig->left_sensor_buffer->size(); ++i) {
            left_sum += (*config->reflectancePollingConfig->left_sensor_buffer)[i];
        }
        left_mean = left_sum / config->reflectancePollingConfig->left_sensor_buffer->size();


        for (int i = 0; i < config->reflectancePollingConfig->right_sensor_buffer->size(); ++i) {
            right_sum += (*config->reflectancePollingConfig->right_sensor_buffer)[i];
        }
        right_mean = right_sum / config->reflectancePollingConfig->right_sensor_buffer->size();

        char drive_state[20] = "find tape";

        if (right_mean - left_mean > THRESHOLD) {
            strcpy(drive_state, "---->>");
        } else if (left_mean - right_mean > THRESHOLD) {
            strcpy(drive_state, "<<----");
        } else {
            strcpy(drive_state, "^^^^^^");
        }
        Serial.println(drive_state);
        
        if (right_mean - left_mean > THRESHOLD) {
            config->motor_front_left->set_drive(MOTOR_SPEED / 2, forward); 
            config->motor_back_left->set_drive(MOTOR_SPEED / 2, forward);
            config->motor_front_right->set_drive(MOTOR_SPEED, forward);
            config->motor_back_right->set_drive(MOTOR_SPEED, forward);
        } else if(left_mean - right_mean > THRESHOLD) {
            config->motor_front_left->set_drive(MOTOR_SPEED, forward);
            config->motor_back_left->set_drive(MOTOR_SPEED, forward);
            config->motor_front_right->set_drive(MOTOR_SPEED / 2, forward);
            config->motor_back_right->set_drive(MOTOR_SPEED / 2, forward);
        } else {
            config->motor_front_left->set_drive(MOTOR_SPEED, forward);
            config->motor_back_left->set_drive(MOTOR_SPEED, forward);
            config->motor_front_right->set_drive(MOTOR_SPEED, forward);
            config->motor_back_right->set_drive(MOTOR_SPEED, forward);
        }

        vTaskDelay(5);
    }
}