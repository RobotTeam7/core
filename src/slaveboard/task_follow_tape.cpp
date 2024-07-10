#include <Arduino.h>
#include <tape/motor_reflectance_config.h>
#include <FreeRTOS.h>
#include <task.h>

#define MOTOR_TASK_DELAY_MS 10

#define THRESHOLD 300
#define MOTOR_SPEED 8000

void TaskFollowTape(void *pvParameters) {
    TickType_t delay_ticks = pdMS_TO_TICKS(MOTOR_TASK_DELAY_MS);
    MotorReflectanceConfig* config = static_cast<MotorReflectanceConfig*>(pvParameters);

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

        vTaskDelay(delay_ticks);
    }
}