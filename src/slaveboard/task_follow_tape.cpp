#include <Arduino.h>
#include <tape/motor_reflectance_config.h>
#include <FreeRTOS.h>
#include <task.h>

#include <reflectance/get_buffer_average.h>
#include <constants.h>

void TaskFollowTape(void *pvParameters) {

    TickType_t delay_ticks = pdMS_TO_TICKS(MOTOR_ADJUSTMENT_DELAY_TAPE_FOLLOWING_MS);
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
        Serial.println("here in follow loop!");
        int left_mean = get_buffer_average(*(config->reflectancePollingConfig->left_sensor_buffer));
        int right_mean = get_buffer_average(*(config->reflectancePollingConfig->right_sensor_buffer));

        // char drive_state[20] = "find tape";

        // if (right_mean - left_mean > THRESHOLD) {
        //     strcpy(drive_state, "---->>");
        // } else if (left_mean - right_mean > THRESHOLD) {
        //     strcpy(drive_state, "<<----");
        // } else {
        //     strcpy(drive_state, "^^^^^^");
        // }
        // Serial.println(drive_state);
        
        if (right_mean - left_mean > THRESHOLD_SENSOR_DIFFERENCE) {
            config->motor_front_left->set_drive(MOTOR_SPEED_LOW, forward); 
            config->motor_back_left->set_drive(MOTOR_SPEED_LOW, forward);
            config->motor_front_right->set_drive(MOTOR_SPEED_HIGH, forward);
            config->motor_back_right->set_drive(MOTOR_SPEED_HIGH, forward);
        } else if(left_mean - right_mean > THRESHOLD_SENSOR_DIFFERENCE) {
            config->motor_front_left->set_drive(MOTOR_SPEED_HIGH, forward);
            config->motor_back_left->set_drive(MOTOR_SPEED_HIGH, forward);
            config->motor_front_right->set_drive(MOTOR_SPEED_LOW, forward);
            config->motor_back_right->set_drive(MOTOR_SPEED_LOW, forward);
        } else {
            config->motor_front_left->set_drive(MOTOR_SPEED_HIGH, forward);
            config->motor_back_left->set_drive(MOTOR_SPEED_HIGH, forward);
            config->motor_front_right->set_drive(MOTOR_SPEED_HIGH, forward);
            config->motor_back_right->set_drive(MOTOR_SPEED_HIGH, forward);
        }

        vTaskDelay(delay_ticks);
    }
}