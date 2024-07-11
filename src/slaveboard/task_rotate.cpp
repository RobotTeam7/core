#include <Arduino.h>
#include <tape/motor_reflectance_config.h>
#include <FreeRTOS.h>
#include <task.h>

#include <FreeRTOS/Source/include/queue.h>
#include <reflectance/get_buffer_average.h>
#include <constants.h>

// how long before the robot starts looking for tape detection
// prevents rotation from immediately canceling ]
#define INITIAL_DELAY_MS 1000


// enum specifying messages, will need to be changed in the future
typedef enum {
  ROTATION_DONE,
} Message;

// Rotates the robot 180 degrees counter-clockwise
void TaskRotate(void *pvParameters) {
    // convert ms delays into ticks
    TickType_t inital_delay_ticks = pdMS_TO_TICKS(INITIAL_DELAY_MS);
    TickType_t poll_rate_ticks = pdMS_TO_TICKS(MOTOR_ADJUSTMENT_DELAY_ROTATING_MS);

    MotorReflectanceConfig* config = static_cast<MotorReflectanceConfig*>(pvParameters);

    // Ensure config is not null
    if (config == nullptr) {
        Serial.println("Error: Rotate config is null");
        vTaskDelete(NULL);
        return;
    }

    // Ensure contents of config are not null
    if (config->reflectancePollingConfig->left_sensor_buffer == nullptr || config->reflectancePollingConfig->right_sensor_buffer == nullptr) {
        Serial.println("Error: Rotate Config buffer is null");
        vTaskDelete(NULL); 
        return;
    }

    if (config->motor_front_right == nullptr || config->motor_back_right == nullptr || config->motor_front_left == nullptr || config->motor_back_left == nullptr) {
        Serial.println("Error: One of rotate config motors is null");
        vTaskDelete(NULL); 
        return;
    } else {
        Serial.println("Acquired Rotate Config");
    }

    bool rotating;
    for(;;) {
        // start rotating, then delay to ensure that rotation isn't immediately canceled by tape detection
        config->motor_front_right->set_drive(MOTOR_SPEED_ROTATION, forward);
        config->motor_back_right->set_drive(MOTOR_SPEED_ROTATION, forward);
        config->motor_front_left->set_drive(MOTOR_SPEED_ROTATION, reverse);
        config->motor_back_left->set_drive(MOTOR_SPEED_ROTATION, reverse);
        vTaskDelay(inital_delay_ticks);

        // look for tape detection
        rotating = true;
        while(rotating) {
            int left_mean = get_buffer_average(*(config->reflectancePollingConfig->left_sensor_buffer));
            int right_mean = get_buffer_average(*(config->reflectancePollingConfig->right_sensor_buffer));
            if(right_mean > THRESHOLD_SENSOR_SINGLE && left_mean > THRESHOLD_SENSOR_SINGLE) {
                Serial.println("ending rotation");
                rotating = false;
                // Serial.println(left_mean);
                // Serial.println(right_mean);
                config->motor_front_right->stop();
                config->motor_back_right->stop();
                config->motor_front_left->stop();
                config->motor_back_left->stop();

                // send message to TaskMaster that rotation has finished
                Message message = ROTATION_DONE;
                if (xQueueSend((*config->reflectancePollingConfig->xSharedQueue), &message, portMAX_DELAY) != pdPASS) {
                    Serial.println("Failed to send to xSharedQueue");
                }
            }

            vTaskDelay(poll_rate_ticks);
        }
    }
}