#include <Arduino.h>
#include <tape/tape_following_config.h>
#include <tape/reflectance_polling_config.h>
#include <FreeRTOS.h>
#include <task.h>

// #include <Adafruit_SSD1306.h>
// #include <string.h>

#define THRESHOLD 150
#define REFLECTANCE_ONE PA5
#define REFLECTANCE_TWO PA4
#define MOTOR_SPEED 4000


#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET 	-1 // This display does not have a reset pin accessible
Adafruit_SSD1306 display_handler(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);



void TaskPollReflectance(void *pvParameters) {
    TickType_t delay_ticks = pdMS_TO_TICKS(20);

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

    // display code
    // char drive_state[20] = "find tape";
    // if (reflectance_right - reflectance_left > THRESHOLD) {
    //   strcpy(drive_state, "---->>");
    // } else if (reflectance_left - reflectance_right > THRESHOLD) {
    //   strcpy(drive_state, "<<----");
    // }else {
    //   strcpy(drive_state, "^^^^^");
    // }
    
    // display_handler.clearDisplay();
    // display_handler.setCursor(0,0);
    // display_handler.println("Sensor Right:");
    // display_handler.setCursor(0,10);
    // display_handler.println(reflectance_right);
    // display_handler.setCursor(0,20);
    // display_handler.println("Sensor Left:");
    // display_handler.setCursor(0,30);
    // display_handler.println(reflectance_left);
    // display_handler.setCursor(50, 50);
    // display_handler.println(drive_state);
    // display_handler.display();
    //end diplay code

    vTaskDelay(delay_ticks);
  }
}

void TaskFollowTape(void *pvParameters) {
    TickType_t delay_ticks = pdMS_TO_TICKS(100);
    TapeFollowingConfig* config = static_cast<TapeFollowingConfig*>(pvParameters);

    // Ensure config is not null
    if (config == nullptr) {
        Serial.println("Error: TapeFollowingConfig is null");
        // This deletes the task
        vTaskDelete(NULL);
        return;
    }

    // Ensure contents of config are not null
    if (config->left_sensor_buffer == nullptr || config->right_sensor_buffer == nullptr) {
        Serial.println("Error: TapeFollowingConfig buffer is null");
        vTaskDelete(NULL); 
        return;
    }

    if(config->motor_front_right == nullptr || config->motor_back_right == nullptr || config->motor_front_left == nullptr || config->motor_back_left == nullptr) {
        Serial.println("Error: TapeFollowingConfig motor is null");
        vTaskDelete(NULL); 
        return;
    }

    for(;;) {
        int left_mean;
        int right_mean;
        int right_sum = 0;
        int left_sum = 0;
        for (int i = 0; i < config->left_sensor_buffer->size(); ++i) {
            left_sum += (*config->left_sensor_buffer)[i];
        }
        left_mean = left_sum / config->left_sensor_buffer->size();
        Serial.println("left mean: ");
        Serial.print(left_mean);
        Serial.println(" ");


        for (int i = 0; i < config->right_sensor_buffer->size(); ++i) {
            right_sum += (*config->right_sensor_buffer)[i];
        }
        right_mean = right_sum / config->right_sensor_buffer->size();
        Serial.println("right mean: ");
        Serial.print(right_mean);

        Serial.println(" ");
        
        if(right_mean - left_mean > THRESHOLD) {
            config->motor_front_left->set_drive(MOTOR_SPEED, forward);
            config->motor_back_left->set_drive(MOTOR_SPEED, forward);
            config->motor_front_right->stop();
            config->motor_back_right->stop();
        }else if(left_mean - right_mean > THRESHOLD) {
            config->motor_front_left->stop(); 
            config->motor_back_left->stop();
            config->motor_front_right->set_drive(MOTOR_SPEED, forward);
            config->motor_back_right->set_drive(MOTOR_SPEED, forward);
        }else {
            config->motor_front_left->set_drive(MOTOR_SPEED, forward);
            config->motor_back_left->set_drive(MOTOR_SPEED, forward);
            config->motor_front_right->set_drive(MOTOR_SPEED, forward);
            config->motor_back_right->set_drive(MOTOR_SPEED, forward);
        }

        vTaskDelay(delay_ticks);
    }
}