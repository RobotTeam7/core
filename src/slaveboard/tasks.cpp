#include <Arduino.h>
#include <FreeRTOS.h>
#include <task.h>
#include <slaveboard/constants.h>
#include <slaveboard/tasks.h>

#define POLL_SENSOR_DELAY_MS 1
#define MOTOR_TASK_DELAY_MS 10

#define THRESHOLD 300
#define REFLECTANCE_ONE PA5
#define REFLECTANCE_TWO PA4

// returns the mean value in the buffer
int get_buffer_average(CircularBuffer<int, BUFFER_SIZE> &sensor_buffer)
{
  int sum = 0;
  for (int i = 0; i < sensor_buffer.size(); ++i)
  {
    sum += sensor_buffer[i];
  }
  // truncates integer
  return sum / sensor_buffer.size();
}

void TaskPollReflectance(void *pvParameters) {
    TickType_t delay_ticks = pdMS_TO_TICKS(POLL_SENSOR_DELAY_MS);

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

        config->right_sensor_buffer->push(reflectance_right);
        config->left_sensor_buffer->push(reflectance_left);

        // char drive_state[20] = "find tape";
        // if (reflectance_right - reflectance_left > THRESHOLD) {
        //     strcpy(drive_state, "---->>");
        // } else if (reflectance_left - reflectance_right > THRESHOLD) {
        //     strcpy(drive_state, "<<----");
        // } else {
        //     strcpy(drive_state, "^^^^^^");
        // }
        // Serial.println(drive_state);

        vTaskDelay(delay_ticks);
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
        // Serial.println("PLS JUST DO IT MAN");
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
        
        if (right_mean - left_mean > THRESHOLD) {
            config->motor_front_left->set_drive(MOTOR_SPEED_LOW, forward); 
            config->motor_back_left->set_drive(MOTOR_SPEED_LOW, forward);
            config->motor_front_right->set_drive(MOTOR_SPEED_HIGH, forward);
            config->motor_back_right->set_drive(MOTOR_SPEED_HIGH, forward);
        } else if(left_mean - right_mean > THRESHOLD) {
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

        vTaskDelay(5);
    }
}

// Rotates the robot 180 degrees counter-clockwise
void TaskRotate(void *pvParameters)
{
  Serial.println("Rotating...");
  // convert ms delays into ticks
  TickType_t inital_delay_ticks = pdMS_TO_TICKS(1000);
  TickType_t poll_rate_ticks = pdMS_TO_TICKS(MOTOR_ADJUSTMENT_DELAY_ROTATING_MS);

  MotorReflectanceConfig *config = static_cast<MotorReflectanceConfig *>(pvParameters);

  // Ensure config is not null
  if (config == nullptr)
  {
    Serial.println("Error: Rotate config is null");
    vTaskDelete(NULL);
    return;
  }

  // Ensure contents of config are not null
  if (config->reflectancePollingConfig->left_sensor_buffer == nullptr || config->reflectancePollingConfig->right_sensor_buffer == nullptr)
  {
    Serial.println("Error: Rotate Config buffer is null");
    vTaskDelete(NULL);
    return;
  }

  if (config->motor_front_right == nullptr || config->motor_back_right == nullptr || config->motor_front_left == nullptr || config->motor_back_left == nullptr)
  {
    Serial.println("Error: One of rotate config motors is null");
    vTaskDelete(NULL);
    return;
  }
  else
  {
    Serial.println("Acquired Rotate Config");
  }

  bool rotating;
  for (;;)
  {
    // Serial.println("rotating");

    // start rotating, then delay to ensure that rotation isn't immediately canceled by tape detection
    config->motor_front_right->set_drive(MOTOR_SPEED_ROTATION, forward);
    config->motor_back_right->set_drive(MOTOR_SPEED_ROTATION, forward);
    config->motor_front_left->set_drive(MOTOR_SPEED_ROTATION, reverse);
    config->motor_back_left->set_drive(MOTOR_SPEED_ROTATION, reverse);
    vTaskDelay(inital_delay_ticks);

    // look for tape detection
    rotating = true;
    while (rotating)
    {
      int left_mean = get_buffer_average(*(config->reflectancePollingConfig->left_sensor_buffer));
      int right_mean = get_buffer_average(*(config->reflectancePollingConfig->right_sensor_buffer));

      if (right_mean > THRESHOLD_SENSOR_SINGLE && left_mean > THRESHOLD_SENSOR_SINGLE)
      {
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
        if (xQueueSend(*config->xSharedQueue, &message, portMAX_DELAY) != pdPASS)
        {
          Serial.println("Failed to send to xSharedQueue");
        }
      }

      vTaskDelay(poll_rate_ticks);
    }
  }
}