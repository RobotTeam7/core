#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <common/robot_motor.h>
#include <common/pin.h>
#include <string.h>
#include <FreeRTOS.h>
#include <task.h>

// tape following imports
#include <tape/reflectance_polling_config.h>
#include <tape/tape_following_config.h>
#include <tape/task_follow_tape.h>
#include <tape/motor_reflectance_config.h>
#include <constants.h>

#define MOTOR_BACK_RIGHT_FORWARD PB1
#define MOTOR_BACK_RIGHT_REVERSE PB0
#define MOTOR_BACK_LEFT_FORWARD PA7
#define MOTOR_BACK_LEFT_REVERSE PA6
#define MOTOR_FRONT_LEFT_FORWARD PA1
#define MOTOR_FRONT_LEFT_REVERSE PA0
#define MOTOR_FRONT_RIGHT_FORWARD PA3
#define MOTOR_FRONT_RIGHT_REVERSE PA2

#define SPEED 16000
#define TIME_DELAY_MOTOR 500

#define PRIORITY_REFLECTANCE_POLLING 3
#define PRIORITY_FOLLOW_TAPE 1

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET  -1 // This display does not have a reset pin accessible


RobotMotor motor_front_left;
RobotMotor motor_front_right;
RobotMotor motor_back_left;
RobotMotor motor_back_right;
ReflectancePollingConfig config_reflectance;
TapeFollowingConfig config_following;
MotorReflectanceConfig config_rotate;
CircularBuffer<int, BUFFER_SIZE> leftBuffer;
CircularBuffer<int, BUFFER_SIZE> rightBuffer;

// returns the mean value in the buffer
int get_buffer_average(CircularBuffer<int, BUFFER_SIZE>& sensor_buffer) {
    int sum = 0;
    for (int i = 0; i < sensor_buffer.size(); ++i) {
        sum += sensor_buffer[i];
    }
    // truncates integer
    return sum / sensor_buffer.size();
}

// enum specifying messages, will need to be changed in the future
enum Message { ROTATION_DONE};

// Rotates the robot 180 degrees counter-clockwise
void TaskRotate(void *pvParameters) {
    Serial.println("Rotating...");
    // convert ms delays into ticks
    // TickType_t inital_delay_ticks = pdMS_TO_TICKS(INITIAL_DELAY_MS);
    // TickType_t poll_rate_ticks = pdMS_TO_TICKS(MOTOR_ADJUSTMENT_DELAY_ROTATING_MS);

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
    Serial.println("gonna rotate bro");

    bool rotating;
    for(;;) {
        Serial.println("rotating");

        // start rotating, then delay to ensure that rotation isn't immediately canceled by tape detection
        config->motor_front_right->set_drive(MOTOR_SPEED_ROTATION, forward);
        config->motor_back_right->set_drive(MOTOR_SPEED_ROTATION, forward);
        config->motor_front_left->set_drive(MOTOR_SPEED_ROTATION, reverse);
        config->motor_back_left->set_drive(MOTOR_SPEED_ROTATION, reverse);
        vTaskDelay(5);

        // look for tape detection
        rotating = true;
        while(rotating) {
            int left_mean = get_buffer_average(*(config->reflectancePollingConfig->left_sensor_buffer));
            int right_mean = get_buffer_average(*(config->reflectancePollingConfig->right_sensor_buffer));

            Serial.println("rotating?!");

            if(right_mean > THRESHOLD_SENSOR_SINGLE && left_mean > THRESHOLD_SENSOR_SINGLE) {
                Serial.println("ending rotation");
                rotating = false;
                // Serial.println(left_mean);
                // Serial.println(right_mean);
                config->motor_front_right->stop();
                config->motor_back_right->stop();
                config->motor_front_left->stop();
                config->motor_back_left->stop();

                // // send message to TaskMaster that rotation has finished
                // Message message = ROTATION_DONE;
                // if (xQueueSend((*config->reflectancePollingConfig->xSharedQueue), &message, portMAX_DELAY) != pdPASS) {
                //     Serial.println("Failed to send to xSharedQueue");
                // }
                vTaskDelete(NULL);
            }

            vTaskDelay(5);
        }
    }
}

void setup() {
  Serial.begin(9600);
  Serial.println("Got serial!");
  
  motor_front_left = RobotMotor(MOTOR_FRONT_LEFT_FORWARD, MOTOR_FRONT_LEFT_REVERSE);
  motor_front_right = RobotMotor(MOTOR_FRONT_RIGHT_FORWARD, MOTOR_FRONT_RIGHT_REVERSE);
  motor_back_left = RobotMotor(MOTOR_BACK_LEFT_FORWARD, MOTOR_BACK_LEFT_REVERSE);
  motor_back_right = RobotMotor(MOTOR_BACK_RIGHT_FORWARD, MOTOR_BACK_RIGHT_REVERSE);

  config_reflectance = {
    &leftBuffer, &rightBuffer
  };

  config_following = {
    &motor_front_right, &motor_front_left, &motor_back_right, &motor_back_left, &config_reflectance 
  };

  config_rotate = {
    &motor_front_right, &motor_front_left, &motor_back_right, &motor_back_left, &config_reflectance 
  };

  BaseType_t xReturnedReflectance = xTaskCreate(TaskPollReflectance, "Reflectance Polling", 50, &config_reflectance, PRIORITY_REFLECTANCE_POLLING, NULL);
  // BaseType_t xReturnedFollowing = xTaskCreate(TaskFollowTape, "Tape Following", 200, &config_following, PRIORITY_FOLLOW_TAPE, NULL);
  BaseType_t xReturnedRotate = xTaskCreate(TaskRotate, "Tape Rotate", 50, &config_rotate, PRIORITY_FOLLOW_TAPE, NULL);


  // check if reflectance polling task was created
  if (xReturnedReflectance == pdPASS) {
    Serial.println("Reflectance polling task was created successfully.");
  } else
  {
    Serial.println("Reflectance polling task was not created successfully!");
  }

  // // check if tape following task was created
  // if (xReturnedFollowing == pdPASS) {
  //   Serial.println("Tape following task was created successfully.");
  // } else
  // {
  //   Serial.println("Tape following task was not created successfully!");
  // }


  // check if tape following task was created
  if (xReturnedRotate == pdPASS) {
    Serial.println("Tape following task was created successfully.");
  } else
  {
    Serial.println("Tape following task was not created successfully!");
  }


  // size_t freeHeap = xPortGetFreeHeapSize();
  // Serial.println("Free Heap: " + String(freeHeap));

  vTaskStartScheduler();
}


void loop() {
  // motor_front_left.set_drive(SPEED, forward);
  // motor_front_right.set_drive(SPEED, forward);
  // motor_back_left.set_drive(SPEED, forward);
  // motor_back_right.set_drive(SPEED, forward);
  // delay(TIME_DELAY_MOTOR);
  // motor_front_left.stop();
  // motor_front_right.stop();
  // motor_back_left.stop();
  // motor_back_right.stop();
  // delay(1000);

  // motor_front_left.set_drive(SPEED, forward);
  // motor_front_right.set_drive(SPEED, reverse);
  // motor_back_left.set_drive(SPEED, reverse);
  // motor_back_right.set_drive(SPEED, forward);
  // delay(TIME_DELAY_MOTOR * 2);
  // motor_front_left.stop();
  // motor_front_right.stop();
  // motor_back_left.stop();
  // motor_back_right.stop();
  // delay(1000);

  // motor_front_left.set_drive(SPEED, reverse);
  // motor_front_right.set_drive(SPEED, reverse);
  // motor_back_left.set_drive(SPEED, reverse);
  // motor_back_right.set_drive(SPEED, reverse);
  // delay(TIME_DELAY_MOTOR);
  // motor_front_left.stop();
  // motor_front_right.stop();
  // motor_back_left.stop();
  // motor_back_right.stop();
  // delay(1000);

  // motor_front_left.set_drive(SPEED, reverse);
  // motor_front_right.set_drive(SPEED, forward);
  // motor_back_left.set_drive(SPEED, forward);
  // motor_back_right.set_drive(SPEED, reverse);
  // delay(TIME_DELAY_MOTOR * 2);
  // motor_front_left.stop();
  // motor_front_right.stop();
  // motor_back_left.stop();
  // motor_back_right.stop();
  // delay(1000);
  }