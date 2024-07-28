#include <common/stepper_motor.h>

#define TIMER_FREQUENCY 80000 


StepperMotor_t* stepper_motor = NULL;
hw_timer_t* timer = NULL;
volatile bool stepState = false;

void IRAM_ATTR actuate_stepper_ISR() {
  stepState = !stepState; // Toggle step pin state

  stepper_motor->position += stepper_motor->direction * stepState;  // Increment position on rising edge only  
  digitalWrite(stepper_motor->stepPin, stepState);                  // HIGH if stepState == true, LOW if stepState == false
}

int checkStepperMotorData(StepperMotorCommandBuffer_t* data) {
    return data == NULL;
}

/**
 * @brief Returns the sign of the argument 
 * @returns 1 if positive, -1 if negative.
 */
int sign(int x) {
    return (x > 0) - (x < 0);
}

// This task manages the execution of a stepper motor action
void stepperMotorTask(void *pvParameters) {
    // Cast and check points
    StepperMotorCommandBuffer_t* data = (StepperMotorCommandBuffer_t*)pvParameters;
    if (checkStepperMotorData(data)) {
        log_error("Stepper motor task data has nulls!");
        vTaskDelete(NULL);
        return;
    }

    // Acquire the stepper motor's mutex (we do NOT want to have two stepper motor actions going at the same time!)
    if (xSemaphoreTake(stepper_motor->xMutex, portMAX_DELAY) == pdTRUE) {
        log_status("Starting stepper motor action...");
        stepper_motor->running = true;

        digitalWrite(stepper_motor->sleep_pin, HIGH); // Disable sleep mode
        vTaskDelay(pdMS_TO_TICKS(1)); // Driver takes maximum 1ms to wake up from sleep


        // Write direction
        stepper_motor->direction = data->direction;
        digitalWrite(stepper_motor->directionPin, data->direction == UP ? HIGH : LOW);
        
        log_message("Enabling timer...");

        timerAlarmEnable(timer); // Enable alarm
        timerAttachInterrupt(timer, &actuate_stepper_ISR, true);    // Attach ISR

        while (sign(data->new_position - stepper_motor->position) == stepper_motor->direction) { // Easy way to tell if we have reached or past the desired position
            vTaskDelay(pdMS_TO_TICKS(5)); // TODO: Add delay constant
        }

        timerAlarmDisable(timer); // Disable alarm (doesnt seem to work)
        timerDetachInterrupt(timer);    // Detach ISR
        log_message("Disabling timer...");

        digitalWrite(stepper_motor->sleep_pin, LOW);

        xSemaphoreGive(stepper_motor->xMutex); // Release the lock
        stepper_motor->running = false;
    } else {
        log_error("Couldn't acquire mutex to perform stepper motor action!");
    }

    log_status("Stepper motor action completed.");

    // Clean up 
    free(data);
    vTaskDelete(NULL);
}

void init_stepper_motor(uint8_t stepPin, uint8_t dirPin, uint8_t sleep_pin, int position, int speed) {
    stepper_motor = (StepperMotor_t*)malloc(sizeof(StepperMotor_t));
    if (stepper_motor == NULL) {
        log_error("Failed to allocate memory for stepper motor!");
        return;
    }

    // Create the mutex that we will use to ensure only one stepper motor command runs at a given time
    stepper_motor->xMutex = xSemaphoreCreateMutex();
    if (stepper_motor->xMutex == NULL) {
        log_error("Stepper motor mutex couldn't be created!");
        return;
    }

    pinMode(stepPin, OUTPUT);
    pinMode(dirPin, OUTPUT);
    pinMode(sleep_pin, OUTPUT);
    digitalWrite(sleep_pin, LOW);

    uint16_t timer_divider = (int)(TIMER_FREQUENCY / 1000);
    uint64_t alarm_time_us = (int)(1000000 / speed / 2);        // Half period in us of signal

    timer = timerBegin(STEPPER_TIMER, timer_divider, true);     // Setup stepper timer
    timerAlarmWrite(timer, alarm_time_us, true);                // Set alarm to trigger ISR every `alarm_time_us` periods

    stepper_motor->stepPin = stepPin;
    stepper_motor->directionPin = dirPin;
    stepper_motor->sleep_pin = sleep_pin;
    stepper_motor->position = position;
    stepper_motor->speed = speed;
    stepper_motor->direction = UP;

    log_status("Successfully configured stepper motor!");
}

void actuate_stepper_motor(int direction, int new_position) {
    log_status("Preparing stepper motor action...");

    // Allocate memory on the heap for StepperMotorCommandData_t
    StepperMotorCommandBuffer_t* data = (StepperMotorCommandBuffer_t*)malloc(sizeof(StepperMotorCommandBuffer_t));
    if (data == NULL) {
        log_error("Failed to allocate memory for stepper motor task data.");
        return;
    }

    if (stepper_motor == NULL) {
        log_error("Stepper motor has not been configured!");
        free(data);
        return;
    }
    
    // Populate the data structure
    data->new_position = new_position;
    data->direction = direction;
    
    // Start a task to execute the stepper motor action
    if (xTaskCreate(stepperMotorTask, "StepperTask", 1024, data, PRIORITY_STEPPER_TASK, NULL) == pdPASS) {
        log_status("Stepper task was created successfully.");
    } else {
        log_error("Stepper task was not created successfully!");
        free(data);
    }
}

bool is_running(StepperMotor_t* stepper_motor) {
    if (stepper_motor == NULL) {
        log_error("Stepper motor has not been configured!");
        return false;
    }

    return stepper_motor->running;
}
