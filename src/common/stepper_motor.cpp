#include <common/stepper_motor.h>


int checkStepperMotorData(StepperMotorCommandBuffer_t* data) {
    return data == NULL || data->stepperMotor == NULL;
}

/**
 * @brief Evaluate the time, in ticks, that it will require to complete the specified stepper action
 * @param data Must be already checked to be non-null
 */
TickType_t get_stepper_action_duration(StepperMotorCommandBuffer_t* data) {
    int delay = (int)(data->numSteps / data->stepperMotor->speed) * 1000;
    return pdMS_TO_TICKS(delay);
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
    if (xSemaphoreTake(data->stepperMotor->xMutex, portMAX_DELAY) == pdTRUE) {
        log_status("Starting stepper motor action...");

        digitalWrite(data->stepperMotor->sleep_pin, HIGH); // Disable sleep mode
        vTaskDelay(pdMS_TO_TICKS(1)); // Driver takes maximum 1ms to wake up from sleep

        // Write direction
        digitalWrite(data->stepperMotor->directionPin, data->direction == UP ? HIGH : LOW);

        int num_steps = 0;
        
        // Perform action
        while (num_steps < data->numSteps) {
            digitalWrite(data->stepperMotor->stepPin, HIGH);
            vTaskDelay(pdMS_TO_TICKS(1));
            digitalWrite(data->stepperMotor->stepPin, LOW);
            vTaskDelay(pdMS_TO_TICKS(1));
            num_steps++;
        }

        // Mutate stepper motor position to update position after the action
        data->stepperMotor->position += data->numSteps * data->direction;        

        digitalWrite(data->stepperMotor->sleep_pin, LOW);
        xSemaphoreGive(data->stepperMotor->xMutex); // We need to release the lock
    } else {
        log_error("Couldn't acquire mutex to perform stepper motor action!");
    }

    log_status("Stepper motor action completed.");

    // Clean up 
    free(data);
    vTaskDelete(NULL);
}

/**
 * @brief Instantiate a stepper motor bound to `stepPin` and `dirPin`.
 * @returns a pointer to a StepperMotor_t allocated on the heap with malloc(). Returns instantiation if creation failed.
 */
StepperMotor_t* instantiate_stepper_motor(uint8_t stepPin, uint8_t dirPin, uint8_t sleep_pin, int position, int speed) {
    StepperMotor_t* stepperMotor = (StepperMotor_t*)malloc(sizeof(StepperMotor_t));
    if (stepperMotor == NULL) {
        log_error("Failed to allocate memory for stepper motor!");
        return NULL;
    }

    // Create the mutex that we will use to ensure only one stepper motor command runs at a given time
    stepperMotor->xMutex = xSemaphoreCreateMutex();
    if (stepperMotor->xMutex == NULL) {
        log_error("Stepper motor mutex couldn't be created!");
        return NULL;
    }

    pinMode(stepPin, OUTPUT);
    pinMode(dirPin, OUTPUT);
    pinMode(sleep_pin, OUTPUT);
    digitalWrite(sleep_pin, LOW);

    stepperMotor->stepPin = stepPin;
    stepperMotor->directionPin = dirPin;
    stepperMotor->sleep_pin = sleep_pin;
    stepperMotor->position = position;
    stepperMotor->speed = speed;

    log_status("Instantiated stepper motor!");
    
    return stepperMotor;
}

void actuate_stepper_motor(StepperMotor_t* stepperMotor, int direction, int numSteps) {
    log_status("Preparing stepper motor action...");

    // Allocate memory on the heap for StepperMotorCommandData_t
    StepperMotorCommandBuffer_t* data = (StepperMotorCommandBuffer_t*)malloc(sizeof(StepperMotorCommandBuffer_t));
    if (data == NULL) {
        log_error("Failed to allocate memory for stepper motor task data.");
        return;
    }
    
    // Populate the data structure
    data->numSteps = numSteps;
    data->direction = direction;
    data->stepperMotor = stepperMotor;
    
    // Start a task to execute the stepper motor action
    if (xTaskCreate(stepperMotorTask, "StepperTask", 1024, data, PRIORITY_STEPPER_TASK, NULL) == pdPASS) {
        log_status("Stepper task was created successfully.");
    } else {
        log_error("Stepper task was not created successfully!");
        free(data);
    }
}
