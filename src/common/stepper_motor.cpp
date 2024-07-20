#include <common/stepper_motor.h>


int checkStepperMotorData(StepperMotorCommandBuffer_t* data) {
    return data == NULL || data->stepperMotor == NULL;
}

// This task manages the execution of a stepper motor action
void stepperMotorTask(void *pvParameters) {
    log_status("Starting stepper motor action...");

    // Cast and check points
    StepperMotorCommandBuffer_t* data = (StepperMotorCommandBuffer_t*)pvParameters;
    if (checkStepperMotorData(data)) {
        log_error("Stepper motor task data has nulls!");
        vTaskDelete(NULL);
        return;
    }

    // Write direction
    digitalWrite(data->stepperMotor->directionPin, data->direction == UP ? HIGH : LOW);
    int numStepsTaken = 0;

    // Calculate timings
    int half_period_ms = (int)(1 / data->frequency * 1000 / 2); // First, convert to period in seconds, then convert to ms, then get the half.
    TickType_t delay_ticks = pdMS_TO_TICKS(half_period_ms);
    Serial.println(delay_ticks);
    log_message("Stepper motor stepping...");

    // Perform action
    while (numStepsTaken < data->numSteps) {
        digitalWrite(data->stepperMotor->stepPin, HIGH);
        vTaskDelay(delay_ticks);
        digitalWrite(data->stepperMotor->stepPin, LOW);
        vTaskDelay(delay_ticks);

        numStepsTaken++;
    }
    
    // Mutate stepper motor position to update position after the action
    data->stepperMotor->position += numStepsTaken;

    log_status("Stepper motor action completed.");

    // Clean up 
    free(data);
    vTaskDelete(NULL);
}

/**
 * @brief Instantiate a stepper motor bound to `stepPin` and `dirPin`.
 * @returns a pointer to a StepperMotor_t allocated on the heap with malloc(). Returns instantiation if creation failed.
 */
StepperMotor_t* instantiateStepperMotor(uint8_t stepPin, uint8_t dirPin, int position) {
    StepperMotor_t* stepperMotor = (StepperMotor_t*)malloc(sizeof(StepperMotor_t));
    if (stepperMotor == NULL) {
        log_error("Failed to allocate memory for stepper motor!");
        return NULL;
    }

    pinMode(stepPin, OUTPUT);
    pinMode(dirPin, OUTPUT);
    
    stepperMotor->stepPin = stepPin;
    stepperMotor->directionPin = dirPin;
    stepperMotor->position = position;

    log_status("Instantiated stepper motor!");
    
    return stepperMotor;
}

void actuateStepperMotor(StepperMotor_t* stepperMotor, int direction, int numSteps, uint16_t motorFrequency) {
    log_status("Preparing stepper motor action...");

    // Allocate memory on the heap for StepperMotorCommandData_t
    StepperMotorCommandBuffer_t* data = (StepperMotorCommandBuffer_t*)malloc(sizeof(StepperMotorCommandBuffer_t));
    if (data == NULL) {
        log_error("Failed to allocate memory for stepper motor task data.");
        return;
    }
    
    // Populate the data structure
    data->numSteps = numSteps;
    data->frequency = motorFrequency;
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
