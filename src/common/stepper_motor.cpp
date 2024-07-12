#include <common/stepper_motor.h>


StepperMotor::StepperMotor(uint8_t stepPin, uint8_t dirPin, int position = 0) {
    pinMode(stepPin, OUTPUT);
    pinMode(dirPin, OUTPUT);
    
    this->boundStepPin = stepPin;
    this->boundDirPin = dirPin;
    this->position = position;
    Serial.println("Created stepper motor!");
}

StepperMotor::StepperMotor() {
    this->boundStepPin = 0;
    this->boundDirPin = 0;
    this->position = 0;
}

void stepperMotorTask(void *pvParameters) {
    Serial.println("Starting stepper task...");
    StepperMotorCommandData_t* data = static_cast<StepperMotorCommandData_t*>(pvParameters);

    digitalWrite(PA0, data->direction);
    int numStepsTaken = 0;

    int half_period_ms = (int)(500 / data->frequency);  // Period_s -> 1 / frequency -> * 1000 -> Period_ms -> / 2 -> half period_ms
    // TickType_t delay_ticks = pdMS_TO_TICKS(half_period_ms);
    Serial.println("Half period is " + String(half_period_ms));

    while (numStepsTaken < data->numSteps) {
        digitalWrite(data->stepPin, HIGH);
        vTaskDelay(5);
        digitalWrite(data->stepPin, LOW);
        vTaskDelay(5);

        numStepsTaken++;
    }

    Serial.println("Ending stepper task...");

    free(data);
    vTaskDelete(NULL);
}

void StepperMotor::step(int direction, int numSteps) {
    // Allocate memory on the heap for StepperMotorCommandData_t
    Serial.println("Trying bro...");
    StepperMotorCommandData_t* data = (StepperMotorCommandData_t*)malloc(sizeof(StepperMotorCommandData_t));
    if (data == nullptr) {
        Serial.println("Failed to allocate memory for stepper motor task data.");
        return;
    }
    
    // Populate the data structure
    data->numSteps = numSteps;
    data->frequency = TEST_FREQUENCY;
    data->stepPin = this->boundStepPin;
    data->dirPin = this->boundDirPin;
    data->direction = direction;
    
    BaseType_t xReturned = xTaskCreate(stepperMotorTask, "StepperTask", 200, data, PRIORITY_STEPPER_TASK, NULL);
    if (xReturned == pdPASS) {
        Serial.println("Stepper task was created successfully.");
    } else
    {
        Serial.println("Stepper task was not created successfully!");
        Serial.println(xReturned);
        free(data);
    }
  }