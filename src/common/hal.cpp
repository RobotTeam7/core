#include <common/hal.h>


LimitSwitch_t* instantiate_limit_switch(uint8_t interrupt_pin, ISR_Function isr_function) {
    LimitSwitch_t* new_switch = (LimitSwitch_t*)malloc(sizeof(LimitSwitch_t));
    if (new_switch == NULL) {
        log_error("Couldn't instantiate limit switch!");
        return NULL;
    }

    new_switch->interrupt_pin = interrupt_pin;

    pinMode(interrupt_pin, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(interrupt_pin), isr_function, RISING);

    log_status("Created new limit switch!");

    return new_switch;
}

RobotMotor_t* rack_and_pinion;
LimitSwitch_t* claw_limit_switch;
LimitSwitch_t* forklift_limit_switch;

TaskHandle_t xClawHandle = NULL;
TaskHandle_t xForkliftHandle = NULL;
int position;


void IRAM_ATTR claw_isr() {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (xClawHandle != NULL) {
        vTaskNotifyGiveFromISR(xClawHandle, &xHigherPriorityTaskWoken);
    }

    // Request a context switch if giving the notification unblocked a higher priority task
    if (xHigherPriorityTaskWoken == pdTRUE) {
        portYIELD_FROM_ISR();
    }

}

void IRAM_ATTR forklift_isr() {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (xForkliftHandle != NULL) {
        vTaskNotifyGiveFromISR(xForkliftHandle, &xHigherPriorityTaskWoken);
    }

    // Request a context switch if giving the notification unblocked a higher priority task
    if (xHigherPriorityTaskWoken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

void init_rack_and_pinion(uint8_t fowards_pin, uint8_t backwards_pin, RackAndPinionPosition_t initial_position, uint8_t claw_limit_switch_pin, uint8_t forklift_limit_switch_pin) {
    rack_and_pinion = instantiate_robot_motor(fowards_pin, backwards_pin, timer_2);

    position = initial_position;

    claw_limit_switch = instantiate_limit_switch(claw_limit_switch_pin, claw_isr);
    forklift_limit_switch = instantiate_limit_switch(forklift_limit_switch_pin, forklift_isr);
}

static void task_actuate_claw(void* pvParameters) {
    motor_set_drive(rack_and_pinion, RACK_AND_PINION_SPEED);

    uint32_t ulNotificationValue;
    while (1) {
        xTaskNotifyWait(0x00, 0xFFFFFFFF, &ulNotificationValue, portMAX_DELAY);

        motor_set_drive(rack_and_pinion, 0);
        position = 1;

        log_status("Stopping motor!");

        vTaskDelete(NULL);
        xClawHandle = NULL;

        log_status("Claw has been moved forwards!");
    }
}

static void task_actuate_forklift(void* pvParameters) {
    log_status("Moving forklift forwards...");
    motor_set_drive(rack_and_pinion, -RACK_AND_PINION_SPEED);

    uint32_t ulNotificationValue;
    while (1) {
        xTaskNotifyWait(0x00, 0xFFFFFFFF, &ulNotificationValue, portMAX_DELAY);

        motor_set_drive(rack_and_pinion, 0);
        position = -1;

        vTaskDelete(NULL);
        xForkliftHandle = NULL;

        log_status("Forklift has been moved forwards!");
    }
}

void actuate_claw_forwards() {
    if (position == 1) {
        log_status("Claw is already forwards!");
        return;
    }

    // check if claw actuation task was created
    if (xTaskCreate(task_actuate_claw, "Actuate_Claw", 2048, NULL, 4, &xClawHandle) == pdPASS) {
        log_status("Claw actuation task was created successfully.");
    } else {
        log_error("Claw actuation task was not created successfully!");
    }
}

void actuate_forklift_forwards() {
    if (position == -1) {
        log_status("Forklift is already forwards!");
        return;
    }
    // check if forklift actuation task was created
    if (xTaskCreate(task_actuate_forklift, "Actuate_Forklift", 2048, NULL, 4, &xForkliftHandle) == pdPASS) {
        log_status("Forklift actuation task was created successfully.");
    } else {
        log_error("Forklift actuation task was not created successfully!");
    }
}

void set_rack_zero() {
    log_status("setting rack position to zero");
    position = 0;
}

TapeSensor_t* instantiate_tape_sensor(uint8_t pin) {
    TapeSensor_t* tapeSensor = (TapeSensor_t*)malloc(sizeof(TapeSensor_t));
    if (NULL == tapeSensor) {
        log_error("Couldn't allocate memory for tape sensor!");
        return NULL;
    }

    pinMode(pin, INPUT);

    tapeSensor->pin = pin;
    tapeSensor->value = 1111; // An obviously false value to debug failed sensor readings

    log_status("Created tape sensor!");

    return tapeSensor;
}

void read_tape_sensor(TapeSensor_t* tapeSensor) {
    tapeSensor->value = analogRead(tapeSensor->pin);
}

RobotMotor_t* instantiate_robot_motor(uint8_t forward_pin, uint8_t reverse_pin, MotorTimers_t timer) {
    RobotMotor_t* robotMotor = (RobotMotor_t*)malloc(sizeof(RobotMotor_t));
    if (robotMotor == NULL) {
        log_error("Failed to allocate memory for robot motor!");
        return NULL;
    }

    bind_pwm(forward_pin, LEDC_PWM_FREQUENCY, (ledc_timer_t)timer, LEDC_TIMER_16_BIT);
    bind_pwm(reverse_pin, LEDC_PWM_FREQUENCY, (ledc_timer_t)timer, LEDC_TIMER_16_BIT);

    robotMotor->forward_pin = forward_pin;
    robotMotor->reverse_pin = reverse_pin;
    robotMotor->current_state = FORWARD_DRIVE;
    robotMotor->current_drive = 0;

    log_status("Instantiated motor!");
    
    return robotMotor;
}

uint8_t get_output_pin(RobotMotor_t* robotMotor, MotorDirection_t direction) {
    return direction == FORWARD_DRIVE ? robotMotor->forward_pin : robotMotor->reverse_pin;
}

void motor_set_drive(RobotMotor_t* robot_motor, int16_t drive_value) {
    // Interpret positive values of `drive_value` as forward drive, and negative values as reverse drive
    MotorDirection_t target_direction = (MotorDirection_t)sign(drive_value);
    
    uint8_t old_pin = get_output_pin(robot_motor, robot_motor->current_state);  // Currently active control pin
    uint8_t new_pin = get_output_pin(robot_motor, target_direction);            // Future active control pin
    
    // We do not want to try to drive in both directions at once 
    if (new_pin != old_pin) {
        set_pwm(old_pin, 0);
    }

    // This value can be guaranteed to be positive, as `sign(drive_value) == sign(target_direction)`
    uint32_t target_drive = (uint32_t)(drive_value * (int)target_direction);

    set_pwm(new_pin, target_drive);

    robot_motor->current_state = target_direction;
    robot_motor->current_drive = target_drive;
}

void motor_stop(RobotMotor_t* robot_motor) {
    motor_set_drive(robot_motor, 0);
}

ServoMotor_t* instantiate_servo_motor(uint8_t control_pin, double max_duty_cycle, double min_duty_cycle) {
    ServoMotor_t* servoMotor = (ServoMotor_t*)malloc(sizeof(ServoMotor_t));
    if (servoMotor == NULL) {
        log_error("Failed to allocate memory for servo motor!");
        return NULL;
    }

    int closed_position = max_duty_cycle * UINT16_MAX;
    servoMotor->position = closed_position;
    servoMotor->control_pin = control_pin;
    servoMotor->max_duty_cycle = max_duty_cycle;
    servoMotor->min_duty_cycle = min_duty_cycle;

    bind_pwm(servoMotor->control_pin, SERVO_MOTOR_CONTROL_FREQUENCY, MOTOR_TIMER_0);
    set_pwm(servoMotor->control_pin, servoMotor->position);    

    log_status("Created servo motor!");

    return servoMotor;
}

int set_servo_position(ServoMotor_t* servoMotor, uint16_t newPosition) {
    Serial.println("setting servo to power: " + String(newPosition));

    if (servoMotor->position != newPosition) {
        servoMotor->position = newPosition;
        set_pwm(servoMotor->control_pin, servoMotor->position);
        return 1;
    }

    return 0;
}

// sets the position of the claw based on a percentage value
// 0 is closed
// 1 is open
int set_servo_position_percentage(ServoMotor_t* servoMotor, int percentange) {
    if (percentange < 0 || percentange > 100) {
        log_error("Invalid percentage. Must be between 0 and 100!");
        return 0;
    }

    int range = (servoMotor->max_duty_cycle - servoMotor->min_duty_cycle) * UINT16_MAX;
    int min_power = servoMotor->min_duty_cycle * UINT16_MAX;
    int power = min_power + range * ( (float) percentange / 100.0 ) ;

    return set_servo_position(servoMotor, power);
}

StepperMotor_t* stepper_motor = NULL;
hw_timer_t* timer = NULL;
volatile bool step_state = false;

void IRAM_ATTR actuate_stepper_ISR() {
  step_state = !step_state; // Toggle step pin state

  stepper_motor->position += stepper_motor->direction * step_state;   // Increment position on rising edge only  
  digitalWrite(stepper_motor->step_pin, step_state);                  // HIGH if stepState == true, LOW if stepState == false
}

// This task manages the execution of a stepper motor action
static void stepper_motor_task(void *pvParameters) {
    StepperMotorCommandBuffer_t* data = (StepperMotorCommandBuffer_t*)pvParameters;
    if (data == NULL) {
        log_error("Stepper motor task data is null!");
        vTaskDelete(NULL);
        return;
    }

    // Acquire the stepper motor's mutex (we do NOT want to have two stepper motor actions going at the same time!)
    if (xSemaphoreTake(stepper_motor->xMutex, portMAX_DELAY) == pdTRUE) {
        log_status("Starting stepper motor action...");
        stepper_motor->running = true;

        digitalWrite(stepper_motor->sleep_pin, HIGH);   // Disable sleep mode
        vTaskDelay(pdMS_TO_TICKS(1));                   // Driver takes maximum 1ms to wake up from sleep

        // Write direction
        stepper_motor->direction = data->direction;
        digitalWrite(stepper_motor->direction_pin, data->direction == UP ? HIGH : LOW);
        
        timerAlarmEnable(timer);                                    // Enable alarm
        timerAttachInterrupt(timer, &actuate_stepper_ISR, true);    // Attach ISR

        while (sign(data->new_position - stepper_motor->position) == stepper_motor->direction) { // Easy way to tell if we have reached or past the desired position
            vTaskDelay(pdMS_TO_TICKS(5));
        }

        timerAlarmDisable(timer);       // Disable alarm (doesnt seem to have any effect)
        timerDetachInterrupt(timer);    // Detach ISR

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

void init_stepper_motor(uint8_t step_pin, uint8_t direction_pin, uint8_t sleep_pin, int position, int speed) {
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

    pinMode(step_pin, OUTPUT);
    pinMode(direction_pin, OUTPUT);
    pinMode(sleep_pin, OUTPUT);
    digitalWrite(sleep_pin, LOW);

    uint16_t timer_divider = (int)(TIMER_FREQUENCY / 1000);
    uint64_t alarm_time_us = (int)(1000000 / speed / 2);        // Half period in us of signal

    timer = timerBegin(STEPPER_TIMER, timer_divider, true);     // Setup stepper timer
    timerAlarmWrite(timer, alarm_time_us, true);                // Set alarm to trigger ISR every `alarm_time_us` periods

    stepper_motor->step_pin = step_pin;
    stepper_motor->direction_pin = direction_pin;
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
    if (xTaskCreate(stepper_motor_task, "StepperTask", 1024, data, PRIORITY_STEPPER_TASK, NULL) == pdPASS) {
        log_status("Stepper motor task was created successfully.");
    } else {
        log_error("Stepper motor task was not created successfully!");
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
