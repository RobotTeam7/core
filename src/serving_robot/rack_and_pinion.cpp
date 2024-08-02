#include <serving_robot/rack_and_pinion.h>


#define RACK_AND_PINION_SPEED 18000

RobotMotor_t* rack_and_pinion;
LimitSwitch_t* claw_limit_switch;
LimitSwitch_t* forklift_limit_switch;

TaskHandle_t xClawHandle = NULL;
TaskHandle_t xForkliftHandle = NULL;
int position;


void IRAM_ATTR claw_isr() {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    Serial.println("Claw!");

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
    Serial.println("Forklift!");

    if (xForkliftHandle != NULL) {
        vTaskNotifyGiveFromISR(xForkliftHandle, &xHigherPriorityTaskWoken);
    }

    // Request a context switch if giving the notification unblocked a higher priority task
    if (xHigherPriorityTaskWoken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

void init_rack_and_pinion(uint8_t fowards_pin, uint8_t backwards_pin, int initial_position, uint8_t claw_limit_switch_pin, uint8_t forklift_limit_switch_pin) {
    rack_and_pinion = instantiate_robot_motor(fowards_pin, backwards_pin, MOTOR_TIMER_1);

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