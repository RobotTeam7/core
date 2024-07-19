#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <common/robot_motor.h>
#include <common/stepper_motor.h>
#include <common/reflectance_sensor.h>

#include <communication/uart.h>
#include <communication/decode.h>

#include <motion/constants.h>
#include <motion/tasks.h>
#include <motion/utils.h>
#include <motion/state.h>


RobotMotor_t* motor_front_left;
RobotMotor_t* motor_front_right;
RobotMotor_t* motor_back_left;
RobotMotor_t* motor_back_right;

DualTapeSensor_t* frontTapeSensor;
DualTapeSensor_t* backTapeSensor;
MonoTapeSensor_t* left_wing_tape_sensor;
MonoTapeSensor_t* right_wing_tape_sensor;

RobotMotorData_t robotMotors;
TapeAwarenessData_t config_following;
RobotControlData_t config_rotate;

TaskHandle_t xHandleRotating = NULL;
TaskHandle_t xReflectanceHandle = NULL;
TaskHandle_t xHandleFollowing = NULL;
TaskHandle_t xMasterHandle = NULL;
TaskHandle_t xStationTrackingHandle = NULL;

QueueHandle_t xSharedQueue = xQueueCreate(10, sizeof(StatusMessage_t));
QueueHandle_t uart_msg_queue = xQueueCreate(10, sizeof(Packet_t));
QueueHandle_t task_queue = xQueueCreate(10, sizeof(Packet_t));

void TaskMaster(void *pvParameters);
void begin_rotating();
void begin_following();
void begin_station_tracking();


void uart_msg_handler(void *parameter) {
    log_status("Started message handler");
    
    while (1) {
        Packet_t new_packet;
        if (xQueueReceive(uart_msg_queue, &new_packet, portMAX_DELAY)) {
            Serial.println("Received packet...");
            Serial.println(new_packet.command);
            Serial.println(new_packet.value);
            Serial.println("Received.");

            // If the command was not to abort, and we aren't idle, reject switching tasks.
            if (new_packet.command != ABORT && state.current_action != IDLE) {
                log_error("Occupied! Not switching tasks.");
                send_uart_message(OCCUPIED);
            } else {
                // If we aren't busy, set our new task based on the command
                switch (new_packet.command) {
                    // If we should abort, end our current task.
                    case ABORT:
                        state.current_action = IDLE;
                        break;

                    case GOTO:
                        state.desired_station = new_packet.value;
                        state.current_action = GOTO_STATION;
                        break;

                    case DO_SPIN:
                        state.current_action = ROTATE;
                        break;
                }
                send_uart_message(ACCEPTED);
            }
        }
        vTaskDelay(10 / portTICK_PERIOD_MS); // Small delay to yield    
    }
}


void setup() {
    Serial.begin(115200);

    // initialize_uart();
    // begin_uart_read(&uart_msg_queue);

    // xTaskCreate(uart_msg_handler, "uart_msg_handler", 2048, NULL, 1, NULL);

    motor_front_left = instantiate_robot_motor(MOTOR_FRONT_LEFT_FORWARD, MOTOR_FRONT_LEFT_REVERSE);
    motor_front_right = instantiate_robot_motor(MOTOR_FRONT_RIGHT_FORWARD, MOTOR_FRONT_RIGHT_REVERSE);
    motor_back_left = instantiate_robot_motor(MOTOR_BACK_LEFT_FORWARD, MOTOR_BACK_LEFT_REVERSE);
    motor_back_right = instantiate_robot_motor(MOTOR_BACK_RIGHT_FORWARD, MOTOR_BACK_RIGHT_REVERSE);

    frontTapeSensor = instantiate_tape_sensor(FRONT_TAPE_SENSOR_LEFT, FRONT_TAPE_SENSOR_RIGHT);
    backTapeSensor = instantiate_tape_sensor(BACK_TAPE_SENSOR_LEFT, BACK_TAPE_SENSOR_RIGHT);
    left_wing_tape_sensor = instantiate_tape_sensor(LEFT_WING_TAPE_SENSOR);
    right_wing_tape_sensor = instantiate_tape_sensor(RIGHT_WING_TAPE_SENSOR);

    robotMotors = { motor_front_right, motor_front_left, motor_back_right, motor_back_left };
    config_following = { &robotMotors, frontTapeSensor, &xSharedQueue };
    config_rotate = { &config_following, &xSharedQueue };

    // check if reflectance polling task was created
    if (xTaskCreate(TaskPollReflectance, "ReflectancePolling", 2048, frontTapeSensor, PRIORITY_REFLECTANCE_POLLING, &xReflectanceHandle) == pdPASS) {
        log_status("Reflectance polling task was created successfully."); 
    } else {
        log_error("Reflectance polling task was not created successfully!");
    }

    delay(100);

    // check if task master was created
    if (xTaskCreate(TaskMaster, "MasterTask", 2048, NULL, 3, &xMasterHandle) == pdPASS) {
        log_status("Master task was created successfully.");
    } else {
        log_error("Master task was not created successfully!");
    }
}

void loop()
{
    monitorStackUsage(&xHandleRotating, &xReflectanceHandle, &xHandleFollowing, &xMasterHandle, &xStationTrackingHandle); // Monitor stack usage periodically
    delay(2000);
}

void TaskMaster(void *pvParameters)
{
    log_status("Beginning master task...");
    // state.current_action = IDLE;
    send_uart_message(READY);

    while (1) {
        StatusMessage_t receivedMessage;
        switch (state.current_action) {
            Serial.println("Current task " + String(state.current_action));

            case ROTATE:
                // Begin rotating and wait for a message that we see the tape
                begin_rotating();
                if (xQueueReceive(xSharedQueue, &receivedMessage, portMAX_DELAY) == pdPASS) {  // we will not stop rotating if we get an abort command
                    if (receivedMessage == ROTATION_DONE) {
                        log_status("Completed rotation: found tape!");                        
                        vTaskDelete(xHandleRotating);

                        vTaskDelay(pdMS_TO_TICKS(ROTATE_INTO_TAPE_FOLLOW_DELAY));

                        send_uart_message(COMPLETED);
                        log_status("Ending rotation...");
                        if (state.current_action == ROTATE) {
                            state.current_action = IDLE;
                        }
                    }
                }
                vTaskDelay(2000);
                send_uart_message(COMPLETED);
                log_status("Ending rotation...");
                if (state.current_action == ROTATE) {
                    state.current_action = IDLE;
                }
                break;

            case GOTO_STATION:
                // Activate tape follow 
                log_status("Tape following!");
                begin_following();

                log_status("Station tracking!");
                begin_station_tracking();

                while (state.current_action == GOTO_STATION) {
                    if (state.last_station == state.desired_station) {
                        log_status("Arrived at station!");
                        vTaskDelete(xStationTrackingHandle);
                        vTaskDelete(xHandleFollowing);

                        stop_robot_motors(&robotMotors);

                        send_uart_message(COMPLETED);
                        log_status("Ending goto station...");
                        if (state.current_action == GOTO_STATION) {
                            state.current_action = IDLE;
                        }
                        break;
                    }
                    vTaskDelay(1000 / portTICK_PERIOD_MS);
                    send_uart_message(COMPLETED);
                    log_status("Ending goto station...");
                    if (state.current_action == GOTO_STATION) {
                        state.current_action = IDLE;
                    }
                    break;
                }
                break;

            case IDLE:
                log_status("Idling...");
                while (state.current_action == IDLE) {
                    vTaskDelay(20 / portTICK_PERIOD_MS);
                }
        }
    }
}

void begin_rotating() {
    // check if tape following task was created
    if (xTaskCreate(TaskRotate, "TapeRotate", 2048, &config_rotate, PRIORITY_ROTATE, &xHandleRotating) == pdPASS) {
        log_status("Rotate task was created successfully.");
    } else {
        log_error("Rotate task was not created successfully!");
    }
}

void begin_following() {
    // check if tape following task was created
    if (xTaskCreate(TaskFollowTape, "Tape Following", 1024, &config_following, PRIORITY_FOLLOW_TAPE, &xHandleFollowing) == pdPASS) {
        log_status("Tape following task was created successfully.");
    } else {
        log_error("Tape following task was not created successfully!");
    }
}

void begin_station_tracking() {
    // check if station tracking task was created
    if (xTaskCreate(TaskStationTracking, "Station_Tracking", 4096, right_wing_tape_sensor, PRIORITY_STATION_TRACKING, &xStationTrackingHandle) == pdPASS) {
        log_status("Station tracking task was created successfully.");
    } else {
        log_error("Station tracking task was not created successfully!");
    }
}
