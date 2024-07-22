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
#include <motion/motion.h>

#include <common/stepper_motor.h>


RobotMotor_t* motor_front_left;
RobotMotor_t* motor_front_right;
RobotMotor_t* motor_back_left;
RobotMotor_t* motor_back_right;

DualTapeSensor_t* frontTapeSensor;
DualTapeSensor_t* backTapeSensor;
DualTapeSensor_t* wingSensor;

RobotMotorData_t robotMotors;
NavigationData_t config_following;
DockingData_t config_docking;

StepperMotor_t* stepper_motor;

QueueHandle_t xSharedQueue = xQueueCreate(10, sizeof(StatusMessage_t));
QueueHandle_t uart_msg_queue = xQueueCreate(10, sizeof(Packet_t));

void TaskMaster(void *pvParameters);
void begin_rotating();
void begin_following();
void begin_station_tracking();
void begin_docking();

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
                        state.current_action = ActionType_t::SPIN;
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

    initialize_uart();
    begin_uart_read(&uart_msg_queue);

    xTaskCreate(uart_msg_handler, "uart_msg_handler", 2048, NULL, 1, NULL);

    motor_front_left = instantiate_robot_motor(MOTOR_FRONT_LEFT_FORWARD, MOTOR_FRONT_LEFT_REVERSE);
    motor_front_right = instantiate_robot_motor(MOTOR_FRONT_RIGHT_FORWARD, MOTOR_FRONT_RIGHT_REVERSE);
    motor_back_left = instantiate_robot_motor(MOTOR_BACK_LEFT_FORWARD, MOTOR_BACK_LEFT_REVERSE);
    motor_back_right = instantiate_robot_motor(MOTOR_BACK_RIGHT_FORWARD, MOTOR_BACK_RIGHT_REVERSE);

    frontTapeSensor = instantiate_tape_sensor(FRONT_TAPE_SENSOR_LEFT, FRONT_TAPE_SENSOR_RIGHT);
    backTapeSensor = instantiate_tape_sensor(BACK_TAPE_SENSOR_LEFT, BACK_TAPE_SENSOR_RIGHT);
    wingSensor = instantiate_tape_sensor(LEFT_WING_TAPE_SENSOR, RIGHT_WING_TAPE_SENSOR);

    stepper_motor = instantiate_stepper_motor(STEPPER_STEP, STEPPER_DIR, 0, 100);

    robotMotors = { motor_front_right, motor_front_left, motor_back_right, motor_back_left };
    config_following = { frontTapeSensor, backTapeSensor, &xSharedQueue };
    config_docking = { wingSensor, &xSharedQueue };

    // // check if driving task was created
    if (xTaskCreate(TaskDrive, "DrivingTask", 2048, &robotMotors, PRIORITY_DRIVE_UPDATE, &xDriveHandle) == pdPASS) {
        log_status("Driving task was created successfully."); 
    } else {
        log_error("Driving task was not created successfully!");
    }

    // delay(100);

    // check if task master was created
    if (xTaskCreate(TaskMaster, "MasterTask", 2048, NULL, 3, &xMasterHandle) == pdPASS) {
        log_status("Master task was created successfully.");
    } else {
        log_error("Master task was not created successfully!");
    }
}

void loop()
{
    // monitorStackUsage(&xHandleRotating, &xReflectanceHandle, &xHandleFollowing, &xMasterHandle, &xStationTrackingHandle); // Monitor stack usage periodically
    // delay(2000);

    if(state.current_action == GOTO_STATION) {
        Serial.print("going to station: ");
        Serial.println(state.desired_station);
    }else if(state.current_action == SPIN) {
        Serial.println("rotating");
    }else if(state.current_action == IDLE) {
        Serial.println("idling");
    }
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
            case SPIN:
                // Begin rotating and wait for a message that we see the tape
                begin_rotating();
                if (xQueueReceive(xSharedQueue, &receivedMessage, portMAX_DELAY) == pdPASS) {  // we will not stop rotating if we get an abort command
                    if (receivedMessage == ROTATION_DONE) {
                        log_status("Completed rotation: found tape!");         
                        
                        vTaskDelete(xHandleRotating);
                        xHandleRotating = NULL;

                        vTaskDelay(pdMS_TO_TICKS(ROTATE_INTO_TAPE_FOLLOW_DELAY));

                        send_uart_message(COMPLETED);
                        log_status("Ending rotation...");
                        if (state.current_action == SPIN) {
                            state.current_action = IDLE;
                        }
                    }
                }
                vTaskDelay(2000);
                break;

            case GOTO_STATION:
            {   
                int station_difference = state.desired_station - state.last_station;
                if(station_difference == 0) {
                    Serial.println("already at desired station!");
                    send_uart_message(COMPLETED);
                    state.current_action == IDLE;
                }
                
                // check if station difference and direction have opposite sign
                // if they do flip the sign direction
                if(station_difference * state.direction < 0) {
                    state.direction = -state.direction;
                }

                // Start Driving
                state.drive_state = DRIVE;
                state.drive_speed = MOTOR_SPEED_FOLLOWING;

                // Activate tape follow 
                log_status("Tape following!");
                begin_following();

                // Delay in case we are already on tape
                vTaskDelay(pdMS_TO_TICKS(TAPE_TRACKING_INTITAL_DELAY));

                log_status("Station tracking!");
                begin_station_tracking();
                int docking = 0;

                while (state.current_action == GOTO_STATION) {
                    if ((state.last_station == state.desired_station) && !docking) {
                        log_status("Beginning docking...!");

                        vTaskDelete(xStationTrackingHandle);
                        xStationTrackingHandle = NULL;

                        vTaskDelete(xHandleFollowing);
                        xHandleFollowing = NULL;

                        state.drive_state = STOP;
                        vTaskDelay(pdMS_TO_TICKS(2000));

                        state.drive_state = DRIVE;
                        state.direction = -state.direction; // Invert direction
                        state.drive_speed = MOTOR_SPEED_DOCKING;
                        state.yaw = 0;
                        begin_docking();
                        docking = 1;
                    }

                    if (docking) {
                        if (xQueueReceive(xSharedQueue, &receivedMessage, portMAX_DELAY) == pdPASS) {
                            if (receivedMessage == REACHED_POSITION) {
                                log_status("Reached position: found tape!");     

                                vTaskDelete(xDockingHandle);
                                xDockingHandle = NULL;

                                // vTaskDelay(pdMS_TO_TICKS(ROTATE_INTO_TAPE_FOLLOW_DELAY));

                                send_uart_message(COMPLETED);
                                log_status("Ending goto station...");
                                if (state.current_action == GOTO_STATION) {
                                    state.current_action = IDLE;
                                    state.drive_state = STOP;
                                    state.direction = FORWARD_DRIVE;
                                }
                                break;
                            }
                        }
                    }
                    vTaskDelay(10 / portTICK_PERIOD_MS);

                    
                }
                break;
            }

            case IDLE:
                // don't move while idling
                state.drive_state = DriveState_t::STOP;
                log_status("Idling...");

                while (state.current_action == IDLE) {
                    vTaskDelay(20 / portTICK_PERIOD_MS);
                }
        }
    }
}

void begin_rotating() {
    // check if tape following task was created
    if (xTaskCreate(TaskRotate, "TapeRotate", 2048, &config_following, PRIORITY_ROTATE, &xHandleRotating) == pdPASS) {
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
    if (xTaskCreate(TaskStationTracking, "Station_Tracking", 4096, wingSensor, PRIORITY_STATION_TRACKING, &xStationTrackingHandle) == pdPASS) {
        log_status("Station tracking task was created successfully.");
    } else {
        log_error("Station tracking task was not created successfully!");
    }
}

void begin_docking() {
    // check if station tracking task was created
    if (xTaskCreate(TaskDocking, "Station_Tracking", 4096, &config_docking, PRIORITY_STATION_TRACKING, &xDockingHandle) == pdPASS) {
        log_status("Docking task was created successfully.");
    } else {
        log_error("Docking task was not created successfully!");
    }
}
