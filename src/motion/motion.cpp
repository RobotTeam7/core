#include <common/hal.h>
#include <communication/uart.h>

#include <motion/constants.h>
#include <motion/tasks.h>
#include <motion/state.h>
#include <motion/drive.h>


RobotMotor_t* motor_front_left;
RobotMotor_t* motor_front_right;
RobotMotor_t* motor_back_left;
RobotMotor_t* motor_back_right;

TapeSensor_t* frontTapeSensor;
TapeSensor_t* backTapeSensor;
TapeSensor_t* middleTapeSensor;

RobotMotorData_t robotMotors;
FullSensorData_t wall_data;
ReturnToTapeData_t homing_data;

QueueHandle_t xSharedQueue = xQueueCreate(10, sizeof(StatusMessage_t));
QueueHandle_t uart_msg_queue = xQueueCreate(10, sizeof(Packet_t));

TaskHandle_t xMasterHandle = NULL;

void TaskMaster(void *pvParameters);
void begin_counter_docking();
void begin_wall_slamming();
void begin_homing();

void wait_for_notification() {
    uint32_t ulNotificationValue;
    xTaskNotifyWait(0x00, 0xFFFFFFFF, &ulNotificationValue, portMAX_DELAY); // Wait for message from task
}

void uart_msg_handler(void *parameter) {
    log_status("Started message handler");
    
    while (1) {
        Packet_t new_packet;

        if (xQueueReceive(uart_msg_queue, &new_packet, portMAX_DELAY)) {
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
                        state.drive_state = STOP;
                        state.drive_speed = 0;

                        if (xFollowWallHandle != NULL) { // If we're currently wall following, we want to stop 
                            vTaskDelete(xFollowWallHandle);
                            xFollowWallHandle = NULL;
                        }

                        send_uart_message(ACCEPTED, 0, false);
                        break;

                    case COUNTER_DOCK:
                    {
                        state.current_action = DOCK_AT_STATION;
                        state.y_direction = new_packet.value;
                        state.last_side_station = 0;

                        send_uart_message(ACCEPTED, 0, false);
                        break;
                    }

                    case FOLLOW_WALL_TO:
                    {
                        // packet contains the desired side station
                        state.current_action = ActionType_t::WALL_SLAM_TO;
                        state.desired_side_station = new_packet.value;

                        send_uart_message(ACCEPTED, 0, false);
                        break;
                    }
                    case DO_PIROUETTE:
                    {
                        // packet contains the last_side_station on the side we will end up on
                        state.current_action = ActionType_t::PIROUETTE;
                        state.last_side_station = new_packet.value;

                        send_uart_message(ACCEPTED, 0, false);
                        break;
                    }

                    case SWITCH_COUNTER:
                    {
                        state.current_action = ActionType_t::SIDE_SWAP;
                        state.last_side_station = new_packet.value;

                        send_uart_message(ACCEPTED, 0, false);
                        break;
                    }

                    case SET_MULTIPLIER:
                    {
                        float new_multipler = (float)new_packet.value / 100.0;
                        state.speed_modifier = new_multipler;

                        send_uart_message(ACCEPTED, 0, false);
                        break;
                    }

                    case STARTUP_SERVER:
                    {
                        state.current_action = ActionType_t::STARTUP;

                        send_uart_message(ACCEPTED, 0, false);
                        break;
                    }

                    case READY:
                    {
                        send_uart_message(ACK, 0, false);
                    }

                    case 0x40 ... 0x4f:
                        log_status("Received acknowledgement!");
                        break;
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10)); // Small delay to yield    
    }
}


void setup() {
    Serial.begin(115200);

    init_pwm();

    init_state();

    initialize_uart(&uart_msg_queue, TX_PIN, RX_PIN);

    xTaskCreate(uart_msg_handler, "uart_msg_handler", 2048, NULL, 7, NULL);

    motor_front_left = instantiate_robot_motor(MOTOR_FRONT_LEFT_FORWARD, MOTOR_FRONT_LEFT_REVERSE, timer_1);
    motor_front_right = instantiate_robot_motor(MOTOR_FRONT_RIGHT_FORWARD, MOTOR_FRONT_RIGHT_REVERSE, timer_1);
    motor_back_left = instantiate_robot_motor(MOTOR_BACK_LEFT_FORWARD, MOTOR_BACK_LEFT_REVERSE, timer_2);
    motor_back_right = instantiate_robot_motor(MOTOR_BACK_RIGHT_FORWARD, MOTOR_BACK_RIGHT_REVERSE, timer_2);

    frontTapeSensor = instantiate_tape_sensor(FRONT_TAPE_SENSOR_RIGHT);
    backTapeSensor = instantiate_tape_sensor(BACK_TAPE_SENSOR_LEFT);
    middleTapeSensor = instantiate_tape_sensor(MIDDLE_TAPE_SENSOR);

    #if debug_sensors == 0
        while (1) {
            read_tape_sensor(backTapeSensor);
            read_tape_sensor(frontTapeSensor);
            read_tape_sensor(middleTapeSensor);
            Serial.println("Front: " + String(frontTapeSensor->value));
            Serial.println("Back: " + String(backTapeSensor->value));
            Serial.println("Middle: " + String(middleTapeSensor->value));
            delay(100);
        }
    #endif 

    robotMotors = { motor_front_right, motor_front_left, motor_back_right, motor_back_left };
    wall_data = { frontTapeSensor, backTapeSensor };
    homing_data = {middleTapeSensor, &xMasterHandle };

    // check if driving task was created
    if (xTaskCreate(TaskDrive, "DrivingTask", 2048, &robotMotors, PRIORITY_DRIVE_UPDATE, &xDriveHandle) == pdPASS) {
        log_status("Driving task was created successfully."); 
    } else {
        log_error("Driving task was not created successfully!");
    }

    delay(100);

    // check if task master was created
    if (xTaskCreate(TaskMaster, "MasterTask", 2048, NULL, 2, &xMasterHandle) == pdPASS) {
        log_status("Master task was created successfully.");
    } else {
        log_error("Master task was not created successfully!");
    }

}

void loop()
{
    // Nothing to do here :)
}

void TaskMaster(void *pvParameters)
{
    log_status("Beginning master task...");
    send_uart_message(READY);

    while (1) {
        StatusMessage_t receivedMessage;
        switch (state.current_action) {
            case IDLE:
            {
                state.drive_state = DriveState_t::STOP;
                state.drive_speed = 0;
                log_status("Idling...");

                while (state.current_action == IDLE) {
                    vTaskDelay(20 / portTICK_PERIOD_MS);
                }
                break;
            }

            case DOCK_AT_STATION:
            {
                log_status("Counter docking!");

                state.drive_state = TRANSLATE;
                
                state.drive_speed = MOTOR_SPEED_TRANSLATION;

                begin_counter_docking();

                wait_for_notification();

                log_status("Arrived at station counter!");

                if (state.current_action == DOCK_AT_STATION) {
                    state.current_action = IDLE;
                    state.drive_state = STOP;
                    state.direction = FORWARD_DRIVE;
                }

                send_uart_message(COMPLETED);
                break;
            }

            case ActionType_t::WALL_SLAM_TO:
            {
                if (state.desired_side_station == 0) {
                    log_error("invalid, recieved 0 as desired side station");
                    send_uart_message(COMPLETED);
                    state.current_action = IDLE;
                }
                int station_difference = state.desired_side_station - state.last_side_station;

                // abort navigation if desired station is our last station
                if (station_difference == 0) {
                    log_error("already at desired station!");
                    send_uart_message(COMPLETED);
                    state.current_action == IDLE;
                }

                // cannot wall slam if we aren't at a wall
                if (state.y_direction == 0) {
                    log_error("not currently docked at a wall!");
                    send_uart_message(COMPLETED);
                    state.current_action == IDLE;
                }
                
                // determine direction to go in by looking at sign of station difference and orientation
                state.direction = sign(station_difference * state.orientation);

                state.yaw = YAW_WALL_SLAMMING * state.orientation * state.y_direction * state.direction;

                if (state.orientation && state.y_direction && state.direction) {
                    state.yaw = (int)(state.yaw * 1.75);
                }

                Serial.println("YAW: " + String(YAW_WALL_SLAMMING) + " Orientation: " + String(state.orientation) + " Y-Direction: " + String(state.y_direction) + " Direction: " + String(state.direction));

                // Start Driving
                state.drive_state = DRIVE;
                state.drive_speed = MOTOR_SPEED_WALL_SLAMMING;

                // Delay in case we are already on tape
                vTaskDelay(pdMS_TO_TICKS(DELAY_STATION_TRACKING_INTITAL));
                
                log_status("Beginning wall slamming!");
                begin_wall_slamming();

                Serial.println("Wall Slamming: " + String(state.desired_side_station) + " " + String(state.last_side_station));
                
                while (state.current_action == WALL_SLAM_TO) {
                    if (state.desired_side_station == state.last_side_station) {
                        log_status("arrived at desired station!");

                        if(xFollowWallHandle != NULL) {
                            vTaskDelete(xFollowWallHandle);
                            xFollowWallHandle = NULL;
                        }

                        log_status("approaching tape, lowering motor speed");

                        // break to drop speed quickly
                        state.direction = -state.direction;
                        state.yaw = -state.yaw;
                        state.drive_speed = 13000;
                        vTaskDelay(pdMS_TO_TICKS(400));

                        state.direction = -state.direction;
                        state.yaw = -state.yaw;
                        state.drive_state = STOP;
                        vTaskDelayMS(300);

                        begin_homing();

                        wait_for_notification();

                        // slam into counter to align
                        state.yaw = 0;
                        state.drive_speed = MOTOR_SPEED_TRANSLATION;
                        state.drive_state = TRANSLATE;
                        vTaskDelayMS(250);

                        state.drive_speed = 0;
                        state.drive_state = DriveState_t::STOP;
                        state.current_action = IDLE;
                        send_uart_message(COMPLETED);
                    }
                    vTaskDelayMS(3);
                }
                break;
            }

            case ActionType_t::PIROUETTE:
            {   
                if (state.y_direction == 0) {
                    log_error("cannot do a pirouette when not on a wall!");
                    send_uart_message(COMPLETED);
                    state.current_action = IDLE;
                }

                state.yaw = 0;
                float final_angle = 65.0;
                int final_delay = DELAY_FINISH_PIROUETTE;
                int initial_delay = DELAY_START_PIROUETTE;
                bool counter_return = state.last_side_station < 0;
                bool slow_pirouette = state.speed_modifier < 0.99;

                if (state.orientation * state.y_direction == 1) {
                    final_angle *= 0.82;

                }
                
                // if desired side station is negative we want to return to the counter we just came from
                if (counter_return) {
                    // fix the negativeness of the side station
                    state.last_side_station = -state.last_side_station;

                    final_angle *= 1.2;
                    final_delay = int(final_delay * 1.5);
                }

                if (slow_pirouette) {
                    final_delay *= 1 / state.speed_modifier;
                    initial_delay *= 0.8;
                    final_angle *= 1.33 / state.speed_modifier;
                }

                log_status("doing pirouette!!");

                // move away from current counter
                state.y_direction = -state.y_direction;
                state.drive_state = TRANSLATE;
                state.drive_speed = MOTOR_SPEED_PIROUETTE_TRANSLATION;
                vTaskDelay(pdMS_TO_TICKS(initial_delay));

                state.drive_state = DriveState_t::ROTATE_AND_TRANSLATE;

                // angle to 217 for robot 2 pirouette in islation
                for (double angle = 0.0; angle <= final_angle; angle += 0.40 * state.speed_modifier) {
                    state.pirouette_angle = (int)(angle * state.helicity);
                    vTaskDelay(pdMS_TO_TICKS(4));
                }
                
                if (counter_return) {
                    state.y_direction = -state.y_direction;
                }

                // we are now on the opposite wall, and we have rotated 180 degrees
                state.orientation = -state.orientation;
                state.drive_state = TRANSLATE;
                vTaskDelay(pdMS_TO_TICKS(final_delay));

                state.drive_state = STOP;
                state.drive_speed = 0;
                state.pirouette_angle = 0;
                state.current_action = IDLE;

                send_uart_message(COMPLETED);
                break;
            }

            case ActionType_t::SIDE_SWAP:
            {
                if (state.y_direction == 0) {
                    log_error("cannot side swap when y direction is 0");
                    send_uart_message(COMPLETED);
                    state.current_action = IDLE;
                }
                
                state.y_direction = -state.y_direction;
                state.drive_speed = MOTOR_SPEED_TRANSLATION;
                state.drive_state = TRANSLATE;
                vTaskDelay(pdMS_TO_TICKS(DELAY_TRANSLATE_SIDE_SWAP));

                log_status("side swap completed!");

                state.drive_speed = 0;
                state.drive_state = STOP;
                state.y_direction = -state.y_direction;
                state.current_action = IDLE;

                send_uart_message(COMPLETED);
                break;
            }

            case ActionType_t::STARTUP:
            {
                #if !(robot == 0)
                while (1) {
                    log_error("Trying to perform Midnight Rambler's startup sequence for a different robot!");
                    vTaskDelayMS(500);
                }
                #endif

                state.y_direction = 1;
                state.drive_speed = MOTOR_SPEED_TRANSLATION;
                state.drive_state = TRANSLATE;
                vTaskDelay(pdMS_TO_TICKS(1300));

                state.drive_speed = MOTOR_SPEED_WALL_SLAMMING_CRAWL;
                state.drive_state = DRIVE;
                state.direction = -1;
                vTaskDelay(pdMS_TO_TICKS(500));

                state.direction = 1;
                begin_homing();

                wait_for_notification();

                state.drive_speed = 0;
                state.drive_state = STOP; 
                state.current_action = IDLE;
                state.last_side_station = 1;

                send_uart_message(COMPLETED);
                break;
            }
        }
    }
}

void begin_counter_docking() {
    // check if counter docking task was created
    if (xTaskCreate(TaskCounterDocking, "Counter_Docking", 2048, &xMasterHandle, PRIORITY_STATION_TRACKING, &xCounterDockingHandle) == pdPASS) {
        log_status("Docking task was created successfully.");
    } else {
        log_error("Docking task was not created successfully!");
    }
}

void begin_wall_slamming() {
    // check if counter docking task was created
    if (xTaskCreate(TaskFollowWall, "Follow_Wall", 4096, &wall_data, PRIORITY_FOLLOW_WALL, &xFollowWallHandle) == pdPASS) {
        log_status("Wall Follow task was created successfully.");
    } else {
        log_error("Wall Follow task was not created successfully!");
    }
}

void begin_homing() {
    if(xTaskCreate(TaskHoming, "Follow_Wall", 2048, &homing_data, PRIORITY_FOLLOW_WALL, &xHomingHandle) == pdPASS) {
        log_status("Homing task was created successfully.");
    } else {
        log_error("Homing task was not created successfully!");
    }
}
