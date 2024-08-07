#if robot == 0 // MIDNIGHT RAMBLER

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <robot/midnight_rambler_constants.h>
#include <robot/robot.h>

#include <common/hal.h>
#include <common/pwm.h>

#include <communication/wifi.h>
#include <communication/uart.h>


ServoMotor_t* draw_bridge_servo;
ServoMotor_t* plating_servo;

// assumes plating servo is open initially
inline void grab_plate() {
    
    log_status("plate servo close");
    set_servo_position_percentage(plating_servo, ServoPositionsPercentage_t::PLATE_CLOSED);
    vTaskDelayMS(SERVO_ACTUATION_DELAY);

    set_servo_position_percentage(draw_bridge_servo, ServoPositionsPercentage_t::DRAW_BRIDGE_DOWN + 25);
    vTaskDelayMS(SERVO_ACTUATION_DELAY);
}

static inline void serve_food() {
    send_uart_message(CommandMessage_t::SET_MULTIPLIER, 80);

    send_command(FOLLOW_WALL_TO, 3);
    vTaskDelayMS(500);

    send_uart_message(CommandMessage_t::ABORT, 0, false);
    
    send_uart_message(CommandMessage_t::SET_MULTIPLIER, 75);
    
    send_command(DO_PIROUETTE, 2);
    wait_for_motion();

    send_command(FOLLOW_WALL_TO, 1);
    vTaskDelayMS(1250);
    send_command(ABORT, 0);
    vTaskDelayMS(SERVO_ACTUATION_DELAY);

    set_servo_position_percentage(plating_servo, ServoPositionsPercentage_t::PLATE_OPEN);
    vTaskDelayMS(SERVO_ACTUATION_DELAY);
    set_servo_position_percentage(draw_bridge_servo, ServoPositionsPercentage_t::DRAW_BRIDGE_UP);

}

void circuit() {
    // TOMATO _________________
    grab_with_claw(ServoPositionsPercentage_t::CLAW_CLOSED_TOMATO);

    send_command(FOLLOW_WALL_TO, 4);
    vTaskDelayMS(300);
    send_command(CommandMessage_t::ABORT, 0);
    vTaskDelayMS(300);
    send_command(DO_PIROUETTE, 2);
    wait_for_motion();

    // CHEESE _________________
    send_command(FOLLOW_WALL_TO, 1);
    wait_for_motion();
    open_claw(ServoPositionsPercentage_t::VERTICAL_HEIGHT_2);
    grab_with_claw(ServoPositionsPercentage_t::CLAW_CLOSED_CHEESE);

    // LETTUCE _________________
    send_command(FOLLOW_WALL_TO, 2);
    wait_for_motion();
    open_claw(ServoPositionsPercentage_t::VERTICAL_HEIGHT_1);
    grab_with_claw(ServoPositionsPercentage_t::CLAW_CLOSED_CHEESE);

    // PIROUETTE _________________
    send_command(FOLLOW_WALL_TO, 1);
    vTaskDelayMS(1200);
    send_command(CommandMessage_t::ABORT, 0);
    vTaskDelayMS(600);
    send_command(DO_PIROUETTE, 2);
    wait_for_motion();

    if (use_wifi) {
        while (!action_ready) {
            log_status("Waiting for patty to be ready!");
            vTaskDelayMS(50);
        }
        action_ready = false;
        log_status("Patty is ready!");
    }

    // PATTY _________________
    send_command(FOLLOW_WALL_TO, 3);
    wait_for_motion();
    open_claw(ServoPositionsPercentage_t::VERTICAL_HEIGHT_3);
    grab_with_claw(ServoPositionsPercentage_t::CLAW_CLOSED_PATTY);

    // DROP ON PLATE _________
    send_command(FOLLOW_WALL_TO, 4);
    wait_for_motion();
    open_claw(ServoPositionsPercentage_t::VERTICAL_UP);
    vTaskDelayMS(SERVO_ACTUATION_DELAY);

    // SWITCHING    _______________
    send_command(FOLLOW_WALL_TO, 2);
    vTaskDelayMS(350); // -> 300
    set_servo_position_percentage(vertical_servo, ServoPositionsPercentage_t::VERTICAL_DOWN);
    vTaskDelayMS(550); // -> 300
    send_command(ABORT, 0);

    if (use_wifi) {
        log_status("Plate station is clear...");
        send_wifi_message(CommandMessage_t::NEXT_ACTION, 0);
    }

    vTaskDelayMS(600);

    send_command(DO_PIROUETTE, -2);
    set_servo_position_percentage(vertical_servo, ServoPositionsPercentage_t::VERTICAL_UP);
    wait_for_motion();
    vTaskDelayMS(250);

    // GRAB PLATE   _______________
    send_command(FOLLOW_WALL_TO, 4);
    wait_for_motion();

    set_servo_position_percentage(plating_servo, ServoPositionsPercentage_t::PLATE_OPEN);
    vTaskDelayMS(SERVO_ACTUATION_DELAY);
    set_servo_position_percentage(draw_bridge_servo, ServoPositionsPercentage_t::DRAW_BRIDGE_DOWN);
    vTaskDelayMS(SERVO_ACTUATION_DELAY);

    if (use_wifi) {
        while (!action_ready) {
            log_status("Waiting for top bun to be ready!");
            vTaskDelayMS(50);
        }
        action_ready = false;
        log_status("Top bun is ready!");
    }

    grab_plate();

    // SERVE FOOD   _______________
    serve_food();
}

void TaskMaster(void* pvParameters) {
    log_status("Beginning master...");

    // STARTUP ________
    log_status("dock on side");
    send_command(STARTUP_SERVER, 0);
    wait_for_motion();
    
    while (true) {
        circuit();

        send_command(SET_MULTIPLIER, 100);
    
        // SWITCHING COUNTER 
        send_command(SWITCH_COUNTER, 3);
        wait_for_motion();
        send_command(FOLLOW_WALL_TO, 1);
        set_servo_position_percentage(vertical_servo, ServoPositionsPercentage_t::VERTICAL_DOWN);
        vTaskDelayMS(300);
        set_servo_position_percentage(vertical_servo, ServoPositionsPercentage_t::VERTICAL_UP);
        wait_for_motion();
    }
}

void setup() {
    Serial.begin(115200); // Initialize serial monitor

    init_pwm();

    // init_rack_and_pinion(RACK_FORWARD_PIN, RACK_REVERSE_PIN, CLAW_FORWARDS, SWITCH_RACK_PLATESIDE, SWITCH_RACK_CLAWSIDE);

    Serial.println("servos initialized!");

    claw_servo = instantiate_servo_motor(SERVO_CLAW_PIN, SERVO_CLAW_OPEN, SERVO_CLAW_CLOSED);
    draw_bridge_servo = instantiate_servo_motor(SERVO_DRAW_BRIDGE_PIN, SERVO_DRAW_BRIDGE_UP, SERVO_DRAW_BRIDGE_DOWN);
    plating_servo = instantiate_servo_motor(SERVO_PLATE_PIN, SERVO_PLATE_OPEN, SERVO_PLATE_CLOSED);
    vertical_servo = instantiate_servo_motor(SERVO_VERTICAL_PIN, SERVO_VERTICAL_DOWN, SERVO_VERTICAL_UP);
        
    set_servo_position_percentage(vertical_servo, ServoPositionsPercentage_t::VERTICAL_UP);

    init_communications(TX_PIN, RX_PIN);
    
    xTaskCreate(TaskMaster, "Master", 2048, NULL, 1, NULL);
}

void loop() {

}

#endif // MIDNIGHT RAMBLER