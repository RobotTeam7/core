#if robot == 1 // FIDDLER
#include <robot/fiddler_constants.h>
#include <robot/robot.h>

#include <common/hal.h>
#include <common/pwm.h>

#include <communication/wifi.h>
#include <communication/uart.h>

void single_burger_circuit() {
    // send_command(DO_ESCAPE, 3); // REMOVE THIS FFS

    // GOTO PATTY
    send_command(FOLLOW_WALL_TO, 1);
    wait_for_motion();

    grab_with_claw(ServoPositionsPercentage_t::CLAW_CLOSED_PATTY);
    
    // GOTO COOKTOP
    send_command(FOLLOW_WALL_TO, 3);
    vTaskDelayMS(700);
    send_command(ABORT, 0);
    vTaskDelayMS(700);
    send_command(DO_PIROUETTE, 2);
    wait_for_motion();
    send_command(FOLLOW_WALL_TO, 3);
    wait_for_motion();
    open_claw(ServoPositionsPercentage_t::VERTICAL_HEIGHT_1);

    // GOTO BUN
    send_command(DO_PIROUETTE, 3);
    wait_for_motion();
    send_command(FOLLOW_WALL_TO, 2);
    wait_for_motion();
    grab_with_claw(ServoPositionsPercentage_t::CLAW_CLOSED_BUN);

    // DROP BUN  
    log_status("Doing pirouette!");
    send_command(DO_PIROUETTE, 2);
    wait_for_motion();

    log_status("drop bun");
    send_command(FOLLOW_WALL_TO, 4);
    wait_for_motion();
    open_claw(ServoPositionsPercentage_t::VERTICAL_HEIGHT_2);

    // GOTO BUN
    send_command(DO_ESCAPE, 3);
    wait_for_motion();
    send_command(FOLLOW_WALL_TO, 2);
    wait_for_motion();
    grab_with_claw(ServoPositionsPercentage_t::CLAW_CLOSED_BUN);

    // GOTO COOKTOP WITH BUN
    log_status("Doing pirouette!");
    send_command(DO_PIROUETTE, 2);
    wait_for_motion();

    log_status("drop bun");
    send_command(FOLLOW_WALL_TO, 3);
    wait_for_motion();
    open_claw(ServoPositionsPercentage_t::VERTICAL_HEIGHT_2);

    // ASSEMBLE BURGER
    grab_with_claw(ServoPositionsPercentage_t::CLAW_CLOSED_BUN);
    
    send_command(FOLLOW_WALL_TO, 4);
    wait_for_motion();
    open_claw(ServoPositionsPercentage_t::VERTICAL_HEIGHT_1);

    // ESCAPE
    send_command(DO_ESCAPE, 3);
    vTaskDelayMS(150);
        if (use_wifi) {
        log_status("Informing that patty is ready...");
        send_wifi_message(CommandMessage_t::NEXT_ACTION, 0);
    }
    wait_for_motion();
}

void circuit_salad() {
    // GOTO TOMATO
    send_command(DO_PIROUETTE, 3);
    wait_for_motion();

    send_command(FOLLOW_WALL_TO, 1);
    wait_for_motion();
    grab_with_claw(ServoPositionsPercentage_t::CLAW_CLOSED_PATTY);

    send_command(FOLLOW_WALL_TO, 4);
    wait_for_motion();
    open_claw(ServoPositionsPercentage_t::VERTICAL_HEIGHT_2);

    if (use_wifi) {
        log_status("Informing that patty is ready...");
        send_wifi_message(CommandMessage_t::NEXT_ACTION, 0);
    }

    send_command(DO_ESCAPE, 3);
    wait_for_motion();
}

void circuit() {
    // PATTY
    grab_with_claw(ServoPositionsPercentage_t::CLAW_CLOSED_PATTY);
    
    send_command(FOLLOW_WALL_TO, 3);
    vTaskDelayMS(700);
    send_command(ABORT, 0);
    vTaskDelayMS(800);
    send_command(DO_PIROUETTE, 2);
    wait_for_motion();
    send_command(FOLLOW_WALL_TO, 3);
    wait_for_motion();
    open_claw(ServoPositionsPercentage_t::VERTICAL_HEIGHT_1);

    if (use_wifi) {
        log_status("Informing that patty is ready...");
        send_wifi_message(CommandMessage_t::NEXT_ACTION, 0);
    }

    // BUN
    send_command(DO_PIROUETTE, 3);
    wait_for_motion();
    send_command(FOLLOW_WALL_TO, 2);
    wait_for_motion();
    grab_with_claw(ServoPositionsPercentage_t::CLAW_CLOSED_BUN);

    // DROP BUN  
    log_status("Doing pirouette!");
    send_command(DO_PIROUETTE, 2);
    wait_for_motion();

    log_status("drop bun");
    send_command(FOLLOW_WALL_TO, 4);
    wait_for_motion();
    open_claw(ServoPositionsPercentage_t::VERTICAL_HEIGHT_2);

    set_servo_position_percentage(vertical_servo, ServoPositionsPercentage_t::VERTICAL_UP);
    
    // TOP BUN
    send_command(FOLLOW_WALL_TO, 1);
    vTaskDelayMS(175);
    send_command(ABORT, 0);
    vTaskDelayMS(200);
    log_status("Doing pirouette!");
    send_command(DO_PIROUETTE, 3);
    wait_for_motion();



    log_status("getting top bun");
    send_command(FOLLOW_WALL_TO, 2);
    wait_for_motion();
    grab_with_claw(ServoPositionsPercentage_t::CLAW_CLOSED_BUN);

    // DROP BUN  
    log_status("Doing pirouette!");
    send_command(DO_PIROUETTE, 2);
    wait_for_motion();

    if (use_wifi) {
        while (!action_ready) {
            log_status("Waiting for plate station to be clear!");
            vTaskDelayMS(50);
        }
        action_ready = false;
        log_status("Plate station is clear!");
    }
    send_command(FOLLOW_WALL_TO, 4);
    wait_for_motion();
    open_claw(ServoPositionsPercentage_t::VERTICAL_HEIGHT_2);
    
    if (use_wifi) {
        log_status("Informing that top bun is ready...");
        send_wifi_message(CommandMessage_t::NEXT_ACTION, 0);
    }

    // RETURN
    send_command(DO_ESCAPE, 3);
    wait_for_motion();
}


void TaskMaster(void* pvParameters) {
    log_status("Beginning master...");

    send_command(STARTUP_SERVER, 0);
    wait_for_motion();

    circuit();

    send_command(FOLLOW_WALL_TO, 1);
    wait_for_motion();

    circuit();

    circuit_salad();
}

void setup() {
    Serial.begin(115200); // Initialize serial monitor

    init_pwm();

    Serial.println("servos initialized!");

    claw_servo = instantiate_servo_motor(SERVO_CLAW_PIN, SERVO_CLAW_OPEN, SERVO_CLAW_CLOSED);
    vertical_servo = instantiate_servo_motor(SERVO_VERTICAL_PIN, SERVO_VERTICAL_DOWN, SERVO_VERTICAL_UP);

    set_servo_position_percentage(vertical_servo, ServoPositionsPercentage_t::VERTICAL_UP);

    init_communications(TX_PIN, RX_PIN);

    xTaskCreate(TaskMaster, "Master", 2048, NULL, 1, NULL);
}

void loop() {

}

#endif // FIDDLER