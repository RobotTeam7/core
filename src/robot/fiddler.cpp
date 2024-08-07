#if robot == 1 // FIDDLER
#include <robot/fiddler_constants.h>
#include <robot/robot.h>

#include <common/hal.h>
#include <common/pwm.h>

#include <communication/wifi.h>
#include <communication/uart.h>


void TaskMaster(void* pvParameters) {
    log_status("Beginning master...");

    while (true) {
        send_command(COUNTER_DOCK, 1);
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

        // PATTY    
        log_status("goto patty");
        send_command(FOLLOW_WALL_TO, 2);
        vTaskDelayMS(750);
        send_uart_message(CommandMessage_t::ABORT, 0);
        vTaskDelayMS(100);
     
        log_status("Doing pirouette!");
        send_command(DO_PIROUETTE, 2);
        wait_for_motion();

        log_status("getting patty");
        send_command(FOLLOW_WALL_TO, 1);
        wait_for_motion();
        grab_with_claw(ServoPositionsPercentage_t::CLAW_CLOSED_PATTY);

        // COOK
        log_status("Doing pirouette!");
        send_command(DO_PIROUETTE, 1);
        wait_for_motion();

        log_status("goto cooktop");
        send_command(FOLLOW_WALL_TO, 3);
        wait_for_motion();
        open_claw(ServoPositionsPercentage_t::VERTICAL_HEIGHT_2);

        if (use_wifi) {
            log_status("Informing that patty is ready...");
            send_wifi_message(CommandMessage_t::NEXT_ACTION, 0);
        }
        
        // TOP BUN
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

        // RETURN
        send_command(SWITCH_COUNTER, 1);
        if (use_wifi) {
            vTaskDelayMS(200);
            log_status("Informing that top bun is ready...");
            send_wifi_message(CommandMessage_t::NEXT_ACTION, 0);
        }
        wait_for_motion();


        if (use_wifi) {
            while (!action_ready) {
                log_status("Waiting for plate station to be clear!");
                vTaskDelayMS(50);
            }
            action_ready = false;
            log_status("Plate station is clear!");
        }

        send_command(SWITCH_COUNTER, -1);
        wait_for_motion();




        // CIRCUIT 2




        // TOMATO
        send_command(FOLLOW_WALL_TO, 1);
        wait_for_motion();
        grab_with_claw(ServoPositionsPercentage_t::CLAW_CLOSED_TOMATO);

        // PIROUETTE
        send_command(FOLLOW_WALL_TO, 3);
        vTaskDelayMS(1000);
        send_command(ABORT, 0);
        vTaskDelayMS(700);
        send_command(DO_PIROUETTE, 1);
        wait_for_motion();

        // GET TO BOTTOM BUN
        send_command(FOLLOW_WALL_TO, 2);
        open_claw(ServoPositionsPercentage_t::VERTICAL_HEIGHT_1);
        grab_with_claw(ServoPositionsPercentage_t::CLAW_CLOSED_BUN);

        // DROP BUN  
        log_status("Doing pirouette!");
        send_command(DO_PIROUETTE, 2);
        wait_for_motion();

        log_status("drop bun");
        send_command(FOLLOW_WALL_TO, 4);
        wait_for_motion();
        open_claw(ServoPositionsPercentage_t::VERTICAL_HEIGHT_2);

        // PATTY    
        log_status("goto patty");
        send_command(FOLLOW_WALL_TO, 2);
        vTaskDelayMS(750);
        send_uart_message(CommandMessage_t::ABORT, 0);
        vTaskDelayMS(100);
     
        log_status("Doing pirouette!");
        send_command(DO_PIROUETTE, 2);
        wait_for_motion();

        log_status("getting patty");
        send_command(FOLLOW_WALL_TO, 1);
        wait_for_motion();
        grab_with_claw(ServoPositionsPercentage_t::CLAW_CLOSED_PATTY);

        // COOK
        log_status("Doing pirouette!");
        send_command(DO_PIROUETTE, 1);
        wait_for_motion();

        log_status("goto cooktop");
        send_command(FOLLOW_WALL_TO, 3);
        wait_for_motion();
        open_claw(ServoPositionsPercentage_t::VERTICAL_HEIGHT_2);

        if (use_wifi) {
            log_status("Informing that patty is ready...");
            send_wifi_message(CommandMessage_t::NEXT_ACTION, 0);
        }

        // TOP BUN
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

        // RETURN
        send_command(SWITCH_COUNTER, 1);
        if (use_wifi) {
            vTaskDelayMS(200);
            log_status("Informing that top bun is ready...");
            send_wifi_message(CommandMessage_t::NEXT_ACTION, 0);
        }
        wait_for_motion();


        if (use_wifi) {
            while (!action_ready) {
                log_status("Waiting for plate station to be clear!");
                vTaskDelayMS(50);
            }
            action_ready = false;
            log_status("Plate station is clear!");
        }

        send_command(SWITCH_COUNTER, -1);
        wait_for_motion();







        Serial.println("Done!");
        while (1) {
            vTaskDelay(1000);
        }
    }
}

void setup() {
    Serial.begin(115200); // Initialize serial monitor

    init_pwm();

    Serial.println("servos initialized!");

    claw_servo = instantiate_servo_motor(SERVO_CLAW_PIN, SERVO_CLAW_OPEN, SERVO_CLAW_CLOSED);
    vertical_servo = instantiate_servo_motor(SERVO_VERTICAL_PIN, SERVO_VERTICAL_DOWN, SERVO_VERTICAL_UP);

    set_servo_position_percentage(vertical_servo, ServoPositionsPercentage_t::VERTICAL_UP);
    
    delay(1000);

    init_communications(TX_PIN, RX_PIN);

    xTaskCreate(TaskMaster, "Master", 2048, NULL, 1, NULL);
}

void loop() {

}

#endif // FIDDLER