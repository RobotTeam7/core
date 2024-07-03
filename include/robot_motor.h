#ifndef RobotMotor_h
#define RobotMotor_h

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <math.h>

#define DRIVE_MOTOR_PWM_FREQUENCY 50

// Enum to discretize between different driving modes that a motor can be in
enum driveMode { forward, reverse, none };

class RobotMotor {
    PinName boundForwardPin;  // Pin responsible for delivering a "drive forward" pulse train
    PinName boundReversePin;  // Pin responsible for delivering a "drive reverse" pulse train
    driveMode currentState;  
    uint16_t currentDrive;    // 0 – 65535

    public:
        RobotMotor(PinName forwardPin, PinName reversePin);
        RobotMotor();
        
        // Immedatiely change the drive state and strength of this motor
        void set_drive(uint16_t driveValue, driveMode newState);

        void stop();

        driveMode report_drive();
};

#endif