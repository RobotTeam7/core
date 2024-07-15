// Firmware designed for BluePill
#ifndef RobotMotor_h
#define RobotMotor_h

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <math.h>
#include <common/pwm.h>
#include <common/pin.h>

#define DRIVE_MOTOR_PWM_FREQUENCY 50

#define NO_DRIVE 0
#define FORWARD_DRIVE 1
#define REVERSE_DRIVE 2


// /**
//  * \brief This enum exists to discretize between different driving modes that a motor can be in
//  */
// enum driveMode { forward, reverse, none };

// /**
//  * \brief This class represents a robot's motor.
//  * 
//  * \details Use this class to bind forward and reverse pins, and control the motor abstractly through simple commands.
//  */
// class RobotMotor {
//     uint8_t boundForwardPin;        // Pin responsible for delivering a "drive forward" pulse train
//     uint8_t boundReversePin;        // Pin responsible for delivering a "drive reverse" pulse train
//     driveMode currentState;  
//     uint16_t currentDrive;      // 0 – 65535

//     public:
//         /**
//          * \brief Instantiate a motor, binding `forwardPin` and `reversePin` to it. The pins will be configured as necessary.
//          */
//         RobotMotor(uint8_t forwardPin, uint8_t reversePin);
        
//         /**
//          * \brief Instantiate an empty motor.
//          */
//         RobotMotor();

//         /**
//          * \brief Set the drive state of this motor.
//          * 
//          * \param driveValue Drive power, where 0 is stopped and 65535 is full power
//          * \param newState Direction of the motor, either `forward`, `reverse`, or `none`.
//          */
//         void set_drive(uint32_t driveValue, driveMode direction);

//         /**
//          * \brief Stop the motor.
//          */
//         void stop();

//         /**
//          * \brief Get the current motor direction.
//          * 
//          * \return The motor's current direction, either `forward`, `reverse`, or `none`.
//          */
//         driveMode report_drive();
// };


typedef struct {
    uint8_t boundForwardPin;
    uint8_t boundReversePin;
    uint8_t currentState;
    uint16_t currentDrive;
} RobotMotor_t;

RobotMotor_t* instantiate_robot_motor(uint8_t forwardPin, uint8_t reversePin);
void motor_set_drive(RobotMotor_t* robotMotor, uint16_t driveValue, uint8_t direction);
void motor_stop(RobotMotor_t* robotMotor); 

#endif