#ifndef PID_H
#define PID_H

#include <Arduino.h>

#include <motion/constants.h>

int pid_follow_tape(int error, int lastError);


#endif // PID_H