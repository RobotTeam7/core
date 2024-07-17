#ifndef PID_H
#define PID_H

#include <Arduino.h>

int pid_follow_tape(int error, int lastError);

#endif 