#include <motion/pid.h>

int pid_follow_tape(int error, int lastError) {
    float kp = 1.1;
    float kd = 1.8;
    int p = kp * error;
    int d = kd * (error - lastError);
    return p + d;
}