#include <motion/pid.h>


int pid_follow_tape(int error, int lastError) {
    int p = kp * error;
    int d = kd * (error - lastError);
    return p + d;
}
