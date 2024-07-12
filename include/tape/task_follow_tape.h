#ifndef TASK_FOLLOW_TAPE_H
#define TASK_FOLLOW_TAPE_H

#include <Arduino.h>
#include <tape/tape_following_config.h>
#include <tape/reflectance_polling_config.h>

void TaskFollowTape(void *pvParameters);
void TaskPollReflectance(void *pvParameters);

#endif 