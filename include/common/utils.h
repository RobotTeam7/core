#ifndef COMMON_UTILS
#define COMMON_UTILS

#include <Arduino.h>

#include <common/constants.h>


#define vTaskDelayMS(x) (vTaskDelay(pdMS_TO_TICKS(x)))

void log_status(const char*);   // Log a status to the serial output, if verbosity is great enough.
void log_error(const char*);    // Log an error to the serial output, if verbosity is great enough.
void log_message(const char*);  // Log a message to the serial output, if verbosity is great enough.

void check_heap(); // Check how much heap space is available

/**
 * @brief Returns the sign of the argument 
 * @returns 1 if positive, -1 if negative.
 */
int sign(int x);


#endif