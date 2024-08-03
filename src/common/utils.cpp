#include <common/utils.h>


void log_status(const char* status_message) {
    if (VERBOSITY_LEVEL >= STATUS_MESSAGES) {
        Serial.println(status_message);
    }    
}

void log_error(const char* error_message) {
    if (VERBOSITY_LEVEL >= ERRORS_ONLY) {
        Serial.println(error_message);
    }    
}

void log_message(const char* message) {
    if (VERBOSITY_LEVEL >= MOST_VERBOSE) {
        Serial.println(message);
    }    
} 

void check_heap() {
    Serial.print("Free heap: ");
    Serial.println(ESP.getFreeHeap());
}

int sign(int x) {
    return (x > 0) - (x < 0);
}
