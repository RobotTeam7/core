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

// returns the mean value in the buffer
int get_buffer_average(CircularBuffer<int, REFLECTANCE_SENSOR_BUFFER_SIZE> &sensor_buffer)
{
    int sum = 0;
    for (int i = 0; i < sensor_buffer.size(); ++i)
    {
    sum += sensor_buffer[i];
    }
    // truncates integer
    return sum / sensor_buffer.size();
}

void check_heap() {
    Serial.print("Free heap: ");
    Serial.println(ESP.getFreeHeap());
}

int sign(int x) {
    return (x > 0) - (x < 0);
}
