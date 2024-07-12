
#include <CircularBuffer.h>
// for buffer size
#include <reflectance/reflectance_polling_config.h>

// returns the mean value in the buffer
int get_buffer_average(CircularBuffer<int, BUFFER_SIZE>& sensor_buffer) {
    int sum = 0;
    for (int i = 0; i < sensor_buffer.size(); ++i) {
        sum += sensor_buffer[i];
    }
    // truncates integer
    return sum / sensor_buffer.size();
}