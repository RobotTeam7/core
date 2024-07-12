#ifndef GET_BUFFER_AVERAGE_H
#define GET_BUFFER_AVERAGE_H

#include <reflectance/reflectance_polling_config.h>

int get_buffer_average(CircularBuffer<int, BUFFER_SIZE>& sensor_buffer);

#endif