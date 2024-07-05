// Firmware designed for BluePill or ESP32
#ifndef RESOURCE_MANAGER_H
#define RESOURCE_MANAGER_H

#include <Arduino.h>
#include <set>

/**
 * Maximum number of channels that can be created for the ESP32
 */
#define MAX_CHANNELS 16

class ChannelManager {
public:
    static ChannelManager& getInstance();
    int requestChannel();

private:
    int currentChannel;

    ChannelManager();
    ChannelManager(const ChannelManager&) = delete;

    void operator=(const ChannelManager&) = delete;
};


#endif