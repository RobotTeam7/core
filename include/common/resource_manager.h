// Firmware designed for BluePill or ESP32
#ifndef RESOURCE_MANAGER_H
#define RESOURCE_MANAGER_H

#include <Arduino.h>
#include <set>

#define MAX_CHANNELS 16 /// Maximum number of channels that can be created for the ESP32

/**
 * @brief A Singleton that manages PWM emission channels
 */
class ChannelManager {
public:
    /**
     * @brief Acquire the instance. 
     */
    static ChannelManager& getInstance();

    /**
     * @brief Request a new channel. 
     * @returns The newly-assigned channel number if one is available, and -1 otherwise. 
     */    
    int requestChannel();

private:
    // We make the constructor private as we only want a single instance of this class
    ChannelManager();

    // We delete the move and copy operators as we only want a single instance of this class
    ChannelManager(const ChannelManager&) = delete;
    void operator=(const ChannelManager&) = delete;

    int currentChannel; // Tracks the current channel (channel numbers increment from 0-15, this will be the number
                        // of the newest-assigned channel).
};

// class PinManager {
// public:
//     static PinManager& getInstance();
//     int requestPin(int pin);

// private:
//     PinManager();

//     PinManager(const PinManager&) = delete;
//     void operator=(const PinManager&) = delete;

//     std::set<int> assignedPins;
// };


#endif