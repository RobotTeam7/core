#include <common/resource_manager.h>

ChannelManager& ChannelManager::getInstance() {
    static ChannelManager instance;
    return instance;
}

int ChannelManager::requestChannel() {
    int nextChannel = currentChannel + 1;
    if (nextChannel < MAX_CHANNELS) {
        this->currentChannel = nextChannel;
        return nextChannel;
    } else {
        Serial.println("Maximum channels already assigned");
        return -1;
    }
}

ChannelManager::ChannelManager() {
    this->currentChannel = -1;
}

// PinManager& PinManager::getInstance() {
//     static PinManager instance;
//     return instance;
// }

// int PinManager::requestPin(int pin) {
//     if (assignedPins.find(pin) != assignedPins.end()) {
//         Serial.println("Pin" + String(pin) + "already assigned!");
//         return -1;
//     }
//     assignedPins.insert(pin);
//     return pin;
// }

// PinManager::PinManager() {
//     this->assignedPins = {};
// }