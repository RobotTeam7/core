#include <common/channel_manager.h>

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