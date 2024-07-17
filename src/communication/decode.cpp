#include <communication/decode.h>


CommandMessage_t decode_command(uint8_t message_identifier) {
    switch (message_identifier) {
        case 0x0000:
            return GOTO;
        
        case 0x0001:
            return DO_SPIN;

        case 0x0002:
            return COMPLETED;

        default:
            return NONE;
    }
}
