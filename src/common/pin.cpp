// #include <common/pin.h>
// #include <common/resource_manager.h>

// #if USING_BLUE_PILL
//     Pin::Pin(PinName pinName) {
//         int pinNumber = PinConvert::pin_name_to_pin_number[pinName];
        
//         int acquired = PinManager::getInstance().requestPin(pinNumber);
//         if (acquired != 0) {
//             Serial.println("Couldn't acquire pin: " + String(pinName));
//             exit(1);
//         }

//         this->type = ARUDINO;
//         this->pinNumber = pinNumber;
//     }

//     PinName Pin::getName() {
//         return PinConvert::pin_number_to_pin_name[this->pinNumber];
//     }
// #endif

// Pin::Pin(int pinNumber) {
//     int acquired = PinManager::getInstance().requestPin(pinNumber);
//     if (acquired != pinNumber) {
//         Serial.println("Couldn't acquire pin: " + String(pinNumber));
//         exit(1);
//     } else {
//         Serial.println("Acquired pin: " + String(pinNumber));
//     }

//     this->type = ARUDINO;
//     this->pinNumber = pinNumber;
// }

// int Pin::getNumber() {
//     return this->pinNumber;
// }