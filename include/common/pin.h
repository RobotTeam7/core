#ifndef PIN_H
#define PIN_H


#include <Arduino.h>
#include <map>

/// @brief Namespace containing members for converting between STM32 pin names (PA_0) and Arduino pin number aliases (PA0)
namespace PinConvert {
    #if USING_BLUE_PILL
        /// @brief Map Arduino pin number aliases (PA0) to STM32 pin names (PA_0)
        static std::map<int, PinName> pin_number_to_pin_name = {
            {PA0, PA_0},
            {PA1, PA_1},
            {PA2, PA_2},
            {PA3, PA_3},
            {PA4, PA_4},
            {PA5, PA_5},
            {PA6, PA_6},
            {PA7, PA_7},
            {PA8, PA_8},
            {PA9, PA_9},
            {PA10, PA_10},
            {PA11, PA_11},
            {PA12, PA_12},
            {PA13, PA_13},
            {PA14, PA_14},
            {PA15, PA_15},
            {PB0, PB_0},
            {PB1, PB_1},
            {PB2, PB_2},
            {PB3, PB_3},
            {PB4, PB_4},
            {PB5, PB_5},
            {PB6, PB_6},
            {PB7, PB_7},
            {PB8, PB_8},
            {PB9, PB_9},
            {PB10, PB_10},
            {PB11, PB_11},
            {PB12, PB_12},
            {PB13, PB_13},
            {PB14, PB_14},
            {PB15, PB_15},
        };

        /// @brief Map STM32 pin names (PA_0) to Arduino pin number aliases (PA0)
        static std::map<PinName, int> pin_name_to_pin_number  = {
            {PA_0, PA0},
            {PA_1, PA1},
            {PA_2, PA2},
            {PA_3, PA3},
            {PA_4, PA4},
            {PA_5, PA5},
            {PA_6, PA6},
            {PA_7, PA7},
            {PA_8, PA8},
            {PA_9, PA9},
            {PA_10, PA10},
            {PA_11, PA11},
            {PA_12, PA12},
            {PA_13, PA13},
            {PA_14, PA14},
            {PA_15, PA15},
            {PB_0, PB0},
            {PB_1, PB1},
            {PB_2, PB2},
            {PB_3, PB3},
            {PB_4, PB4},
            {PB_5, PB5},
            {PB_6, PB6},
            {PB_7, PB7},
            {PB_8, PB8},
            {PB_9, PB9},
            {PB_10, PB10},
            {PB_11, PB11},
            {PB_12, PB12},
            {PB_13, PB13},
            {PB_14, PB14},
            {PB_15, PB15},
        };
    #endif
};


// enum PinType {
//     ARUDINO,
//     STM32
// };

// class Pin {
// public:
//     Pin(int pinNumber);

//     #if USING_BLUE_PILL
//         Pin(PinName pinName);
//         PinName getName();
//     #endif

//     int getNumber();
// private:
//     PinType type;
//     int pinNumber;
// };

#endif // PIN_H