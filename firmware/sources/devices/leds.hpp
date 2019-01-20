#ifndef LEDS_HPP
#define LEDS_HPP

#include <hal.h>

// PB_0 - led1
// PB_7 - led2
// PB_14 - led3
class Led
{
public:
    static void Init();
    static void OnFirst();
    static void OffFirst();
    static void OnSecond();
    static void OffSecond();
    static void OnThird();
    static void OffThird();
};

#endif /* LEDS_HPP */
