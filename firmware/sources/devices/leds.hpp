/**
* @file leds.hpp
* @brief Board leds class
*/
#ifndef LEDS_HPP
#define LEDS_HPP

#include <hal.h>


/**
* @brief Board leds
*/
class Leds
{
public:
    static void Init();

    static void OnFirst();
    static void OffFirst();
    static void OnSecond();
    static void OffSecond();
    static void OnThird();
    static void OffThird();
private:

};

#endif /* LEDS_HPP */
