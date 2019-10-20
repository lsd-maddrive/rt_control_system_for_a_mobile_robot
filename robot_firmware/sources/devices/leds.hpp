/**
* @file leds.hpp
* @brief Board leds class
*/
#ifndef LEDS_HPP
#define LEDS_HPP


/**
* @brief Leds board
* @note It allows enable or disable leds using GPIO
* - PB_0  - led1
* - PB_7  - led2
* - PB_14 - led3
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
