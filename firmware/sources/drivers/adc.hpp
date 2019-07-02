/**
* @file adc.hpp
* @brief Adc driver class
*/
#ifndef ADC_HPP
#define ADC_HPP

#include <hal.h>

/**
* @brief Adc
* @details It can be used for any sensor working with adc, for example current sensor.
* @note don't work now :(
*/
class Adc
{
public:
    enum AdcConstants
    {
        ADC1_CHANNELS_QUANTITY = 2,
        ADC1_BUFFER_DEPTH = 2,
    };

    static void Start();
    static adcsample_t Buffer[ADC1_CHANNELS_QUANTITY * ADC1_BUFFER_DEPTH];
};

#endif /* ADC_HPP */
