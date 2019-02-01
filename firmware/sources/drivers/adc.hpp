/**
* @file adc.hpp
* @brief Adc driver class
*/
#ifndef ADC_HPP
#define ADC_HPP

#include <hal.h>

/**
* @brief Adc driver
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
    static adcsample_t Kek[ADC1_CHANNELS_QUANTITY * ADC1_BUFFER_DEPTH];
};

#endif /* ADC_HPP */
