/**
* @file pwm.hpp
* @brief Pwm driver class
*/
#ifndef PWM_HPP
#define PWM_HPP

#include <hal.h>

/**
* @brief Pwm driver
*/
class Pwm
{
public:
    enum Pin_t: uint8_t
    {
        PIN_PB0,
        PIN_PB1,
        PIN_PB7,
        /*
         * TIM3_CH3     PB_0    led 1
         * TIM4_CH2     PB_7    led 2
         * TIM12_CH1    PB_14   led 3
         *
         * TIM3_CH1     PA_6
         * TIM3_CH2     PA_7
         * TIM3_CH3     PB_0
         * TIM3_CH4     PB_1
        */
    };
    //Pwm();
    void Create(Pin_t);
    void Start(uint8_t dutyCycle);
private:
    enum Channel_t: uint8_t
    {
        CHANNEL_1 = 0,
        CHANNEL_2 = 1,
        CHANNEL_3 = 2,
        CHANNEL_4 = 3,
    };
    Channel_t Channel;
    PWMDriver* PwmDriver;
    PWMConfig PwmConf;
};

#endif /* PWM_HPP */
