/**
* @file pwm.cpp
* @brief Pwm driver implementation
*/
#include "pwm.hpp"
#include <ch.h>
#include <chprintf.h>


/*
* Short description:
* Requirements:
* halconf.h - enable HAL_USE_PWM
* mcuconf.h - enable STM32_PWM_USE_TIM*
*/


/**
* @brief Init Pwm driver
* @note It include:
* 1. Create pointer to PWM driver.
* 2. Create configuration structure:
*    - Frequency of base timer;
*    - Period of PWM measured in base time ticks
*      To obtain period of PWM: per_time = <.period> / <.frequency> [s]
*      Don't forget about size of timer counter register
*    - Callback invoked on PWM counter overflow reset
*      Here invoked every <.period> ticks
*    - PWM channels configuration. They initialized as array.
*      Each channels has two fields:
*      .mode     - disable / active high (direct) / active low (inversed) PWM
*      PWM_OUTPUT_DISABLED / PWM_OUTPUT_ACTIVE_HIGH / PWM_OUTPUT_ACTIVE_LOW
*      .callback - callback invoked on channel compare event
*    - Timer direct registers.
* 3. Setup PWM pin.
* 4. Start PWM driver.
**/
void Pwm::Create(Pin_t pin)
{
    PwmConf.frequency = 25000;
    PwmConf.period = 100;
    PwmConf.callback = NULL;
    PwmConf.cr2 = 0;
    PwmConf.dier = 0;

    switch(pin)
    {
        /*
        case PIN_PB0:
        {
            PwmDriver = &PWMD3;
            Channel = CHANNEL_3;
            PwmConf.channels[CHANNEL_3].mode = PWM_OUTPUT_ACTIVE_HIGH;
            PwmConf.channels[CHANNEL_3].callback = NULL;
            palSetPadMode( GPIOB, 0, PAL_MODE_ALTERNATE(2) );
            break;
        }
        */
        case PIN_PB7:
        {
            PwmDriver = &PWMD4;
            Channel = CHANNEL_2;
            PwmConf.channels[CHANNEL_2].mode = PWM_OUTPUT_ACTIVE_HIGH;
            PwmConf.channels[CHANNEL_2].callback = NULL;
            palSetPadMode( GPIOB, 7, PAL_MODE_ALTERNATE(2) );
            break;
        }
        default:
        {
            return;
        }
    }
    pwmStart( PwmDriver, &PwmConf );
}



void Pwm::Start(uint8_t dutyCycle)
{
    if((dutyCycle >= 0) && (dutyCycle <= 100))
    {
        pwmEnableChannel( PwmDriver, Channel, dutyCycle );
    }
}

