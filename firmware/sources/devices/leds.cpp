/**
* @file leds.cpp
* @brief Board leds implementation
*/
#include "leds.hpp"
#include "pwm.hpp"

/*
* Short description:
* PB_0 - led1
* PB_7 - led2
* PB_14 - led3
*/

/**
* @brief Work areas
* @param[in] first  - the name to be assigned to the stack array
* @param[in] second - the stack size to be assigned to the thread
*/
static THD_WORKING_AREA(firstLedThreadWorkingArea, 128);
static THD_WORKING_AREA(secondLedThreadWorkingArea, 128);
static THD_WORKING_AREA(thirdLedThreadWorkingArea, 128);

/**
* @brief First led thread function
* @param[in] FirstLedThread - function name
* @param[in] arg - arguments
*/
static THD_FUNCTION(FirstLedThread, arg)
{
    arg = arg;
    Led::Init();
    while (TRUE)
    {
        Led::OnFirst();
        chThdSleepMilliseconds(1000);
        Led::OffFirst();
        chThdSleepMilliseconds(1000);
    }
}


/**
* @brief Second led thread function
* @param[in] SecondLedThread - function name
* @param[in] arg - arguments
*/
static THD_FUNCTION(SecondLedThread, arg)
{
    arg = arg;
    Led::Init();
    while (TRUE)
    {
        Led::OnSecond();
        chThdSleepMilliseconds(500);
        Led::OffSecond();
        chThdSleepMilliseconds(500);
    }
}


/**
* @brief Third led thread function
* @param[in] ThirdLedThread - function name
* @param[in] arg - arguments
*/
static THD_FUNCTION(ThirdLedThread, arg)
{
    arg = arg;
    Led::Init();
    while (TRUE)
    {
        Led::OnThird();
        chThdSleepMilliseconds(250);
        Led::OffThird();
        chThdSleepMilliseconds(250);
    }
}


void Led::Init()
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    GPIOB->MODER |= (1 << 0) | (1 << 7) | (1 << 14);
}


void Led::StartBlink()
{
    chThdCreateStatic(firstLedThreadWorkingArea, sizeof(firstLedThreadWorkingArea), NORMALPRIO, FirstLedThread, NULL /* arg is NULL */);
    chThdCreateStatic(secondLedThreadWorkingArea, sizeof(secondLedThreadWorkingArea), NORMALPRIO, SecondLedThread, NULL /* arg is NULL */);
    chThdCreateStatic(thirdLedThreadWorkingArea, sizeof(thirdLedThreadWorkingArea), NORMALPRIO, ThirdLedThread, NULL /* arg is NULL */);
}


void Led::OnFirst()
{
    GPIOB->BSRR.W = GPIO_BSRR_BS_0;
}


void Led::OffFirst()
{
    GPIOB->BSRR.W = GPIO_BSRR_BR_0;
}


void Led::OnSecond()
{
    GPIOB->BSRR.W = GPIO_BSRR_BS_7;
}


void Led::OffSecond()
{
    GPIOB->BSRR.W = GPIO_BSRR_BR_7;
}


void Led::OnThird()
{
    GPIOB->BSRR.W = GPIO_BSRR_BS_14;
}


void Led::OffThird()
{
    GPIOB->BSRR.W = GPIO_BSRR_BR_14;
}
