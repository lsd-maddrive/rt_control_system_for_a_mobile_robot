/**
* @file leds.cpp
* @brief Leds board implementation
*/


#include "leds.hpp"


void Leds::Init()
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    GPIOB->MODER |= (1 << 0) | (1 << 7) | (1 << 14);
}


void Leds::OnFirst()
{
    GPIOB->BSRR.W = GPIO_BSRR_BS_0;
}


void Leds::OffFirst()
{
    GPIOB->BSRR.W = GPIO_BSRR_BR_0;
}


void Leds::OnSecond()
{
    GPIOB->BSRR.W = GPIO_BSRR_BS_7;
}


void Leds::OffSecond()
{
    GPIOB->BSRR.W = GPIO_BSRR_BR_7;
}


void Leds::OnThird()
{
    GPIOB->BSRR.W = GPIO_BSRR_BS_14;
}


void Leds::OffThird()
{
    GPIOB->BSRR.W = GPIO_BSRR_BR_14;
}
