#include "leds.hpp"


void Led::Init()
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    GPIOB->MODER |= (1 << 0) | (1 << 7) | (1 << 14);
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
