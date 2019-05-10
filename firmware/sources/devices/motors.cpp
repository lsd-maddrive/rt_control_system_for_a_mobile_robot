/**
* @file motors.cpp
* @brief Motors implementation
*/
#include "motors.hpp"
#include <ch.h>
#include <chprintf.h>
#include <math.h>


/*
* Short description:
* Requirements:
* halconf.h - enable HAL_USE_PWM
* mcuconf.h - enable STM32_PWM_USE_TIM*
*/


/**
* @brief Pointers to PWM drivers
**/
PWMDriver* PwmDriverLeft    = &PWMD4;
PWMDriver* PwmDriverRight   = &PWMD2;


/**
* @brief Configuration structure
* @note It include:
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
**/
PWMConfig PwmConf =
{
    .frequency      = 2000000,
    .period         = 10000,
    .callback       = NULL,
    .channels       = {
                          {.mode = PWM_OUTPUT_ACTIVE_LOW,   .callback = NULL},
                          {.mode = PWM_OUTPUT_ACTIVE_LOW,   .callback = NULL},
                          {.mode = PWM_OUTPUT_ACTIVE_LOW,   .callback = NULL},
                          {.mode = PWM_OUTPUT_ACTIVE_LOW,   .callback = NULL}
                      },
    .cr2            = 0,
    .dier           = 0
};


/**
* @brief Coefficient between power and pwm
**/
static int32_t PowerToPwm;
static int8_t MotorLeftDutyCycle = 0;
static int8_t MotorRightDutyCycle = 0;


/**
* @brief Indexes
**/
enum
{
    PWM_MOTOR_LEFT_POS_IDX = 0,
    PWM_MOTOR_LEFT_NEG_IDX = 1,
    PWM_MOTOR_RIGHT_POS_IDX = 2,
    PWM_MOTOR_RIGHT_NEG_IDX = 3,
};


/**
* @brief Init Pwm driver
* @note It include:
*   1. Setup PWM pins.
*   2. Calculate coefficient power to pwm
*   3. Start PWM drivers.
**/
void Motors::Init()
{
    palSetPadMode( GPIOD, 12, PAL_MODE_ALTERNATE(2) );  // PWM4/1
    palSetPadMode( GPIOD, 13, PAL_MODE_ALTERNATE(2) );  // PWM4/2

    palSetPadMode( GPIOB, 10, PAL_MODE_ALTERNATE(1) );  // PWM2/3
    palSetPadMode( GPIOB, 11, PAL_MODE_ALTERNATE(1) );  // PWM2/4

    PowerToPwm = PwmConf.period / 100;

    pwmStart( PwmDriverLeft, &PwmConf );
    pwmStart( PwmDriverRight, &PwmConf );
}


/**
* @brief Set motor left power from -100 to +100
* @param power - duty cycle, where negative mean inverse direction.
**/
void Motors::SetLeftPower(int8_t power)
{
    if(power < -100)
        power = -100;
    else if(power > 100)
        power = 100;

    MotorLeftDutyCycle = power;

    int pwmValue = abs(power * PowerToPwm);

    pwmEnableChannel( PwmDriverLeft, PWM_MOTOR_LEFT_POS_IDX, power > 0 ? pwmValue : 0 );
    pwmEnableChannel( PwmDriverLeft, PWM_MOTOR_LEFT_NEG_IDX, power < 0 ? pwmValue : 0 );
}


/**
* @brief Set motor right power from -100 to +100
* @param power - duty cycle, where negative mean inverse direction.
**/
void Motors::SetRightPower(int8_t power)
{
    if(power < -100)
        power = -100;
    else if(power > 100)
        power = 100;

    MotorRightDutyCycle = power;

    int pwmValue = abs(power * PowerToPwm);

    pwmEnableChannel( PwmDriverRight, PWM_MOTOR_RIGHT_POS_IDX, power > 0 ? pwmValue : 0 );
    pwmEnableChannel( PwmDriverRight, PWM_MOTOR_RIGHT_NEG_IDX, power < 0 ? pwmValue : 0 );
}


/**
* @brief Get motor left power from -100 to +100
* @return power - duty cycle, where negative mean inverse direction.
**/
int8_t Motors::GetLeftPower()
{
    return MotorLeftDutyCycle;
}


/**
* @brief Get motor right power from -100 to +100
* @return power - duty cycle, where negative mean inverse direction.
**/
int8_t Motors::GetRightPower()
{
    return MotorRightDutyCycle;
}
