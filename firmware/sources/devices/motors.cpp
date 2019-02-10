/**
* @file motors.cpp
* @brief Motors implementation
*/
#include "motors.hpp"
#include "pwm.hpp"
#include <math.h>


/*
*/

PWMDriver* pwmDriverLeft    = &PWMD4;
PWMDriver* pwmDriverRight   = &PWMD2;
PWMConfig pwmConf =
{
    .frequency      = 2000000,
    .period         = 10000,
    .callback       = NULL,
    .channels       = {     /* PD12 - 15 */
                          {.mode = PWM_OUTPUT_ACTIVE_LOW,   .callback = NULL},
                          {.mode = PWM_OUTPUT_ACTIVE_LOW,   .callback = NULL},
                          {.mode = PWM_OUTPUT_ACTIVE_LOW,   .callback = NULL},
                          {.mode = PWM_OUTPUT_ACTIVE_LOW,   .callback = NULL}
                      },
    .cr2            = 0,
    .dier           = 0
};

static int power_2_pwm;


#define PWM_MOTOR_LEFT_POS_IDX  0
#define PWM_MOTOR_LEFT_NEG_IDX  1

#define PWM_MOTOR_RIGHT_POS_IDX 2
#define PWM_MOTOR_RIGHT_NEG_IDX 3


uint8_t FixDutyCycleRange(int8_t dc)
{
    const int8_t MAX = 100;
    const int8_t MIN = -100;
    return dc > MAX ? MAX : dc < MIN ? MIN : dc;
}


void Motors::Init()
{
    palSetPadMode( GPIOD, 12, PAL_MODE_ALTERNATE(2) );  // PWM4/1
    palSetPadMode( GPIOD, 13, PAL_MODE_ALTERNATE(2) );  // PWM4/2

    palSetPadMode( GPIOB, 10, PAL_MODE_ALTERNATE(1) );  // PWM2/3
    palSetPadMode( GPIOB, 11, PAL_MODE_ALTERNATE(1) );  // PWM2/4

    power_2_pwm = pwmConf.period / 100;

    pwmStart( pwmDriverLeft, &pwmConf );
    pwmStart( pwmDriverRight, &pwmConf );
}


void Motors::SetLeftPower(int8_t power)
{
    power = FixDutyCycleRange(power);

    int pwmValue = abs(power * power_2_pwm);

    pwmEnableChannel( pwmDriverLeft, PWM_MOTOR_LEFT_POS_IDX, power > 0 ? pwmValue : 0 );
    pwmEnableChannel( pwmDriverLeft, PWM_MOTOR_LEFT_NEG_IDX, power < 0 ? pwmValue : 0 );
}


void Motors::SetRightPower(int8_t power)
{
    power = FixDutyCycleRange(power);

    int pwmValue = abs(power * power_2_pwm);

    pwmEnableChannel( pwmDriverRight, PWM_MOTOR_RIGHT_POS_IDX, power > 0 ? pwmValue : 0 );
    pwmEnableChannel( pwmDriverRight, PWM_MOTOR_RIGHT_NEG_IDX, power < 0 ? pwmValue : 0 );
}
