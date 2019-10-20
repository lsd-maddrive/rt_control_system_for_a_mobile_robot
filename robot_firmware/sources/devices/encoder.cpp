/**
* @file encoder.cpp
* @brief Encoder implementation
*/

#include <hal.h>
#include "encoder.hpp"


enum
{
    LEFT_ENC_A_CH = 10,
    LEFT_ENC_B_CH = 12,
    RIGHT_ENC_A_CH = 14,
    RIGHT_ENC_B_CH = 15,
};

static void left_wheel_a_cb(EXTDriver* extp, expchannel_t channel);
static void right_wheel_a_cb(EXTDriver* extp, expchannel_t channel);
static void left_wheel_b_cb(EXTDriver* extp, expchannel_t channel);
static void right_wheel_b_cb(EXTDriver* extp, expchannel_t channel);
static void speed_tmr_cb ( GPTDriver* speedTmr );


static int32_t LeftEncoderTicks = 0;
static int32_t RightEncoderTicks = 0;
static int32_t RightEncoderTicksCash = 0;
static int32_t LeftEncoderTicksCash = 0;
static float LeftEncoderSpeed = 0;
static float RightEncoderSpeed = 0;
static float TimerCallbacksPerSecond;

static const GPTConfig speedTimerCfg =
{
	.frequency      =  100000, // 100 KHz
	.callback       =  speed_tmr_cb,
	.cr2            =  0,
	.dier           =  0U
};


void Encoder::Init()
{
    const EXTConfig extcfg =
    {
        .channels =
        {
            [0]  = {EXT_CH_MODE_DISABLED, NULL},
            [1]  = {EXT_CH_MODE_DISABLED, NULL},
            [2]  = {EXT_CH_MODE_DISABLED, NULL},
            [3]  = {EXT_CH_MODE_DISABLED, NULL},
            [4]  = {EXT_CH_MODE_DISABLED, NULL},
            [5]  = {EXT_CH_MODE_DISABLED, NULL},
            [6]  = {EXT_CH_MODE_DISABLED, NULL},
            [7]  = {EXT_CH_MODE_DISABLED, NULL},
            [8]  = {EXT_CH_MODE_DISABLED, NULL},
            [9]  = {EXT_CH_MODE_DISABLED, NULL},
            [10] = {EXT_CH_MODE_BOTH_EDGES | EXT_CH_MODE_AUTOSTART | EXT_MODE_GPIOE, left_wheel_a_cb},
            [11] = {EXT_CH_MODE_DISABLED, NULL},
            [12] = {EXT_CH_MODE_BOTH_EDGES | EXT_CH_MODE_AUTOSTART | EXT_MODE_GPIOE, left_wheel_b_cb},
            [13] = {EXT_CH_MODE_DISABLED, NULL},
            [14] = {EXT_CH_MODE_BOTH_EDGES | EXT_CH_MODE_AUTOSTART | EXT_MODE_GPIOE, right_wheel_a_cb},
            [15] = {EXT_CH_MODE_BOTH_EDGES | EXT_CH_MODE_AUTOSTART | EXT_MODE_GPIOE, right_wheel_b_cb},
        }
    };

    extStart( &EXTD1, &extcfg );

    GPTDriver* speedTimer = &GPTD3;
    const int32_t periodInTicks = 10000;
    TimerCallbacksPerSecond = speedTimerCfg.frequency / periodInTicks;
    gptStart( speedTimer, &speedTimerCfg );
    gptStartContinuous( speedTimer, periodInTicks );
}


void Encoder::Reset()
{
    RightEncoderTicks = 0;
    LeftEncoderTicks = 0;

    RightEncoderTicksCash = 0;
    LeftEncoderTicksCash = 0;
}


int32_t Encoder::GetLeftValue()
{
    return LeftEncoderTicks;
}


int32_t Encoder::GetRightValue()
{
    return RightEncoderTicks;
}


void Encoder::SetLeftValue(int32_t numberOfTicks)
{
    LeftEncoderTicks = numberOfTicks;
}


void Encoder::SetRightValue(int32_t numberOfTicks)
{
    RightEncoderTicks = numberOfTicks;
}


int32_t Encoder::GetLeftSpeed()
{
    return LeftEncoderSpeed;
}


int32_t Encoder::GetRightSpeed()
{
    return RightEncoderSpeed;
}


/**
* @brief Calculate and update value of encoders speed static variable
**/
static void speed_tmr_cb(GPTDriver* speedTimer)
{
    speedTimer = speedTimer;

    float right_delta = RightEncoderTicks - RightEncoderTicksCash;
    RightEncoderTicksCash = RightEncoderTicks;
    RightEncoderSpeed = right_delta * TimerCallbacksPerSecond;

    float left_delta = LeftEncoderTicks - LeftEncoderTicksCash;
    LeftEncoderTicksCash = LeftEncoderTicks;
    LeftEncoderSpeed = left_delta * TimerCallbacksPerSecond;
}


/**
* @brief Increase or decrease value of encoder counter when interrupt occur
**/
static void left_wheel_a_cb(EXTDriver* extp, expchannel_t channel)
{
    extp = extp; channel = channel;

    if (palReadPad(GPIOE, LEFT_ENC_A_CH))
    {
        if (palReadPad( GPIOE, LEFT_ENC_B_CH))
            LeftEncoderTicks++;
        else
            LeftEncoderTicks--;
    }
    else
    {
        if (palReadPad(GPIOE, LEFT_ENC_B_CH))
            LeftEncoderTicks--;
        else
            LeftEncoderTicks++;
    }
}


/**
* @brief Increase or decrease value of encoder counter when interrupt occur
**/
static void right_wheel_a_cb(EXTDriver* extp, expchannel_t channel)
{
    extp = extp; channel = channel;

    if (palReadPad(GPIOE, RIGHT_ENC_A_CH))
    {
        if (palReadPad(GPIOE, RIGHT_ENC_B_CH))
            RightEncoderTicks++;
        else
            RightEncoderTicks--;
    }
    else
    {
        if (palReadPad(GPIOE, RIGHT_ENC_B_CH))
            RightEncoderTicks--;
        else
            RightEncoderTicks++;
    }
}


/**
* @brief Increase or decrease value of encoder counter when interrupt occur
**/
static void left_wheel_b_cb(EXTDriver* extp, expchannel_t channel)
{
    extp = extp; channel = channel;

    if (palReadPad( GPIOE, LEFT_ENC_B_CH))
    {
        if (palReadPad(GPIOE, LEFT_ENC_A_CH))
            LeftEncoderTicks--;
        else
            LeftEncoderTicks++;
    }
    else
    {
        if (palReadPad(GPIOE, LEFT_ENC_A_CH))
            LeftEncoderTicks++;
        else
            LeftEncoderTicks--;
    }
}


/**
* @brief Increase or decrease value of encoder counter when interrupt occur
**/
static void right_wheel_b_cb(EXTDriver* extp, expchannel_t channel)
{
    extp = extp; channel = channel;

    if (palReadPad(GPIOE, RIGHT_ENC_B_CH))
    {
        if (palReadPad(GPIOE, RIGHT_ENC_A_CH))
            RightEncoderTicks--;
        else
            RightEncoderTicks++;
    }
    else
    {
        if (palReadPad(GPIOE, RIGHT_ENC_A_CH))
            RightEncoderTicks++;
        else
            RightEncoderTicks--;
    }
}
