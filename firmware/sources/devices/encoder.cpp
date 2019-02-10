/**
* @file encoder.cpp
* @brief Encoder implementation
*/
#include "encoder.hpp"

/*
*/


#define LEFT_ENC_A_CH   10
#define LEFT_ENC_B_CH   12
#define RIGHT_ENC_A_CH  14
#define RIGHT_ENC_B_CH  15


static void left_wheel_a_cb(EXTDriver *extp, expchannel_t channel);
static void right_wheel_a_cb(EXTDriver *extp, expchannel_t channel);
static void left_wheel_b_cb(EXTDriver *extp, expchannel_t channel);
static void right_wheel_b_cb(EXTDriver *extp, expchannel_t channel);

static void speed_tmr_cb ( GPTDriver *speedTmr );


static int32_t left_enc_ticks = 0;
static int32_t right_enc_ticks = 0;

static float left_enc_speed = 0;
static float right_enc_speed = 0;

static int32_t right_enc_cache = 0;
static int32_t left_enc_cache = 0;

static GPTDriver* speedTmr = &GPTD3;
static int32_t tmr_interval = 10000;

static const GPTConfig speedTmrCfg =
{
    .frequency      =  100000, // 100 KHz
    .callback       =  speed_tmr_cb,
    .cr2            =  0,
    .dier           =  0U
};

static float msr_2_sec;


/**
* @brief Init encoders
**/
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

    msr_2_sec = speedTmrCfg.frequency / tmr_interval;
    gptStart( speedTmr, &speedTmrCfg );
    gptStartContinuous( speedTmr, tmr_interval );
}


/**
* @brief Reset value of ticks and speeds
**/
void Encoder::Reset()
{
    right_enc_ticks = 0;
    left_enc_ticks = 0;

    right_enc_cache = 0;
    left_enc_cache = 0;
}


/**
* @brief Get number of left encoder ticks
**/
int32_t Encoder::GetLeftValue()
{
    return left_enc_ticks;
}


/**
* @brief Get number of right encoder ticks
**/
int32_t Encoder::GetRightValue()
{
    return right_enc_ticks;
}


/**
* @brief Get number of left encoder speed
**/
int32_t Encoder::GetLeftSpeed()
{
    return left_enc_speed;
}


/**
* @brief Get number of right encoder speed
**/
int32_t Encoder::GetRightSpeed()
{
    return right_enc_speed;
}


static void speed_tmr_cb ( GPTDriver *speedTmr )
{
    speedTmr = speedTmr;

    float right_delta = right_enc_ticks - right_enc_cache;
    right_enc_cache = right_enc_ticks;
    right_enc_speed = right_delta * msr_2_sec;

    float left_delta = left_enc_ticks - left_enc_cache;
    left_enc_cache = left_enc_ticks;
    left_enc_speed = left_delta * msr_2_sec;
}


static void left_wheel_a_cb(EXTDriver *extp, expchannel_t channel)
{
    extp = extp; channel = channel;

    if ( palReadPad( GPIOE, LEFT_ENC_A_CH ) )
    {
        if ( palReadPad( GPIOE, LEFT_ENC_B_CH ) )
            left_enc_ticks++;
        else
            left_enc_ticks--;
    }
    else
    {
        if ( palReadPad( GPIOE, LEFT_ENC_B_CH ) )
            left_enc_ticks--;
        else
            left_enc_ticks++;
    }
}


static void right_wheel_a_cb(EXTDriver *extp, expchannel_t channel)
{
    extp = extp; channel = channel;

    if ( palReadPad( GPIOE, RIGHT_ENC_A_CH ) )
    {
        if ( palReadPad( GPIOE, RIGHT_ENC_B_CH ) )
            right_enc_ticks++;
        else
            right_enc_ticks--;
    }
    else
    {
        if ( palReadPad( GPIOE, RIGHT_ENC_B_CH ) )
            right_enc_ticks--;
        else
            right_enc_ticks++;
    }
}


static void left_wheel_b_cb(EXTDriver *extp, expchannel_t channel)
{
    extp = extp; channel = channel;

    if ( palReadPad( GPIOE, LEFT_ENC_B_CH ) )
    {
        if ( palReadPad( GPIOE, LEFT_ENC_A_CH ) )
            left_enc_ticks--;
        else
            left_enc_ticks++;
    }
    else
    {
        if ( palReadPad( GPIOE, LEFT_ENC_A_CH ) )
            left_enc_ticks++;
        else
            left_enc_ticks--;
    }
}


static void right_wheel_b_cb(EXTDriver *extp, expchannel_t channel)
{
    extp = extp; channel = channel;

    if ( palReadPad( GPIOE, RIGHT_ENC_B_CH ) )
    {
        if ( palReadPad( GPIOE, RIGHT_ENC_A_CH ) )
            right_enc_ticks--;
        else
            right_enc_ticks++;
    }
    else
    {
        if ( palReadPad( GPIOE, RIGHT_ENC_A_CH ) )
            right_enc_ticks++;
        else
            right_enc_ticks--;
    }
}
