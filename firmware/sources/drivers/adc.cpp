/**
* @file adc.cpp
* @brief Adc driver implementation
* 
* Requirements for adc work:       
* - halconf.h -  Enable HAL   
* - mcuconf.h -  Choose required ADC unit STM32_ADC_USE_ADC and   
*                set STM32_ADC_ADCPRE for ADC clock divider (2, 4, 6 or 8)
*/

#include "adc.hpp"
#include <ch.h>
#include <chprintf.h>
#include <cstring>


/**
* Buffer must be static with external binding (static public) because method 
* adcCallback must have access to it.
*/
adcsample_t Adc::Buffer[Adc::ADC1_CHANNELS_QUANTITY * Adc::ADC1_BUFFER_DEPTH];


/**
 * @brief Callback function
 * @param[in] adcp - adc driver
 * @param[in] buffer - buffer with adc data
 * @param[in] n - buffer size
 */
static void adcCallback(ADCDriver* adcp, adcsample_t* buffer, size_t n)
{
    adcp = adcp; n = n;

    // Full buffer
    if ( buffer != Adc::Buffer )
    {
        // Pointer not equal to begin of buffer - second half of buffer is filled
    }
    else
    {
        // Pointer is equal to begin of the buffer - first half is filled
    }
}


/*
 * ADC needs no configuration - only conversion group setup
 */
static const ADCConversionGroup adc1conf = {
  /*
   * Mode chose
   *   circular (true)  - filling of the buffer is looped (after end reaced - start from beginning)
   *   linear   (false) - buffer is fully filled once (callback not called on half filling in linear)
   */
  .circular     = true,

  // Set number of channels
  .num_channels = Adc::ADC1_CHANNELS_QUANTITY,
  /*
   * Set callback function to call
   */
  .end_cb       = adcCallback,
  /*
   * Callback for function if error occurs
   */
  .error_cb     = NULL,
  /*
   * Boooring registers =)
   * cr1 - here we can set resolution, but by default it is 12 bit and it is enough
   */
  .cr1          = 0,
  /*
   * cr2 - For simple continuous conversion it is not required
   */
  .cr2          = 0,
  /*
   * Next registers are really important!
   * smpr1 and smpr2 set sample rate
   * can be 3, 15, 28, 56, 84, 112, 144, 480
   * defined with macros ADC_SAMPLE_*
   * set amount of time for sampling
   * Ex:
   *   ADC_SAMPLE_112 means (112 * Tadc) = Time for one channel sampling
   *   Max ADC freq = 108 MHz (for this microcontroller)
   *   Divider (STM32_ADC_ADCPRE) is 4 now
   *   So fadc = 108 / 4 = 27 MHz
   *   Tadc = 1/fadc
   * You choose to set smpr1 or smpr2 by name of ADC_SMPR*_SMP_AN*() function
   * In our microcontroller: smpr1 - ADC 10-15
   *                         smpr2 - ADC 0-9
   * If you want to set sampling for 6th channel find function ADC_SMPR?_SMP_AN6()
   * and see if there is 1 or 2 in ADC_SMPR?_SMP_AN6() name
   * ADC_SMPR2_SMP_AN3() - set sampling for chennel 3 (AN3), write to register smpr2 (SMPR2)
   */
  .smpr1        = ADC_SMPR1_SMP_AN10(ADC_SAMPLE_480),
  .smpr2        = ADC_SMPR2_SMP_AN3(ADC_SAMPLE_480),
  /*
   * These are very valuable too
   * sqr1, sqr2, sqr3 - set sequence of channels
   * Function: ADC_SQR*_SQ*_N()
   * For example you want to set sequence: ch2, ch5, ch1
   * You should use function ADC_SQR?_SQ1_N(), ADC_SQR?_SQ2_N(), ADC_SQR?_SQ3_N()
   * Name: SQ1 - means sequence order 1, SQ2 - order 2, ...
   *       SQR2 - write to register sqr2
   * In out microcontroller: sequence (SQ*) 1-6   - sqr3
   *                         sequence (SQ*) 7-12  - sqr2
   *                         sequence (SQ*) 13-16 - sqr1
   *
   * Channels are refered through ADC_CHANNEL_IN* macros
   */
  .sqr1         = 0,
  .sqr2         = 0,
  /*
   * In example: sequence ch3, ch10
   * To gather some instructions for one register use logical OR ('|')
   */
  .sqr3         = ADC_SQR3_SQ1_N(ADC_CHANNEL_IN3) | ADC_SQR3_SQ2_N(ADC_CHANNEL_IN10)
};



void Adc::Start()
{
    adcInit();
    adcStart(&ADCD1, NULL);
    palSetLineMode( LINE_ADC123_IN10, PAL_MODE_INPUT_ANALOG );  // PC0
    palSetLineMode( LINE_ADC123_IN3, PAL_MODE_INPUT_ANALOG );   // PA3
    adcStartConversion(&ADCD1, &adc1conf, &Buffer[0], ADC1_BUFFER_DEPTH);
}
