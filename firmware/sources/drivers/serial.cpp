/**
* @file serial.cpp
* @brief Serial driver implementation
*/
#include "serial.hpp"
#include <ch.h>


/*
* Short description:
* Requirements:
* halconf.h -  Enable HAL_USE_SERIAL
*              To change the size of the buffer, change SERIAL_BUFFERS_SIZE
* mcuconf.h -  Choose required serial STM32_SERIAL_USE_*
*/


void Serial::Init()
{
    // Set Baudrate, directly number.
    // No need to set CR1, CCR2 and CCR3.
    // If need to set one of registers use USART_CR1_*, USART_CR2_* or USART_CR3_*.
    // CRM says that USART_CR2_LINEN enables error detection,
    // so this should work without this USART_CR2_LINEN
    const SerialConfig sdcfg =
    {
      .speed = 115200,
      .cr1 = 0,
      .cr2 = 0,
      .cr3 = 0
    };

    // As 6th driver is used, use SD6 structure for driver functions
    sdStart( &SD6, &sdcfg );

    // https://os.mbed.com/platforms/ST-Nucleo-F767ZI/
    // serial 6th driver is on PG_14, PG_9
    // alternate function is 8th (check datasheet)
    palSetPadMode( GPIOG, 14, PAL_MODE_ALTERNATE(8) );  // TX = PG_14
    palSetPadMode( GPIOG, 9, PAL_MODE_ALTERNATE(8) );   // RX = PG_9
}


void Serial::Transmit(const uint8_t* buffer, uint8_t size) const
{
    for(uint_fast8_t byteIndex = 0; byteIndex < size; byteIndex++)
    {
        sdPut( &SD6, *buffer++ );
    }
}


void Serial::Transmit(const uint8_t* buffer) const
{
    while(*buffer != '\n')
    {
        sdPut( &SD6, *buffer++ );
    }
    sdPut( &SD6, '\n' );
    sdPut( &SD6, '\r' );
}


void Serial::Do() const
{

}
