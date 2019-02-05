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


/// Static objects:
static Serial* serial6;
static Serial* serial7;

static bool isSerialSixthBusy = false;
static bool isSerialSeventhBusy = false;


/**
* @brief Init Serial
* @param[in] - Serial number
* @note Instance of classical singleton with several instances
*/
Serial* Serial::GetInstance(SerialNumber_t serialNumber)
{
    const SerialConfig sdcfg =
    {
         .speed = 115200,
         .cr1 = 0,
         .cr2 = 0,
         .cr3 = 0
    };

    if((serialNumber == Serial_6) && (isSerialSixthBusy == false))
    {
        sdStart( &SD6, &sdcfg );
        palSetPadMode( GPIOG, 14, PAL_MODE_ALTERNATE(8) );  // TX
        palSetPadMode( GPIOG, 9, PAL_MODE_ALTERNATE(8) );   // RX
        isSerialSeventhBusy = true;
        return serial6;
    }
    else if((serialNumber == Serial_7) && (isSerialSeventhBusy == false))
    {
        sdStart( &SD7, &sdcfg );
        palSetPadMode( GPIOE, 8, PAL_MODE_ALTERNATE(8) );   // TX
        palSetPadMode( GPIOE, 7, PAL_MODE_ALTERNATE(8) );   // RX
        isSerialSeventhBusy = true;
        return serial7;
    }
    return nullptr;
}


/**
* @brief Constructor of Serial
* @param[in] - Serial number
* @note Instance of classical singleton with several instances
*/
Serial::Serial(SerialNumber_t serialNumber)
{
    if(serialNumber == Serial_6)
    {
        Driver = &SD6;
    }
    else if(serialNumber == Serial_7)
    {
        Driver = &SD7;
    }
}


/**
* @brief Transmit buffer to port
* @param[in] buffer - buffer with message
* @param[in] size - buffer size
*/
void Serial::Transmit(const uint8_t* buffer, uint8_t size) const
{
    if(Driver != nullptr)
    {
        for(uint_fast8_t byteIndex = 0; byteIndex < size; byteIndex++)
        {
            sdPut( Driver, *buffer++ );
        }
    }
}


/**
* @brief Transmit c-string to port
* @param[in] buffer - c-string with message
* @note buffer must end with a character '\n'
* For safety the buffer length must be less than 128.
*/
void Serial::Transmit(const uint8_t* buffer) const
{
    if(Driver != nullptr)
    {
        uint8_t iteration = 0;
        while((*buffer != '\n') && (iteration++ < 128))
        {
            sdPut( Driver, *buffer++ );
        }
        sdPut( Driver, '\n' );
        sdPut( Driver, '\r' );
    }
}


