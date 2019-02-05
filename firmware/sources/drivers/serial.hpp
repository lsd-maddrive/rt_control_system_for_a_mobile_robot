/**
* @file serial.hpp
* @brief Serial driver class
*/
#ifndef SERIAL_HPP
#define SERIAL_HPP

#include <hal.h>


/**
* @brief Serial driver
*/
class Serial
{
public:
    enum SerialNumber_t
    {
        Serial_6 = 1,
        Serial_7 = 2,
    };
    static Serial* GetInstance(SerialNumber_t);
    void Transmit(const uint8_t* buffer, uint8_t size) const;
    void Transmit(const uint8_t* buffer) const;
private:
    Serial() = delete;
    Serial(SerialNumber_t);
    Serial(const Serial&) = delete;
    Serial& operator=(const Serial&) = delete;

    SerialDriver* Driver;
};

#endif /* SERIAL_HPP */
