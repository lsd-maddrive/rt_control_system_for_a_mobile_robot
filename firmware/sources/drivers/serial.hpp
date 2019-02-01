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
    enum
    {
        SERIAL_CHARACTER =          0x00,
        SERIAL_BYTE_ARRAY =         0x01,
        SERIAL_UINT32 =             0x02,
        SERIAL_STRING =             0x03,
        SERIAL_FORMATTED_STRING =   0x04,
        SERIAL_MATLAB =             0x05,
    };
    Serial(): SendType(SERIAL_CHARACTER) {}
    void Init();
    void Transmit(const uint8_t* buffer, uint8_t size) const;
    void Transmit(const uint8_t* buffer) const;
    void Do() const;
private:
    const uint8_t SendType;
};

#endif /* SERIAL_HPP */
