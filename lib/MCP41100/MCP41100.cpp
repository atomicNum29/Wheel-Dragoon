// MCP41100.cpp
// Implementation for MCP41100 declared in MCP41100.hpp

#include "MCP41100.hpp"
#include <Arduino.h>
#include <SPI.h>
#include <math.h>

MCP41100::MCP41100(uint8_t csPin, SPIClass &spi, uint32_t spiClock, uint8_t steps, uint8_t cmdWrite, uint8_t cmdShutdown)
    : _cs(csPin), _spi(&spi), _spiSettings(spiClock, MSBFIRST, SPI_MODE0), _steps(steps), _lastValue(0), _cmdWrite(cmdWrite), _cmdShutdown(cmdShutdown)
{
}

void MCP41100::begin()
{
    pinMode(_cs, OUTPUT);
    digitalWrite(_cs, HIGH);
    _spi->begin();
}

void MCP41100::setWiper(uint8_t value)
{
    if (value >= _steps)
        value = _steps - 1;
    writeRaw(_cmdWrite, value);
    _lastValue = value;
}

void MCP41100::setPercent(float pct)
{
    if (isnan(pct))
        return;
    float clamped = constrain(pct, 0.0f, 100.0f);
    uint8_t v = (uint8_t)roundf(clamped * (float)(_steps - 1) / 100.0f);
    setWiper(v);
}

uint8_t MCP41100::lastWiper() const
{
    return _lastValue;
}

void MCP41100::shutdown(bool enable)
{
    if (enable)
        writeRaw(_cmdShutdown, 0x00);
    else
        writeRaw(_cmdWrite, _lastValue);
}

void MCP41100::writeRaw(uint8_t command, uint8_t data)
{
    _spi->beginTransaction(_spiSettings);
    digitalWrite(_cs, LOW);
    _spi->transfer(command);
    _spi->transfer(data);
    digitalWrite(_cs, HIGH);
    _spi->endTransaction();
}

void MCP41100::setSpiClock(uint32_t hz)
{
    _spiSettings = SPISettings(hz, MSBFIRST, SPI_MODE0);
}
