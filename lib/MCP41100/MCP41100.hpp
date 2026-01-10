#ifndef MCP41100_HPP
#define MCP41100_HPP

#include <Arduino.h>
#include <SPI.h>

/*
    MCP41100 - header with class declaration only.
    Implementation is placed in MCP41100.cpp to avoid duplicate definitions.
*/

class MCP41100
{
public:
    MCP41100(uint8_t csPin,
             SPIClass &spi = SPI,
             uint32_t spiClock = 1000000,
             uint8_t steps = 255,
             uint8_t cmdWrite = 0x11,
             uint8_t cmdShutdown = 0x20);

    // Initialize SPI and CS pin. Call from setup().
    void begin();

    // Set wiper by raw step (0..steps-1).
    void setWiper(uint8_t value);

    // Set wiper by percentage 0.0..100.0
    void setPercent(float pct);

    // Return cached last written wiper value.
    uint8_t lastWiper() const;

    // Shutdown (true) or enable (false) the device.
    void shutdown(bool enable);

    // Send raw command/data pair.
    void writeRaw(uint8_t command, uint8_t data);

    // Change SPI clock speed at runtime.
    void setSpiClock(uint32_t hz);

private:
    uint8_t _cs;
    SPIClass *_spi;
    SPISettings _spiSettings;
    uint8_t _steps;
    uint8_t _lastValue;
    uint8_t _cmdWrite;
    uint8_t _cmdShutdown;
};

#endif // MCP41100_HPP