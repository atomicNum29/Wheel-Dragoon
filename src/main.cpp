#include <Arduino.h>
#include <SPI.h>
#include "MCP41100.hpp"

int real_POT_pin = A0;

MCP41100 lefts(9);
MCP41100 rights(10);

void setup()
{
    lefts.begin();
    rights.begin();
}

void loop()
{
    int adc = analogRead(real_POT_pin);
    adc = map(adc, 0, 1023, 0, 255);
    lefts.setWiper(adc);
    rights.setWiper(adc);
}
