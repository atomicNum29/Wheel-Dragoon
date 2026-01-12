#include <Arduino.h>
#include <SPI.h>
#include "MCP41100.hpp"

const int left_speed_controller_pin = 5;
const int right_speed_controller_pin = 6;
const int brake_controller_pin = 7;

const int left_speed_control_pin = 9;
const int right_speed_control_pin = 10;

const int left_dir_control_pin = 11;
const int right_dir_control_pin = 12;
const int brake_control_pin = 13;
const int stop_control_pin = 14;

MCP41100 lefts(left_speed_control_pin);
MCP41100 rights(right_speed_control_pin);

volatile unsigned int left_pulseWidth = 0;
volatile unsigned int right_pulseWidth = 0;
volatile unsigned int isbraked = true;

void left_decodePWM();
void right_decodePWM();
void brake_decodePWM();

void setup()
{
    lefts.begin();
    rights.begin();

    attachInterrupt(digitalPinToInterrupt(left_speed_controller_pin), left_decodePWM, CHANGE);
    attachInterrupt(digitalPinToInterrupt(right_speed_controller_pin), right_decodePWM, CHANGE);
    attachInterrupt(digitalPinToInterrupt(brake_controller_pin), brake_decodePWM, CHANGE);

    pinMode(left_dir_control_pin, OUTPUT);
    pinMode(right_dir_control_pin, OUTPUT);
    pinMode(brake_control_pin, OUTPUT);
    pinMode(stop_control_pin, OUTPUT);

    digitalWrite(stop_control_pin, LOW);
    digitalWrite(brake_control_pin, LOW);
    digitalWrite(left_dir_control_pin, LOW);
    digitalWrite(right_dir_control_pin, HIGH);
}

void loop()
{
    if (isbraked)
    {
        digitalWrite(brake_control_pin, HIGH);
    }
    else
    {
        digitalWrite(brake_control_pin, LOW);
    }
    digitalWrite(stop_control_pin, HIGH);

    unsigned int left_speed = left_pulseWidth;
    unsigned int right_speed = right_pulseWidth;
    if (left_speed < 1500)
    {
        digitalWrite(left_dir_control_pin, LOW);
        left_speed = 1500 + 1500 - left_speed;
    }
    else
    {
        digitalWrite(left_dir_control_pin, HIGH);
    }
    if (right_speed < 1500)
    {
        digitalWrite(right_dir_control_pin, HIGH);
        right_speed = 1500 + 1500 - right_speed;
    }
    else
    {
        digitalWrite(right_dir_control_pin, LOW);
    }

    left_speed = map(left_speed, 1500, 2000, 0, 100);
    right_speed = map(right_speed, 1500, 2000, 0, 100);

    lefts.setWiper(left_speed);
    rights.setWiper(right_speed);
}

void left_decodePWM()
{
    static unsigned long prevTime = 0;
    static unsigned long dataA[10] = {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500};
    static unsigned int idx = 0;
    static unsigned long sum = 15000;

    if ((prevTime != 0) && !digitalRead(left_speed_controller_pin))
    {
        unsigned long tmp = micros() - prevTime;
        if (tmp > 2000)
            return;
        sum += tmp - dataA[idx];
        dataA[idx] = tmp;
        idx = (idx + 1) % 10;
        left_pulseWidth = sum / 10;
        prevTime = 0;
    }
    else
    {
        prevTime = micros();
    }
}

void right_decodePWM()
{
    static unsigned long prevTime = 0;
    static unsigned long dataA[10] = {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500};
    static unsigned int idx = 0;
    static unsigned long sum = 15000;

    if ((prevTime != 0) && !digitalRead(right_speed_controller_pin))
    {
        unsigned long tmp = micros() - prevTime;
        if (tmp > 2000)
            return;
        sum += tmp - dataA[idx];
        dataA[idx] = tmp;
        idx = (idx + 1) % 10;
        right_pulseWidth = sum / 10;
        prevTime = 0;
    }
    else
    {
        prevTime = micros();
    }
}

void brake_decodePWM()
{
    static unsigned long prevTime = 0;
    static volatile unsigned int pulseWidth = 0;

    if ((prevTime != 0) && !digitalRead(brake_controller_pin))
    {
        pulseWidth = micros() - prevTime;

        if (pulseWidth > 1700)
            isbraked = true;
        else
            isbraked = false;
    }
    else
    {
        prevTime = micros();
    }
}
