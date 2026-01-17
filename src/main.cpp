#include <Arduino.h>
#include <SPI.h>
#include "MCP41100.hpp"

const int w_speed_controller_pin = 5;
const int v_speed_controller_pin = 6;
const int brake_controller_pin = 7;

const int left_speed_control_pin = 9;
const int right_speed_control_pin = 10;

const int left_dir_control_pin = 12;
const int left_brake_control_pin = 13;
const int right_dir_control_pin = 14;
const int right_brake_control_pin = 15;

const double W = 0.42; // m
const double R = 0.14; // m

MCP41100 lefts(left_speed_control_pin);
MCP41100 rights(right_speed_control_pin);

volatile unsigned int v_pulseWidth = 0;
volatile unsigned int w_pulseWidth = 0;
volatile unsigned int isbraked = true;

void v_decodePWM();
void w_decodePWM();
void brake_decodePWM();

void setup()
{
    Serial.begin(115200);

    lefts.begin();
    rights.begin();

    attachInterrupt(digitalPinToInterrupt(v_speed_controller_pin), v_decodePWM, CHANGE);
    attachInterrupt(digitalPinToInterrupt(w_speed_controller_pin), w_decodePWM, CHANGE);
    attachInterrupt(digitalPinToInterrupt(brake_controller_pin), brake_decodePWM, CHANGE);

    pinMode(left_dir_control_pin, OUTPUT);
    pinMode(left_brake_control_pin, OUTPUT);
    pinMode(right_dir_control_pin, OUTPUT);
    pinMode(right_brake_control_pin, OUTPUT);

    digitalWrite(left_brake_control_pin, HIGH);
    digitalWrite(right_brake_control_pin, HIGH);
    digitalWrite(left_dir_control_pin, LOW);
    digitalWrite(right_dir_control_pin, HIGH);
}

void loop()
{
    if (isbraked)
    {
        digitalWrite(left_brake_control_pin, HIGH);
        digitalWrite(right_brake_control_pin, HIGH);
    }
    else
    {
        digitalWrite(left_brake_control_pin, LOW);
        digitalWrite(right_brake_control_pin, LOW);
    }

    double v_velocity = v_pulseWidth;
    double w_velocity = w_pulseWidth;

    v_velocity -= 1500; // 양수면 전진
    w_velocity -= 1500; // 양수면 좌선회 (우측 바퀴가 +)

    v_velocity /= 250; // -500~500 범위를 -2~2 범위로 줄임. (m/s)
    w_velocity /= 100; // -500~500 범위를 -5~5 범위로 줄임. (rad/s)

    double left_velocity = 0;
    double right_velocity = 0;

    left_velocity = v_velocity - w_velocity * W / 2;
    right_velocity = v_velocity + w_velocity * W / 2;

    int left_control_signal = left_velocity * 75;
    int right_control_signal = right_velocity * 75;

    // Serial.print(">v:");
    // Serial.println(v_velocity);
    // Serial.print(">w:");
    // Serial.println(w_velocity);
    // Serial.print(">left_velocity:");
    // Serial.println(left_velocity);
    // Serial.print(">right_velocity:");
    // Serial.println(right_velocity);
    // Serial.print(">left_control_signal:");
    // Serial.println(left_control_signal);
    // Serial.print(">right_control_signal:");
    // Serial.println(right_control_signal);
    // delay(10);

    if (left_control_signal < 0)
    {
        digitalWrite(left_dir_control_pin, LOW);
        left_control_signal *= -1;
    }
    else
    {
        digitalWrite(left_dir_control_pin, HIGH);
    }

    if (right_control_signal < 0)
    {
        digitalWrite(right_dir_control_pin, HIGH);
        right_control_signal *= -1;
    }
    else
    {
        digitalWrite(right_dir_control_pin, LOW);
    }

    lefts.setWiper(left_control_signal);
    rights.setWiper(right_control_signal);
}

void v_decodePWM()
{
    static unsigned long prevTime = 0;
    static unsigned long dataA[10] = {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500};
    static unsigned int idx = 0;
    static unsigned long sum = 15000;

    if ((prevTime != 0) && !digitalRead(v_speed_controller_pin))
    {
        unsigned long tmp = micros() - prevTime;
        if (tmp > 2000)
            return;
        sum += tmp - dataA[idx];
        dataA[idx] = tmp;
        idx = (idx + 1) % 10;
        v_pulseWidth = sum / 10;
        prevTime = 0;
    }
    else
    {
        prevTime = micros();
    }
}

void w_decodePWM()
{
    static unsigned long prevTime = 0;
    static unsigned long dataA[10] = {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500};
    static unsigned int idx = 0;
    static unsigned long sum = 15000;

    if ((prevTime != 0) && !digitalRead(w_speed_controller_pin))
    {
        unsigned long tmp = micros() - prevTime;
        if (tmp > 2000)
            return;
        sum += tmp - dataA[idx];
        dataA[idx] = tmp;
        idx = (idx + 1) % 10;
        w_pulseWidth = sum / 10;
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
