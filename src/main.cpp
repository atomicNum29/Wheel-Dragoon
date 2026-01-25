#include <Arduino.h>
#include <SPI.h>
#include "MCP41100.hpp"

// remote control signal pins
const int w_speed_controller_pin = 0;
const int v_speed_controller_pin = 1;

volatile unsigned int v_pulseWidth = 0;
volatile unsigned int w_pulseWidth = 0;

void v_decodePWM();
void w_decodePWM();

// motor speed control pins
const int lf_speed_control_pin = 3;
const int lr_speed_control_pin = 4;
const int rr_speed_control_pin = 5;
const int rf_speed_control_pin = 6;

// motor control feedback pins
const int lf_wheel_pulse_pin = 7;
const int lr_wheel_pulse_pin = 8;
const int rr_wheel_pulse_pin = 9;
const int rf_wheel_pulse_pin = 10;
// volatile unsigned int lf_wheel_pulse_count = 0;
// volatile unsigned int lr_wheel_pulse_count = 0;
// volatile unsigned int rr_wheel_pulse_count = 0;
// volatile unsigned int rf_wheel_pulse_count = 0;

// motor direction control pins
const int left_dir_control_pin = 14;
const int right_dir_control_pin = 15;

// car width and wheel radius
const double W = 0.42; // m
const double R = 0.14; // m

MCP41100 lf_motor(lf_speed_control_pin);
MCP41100 lr_motor(lr_speed_control_pin);
MCP41100 rf_motor(rf_speed_control_pin);
MCP41100 rr_motor(rr_speed_control_pin);

void setup()
{
    Serial.begin(115200);

    lf_motor.begin();
    lr_motor.begin();
    rf_motor.begin();
    rr_motor.begin();

    attachInterrupt(digitalPinToInterrupt(v_speed_controller_pin), v_decodePWM, CHANGE);
    attachInterrupt(digitalPinToInterrupt(w_speed_controller_pin), w_decodePWM, CHANGE);

    pinMode(left_dir_control_pin, OUTPUT);
    pinMode(right_dir_control_pin, OUTPUT);
}

void loop()
{

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

    lf_motor.setWiper(left_control_signal);
    lr_motor.setWiper(left_control_signal);
    rf_motor.setWiper(right_control_signal);
    rr_motor.setWiper(right_control_signal);
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
