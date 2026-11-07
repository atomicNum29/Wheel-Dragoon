#pragma once

#include <cassert>
#include <cstddef>
#include <cstdint>
#include <vector>

extern uint64_t fake_clock_us;
inline uint32_t micros() { return static_cast<uint32_t>(fake_clock_us); }
inline unsigned long millis() { return static_cast<uint32_t>(fake_clock_us / 1000u); }
inline void delay(uint32_t ms) { fake_clock_us += uint64_t{ms} * 1000u; }
constexpr int INPUT = 0, OUTPUT = 1, LOW = 0, HIGH = 1, CHANGE = 2, LED_BUILTIN = 13;
inline int *fake_pin_levels() { static int levels[64] = {}; return levels; }
inline int *fake_pin_modes() { static int modes[64] = {}; return modes; }
inline void pinMode(int pin, int mode) { fake_pin_modes()[pin] = mode; }
inline void digitalWrite(int pin, int level) { fake_pin_levels()[pin] = level; }
inline void analogWriteResolution(int) {}
inline void analogWriteFrequency(int, float) {}
inline void analogWrite(int pin, int duty) { fake_pin_levels()[pin] = duty; }
inline int digitalRead(int pin) { return fake_pin_levels()[pin]; }
inline int digitalPinToInterrupt(int pin) { return pin; }
inline void attachInterrupt(int, void (*)(), int) {}
inline void __disable_irq() {}

struct FakeSerial
{
    int space = 0;
    bool reject_write = false;
    uint32_t write_time_us = 0u;
    uint32_t auto_flush_us = 0u; // Optional Teensy partial USB packet model.
    uint64_t flush_deadline_us = 0u;
    uint32_t send_now_calls = 0u;
    int space_after_send_now = 64;
    std::vector<uint8_t> output;
    std::vector<size_t> write_requests;

    void begin(int) {}
    int available() { return 0; }
    int read() { return -1; }
    int availableForWrite() {
        if (auto_flush_us && flush_deadline_us && fake_clock_us >= flush_deadline_us) {
            space = 64;
            flush_deadline_us = 0;
        }
        return space;
    }
    void send_now() {
        ++send_now_calls;
        // Enqueue the current partial packet without changing stream contents.
        if (space > 0 && space < 64) {
            space = space_after_send_now;
            flush_deadline_us = 0;
        }
    }
    size_t write(const uint8_t *data, size_t size)
    {
        assert(size <= static_cast<size_t>(space));
        assert(size <= 64u);
        write_requests.push_back(size);
        if (reject_write)
            return static_cast<size_t>(-1);
        const size_t count = size;
        output.insert(output.end(), data, data + count);
        space -= static_cast<int>(count);
        if (count != 0u)
        {
            fake_clock_us += write_time_us;
            if (auto_flush_us) flush_deadline_us = fake_clock_us + auto_flush_us;
        }
        return count;
    }
};
extern FakeSerial Serial;
