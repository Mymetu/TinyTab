#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <type_traits>

#include "Stream.h"

#define HIGH 1
#define LOW 0
#define INPUT 1
#define OUTPUT 2
#define INPUT_PULLUP 3
#define INPUT_PULLDOWN 4
#define HEX 16
#define DEC 10
#define PROGMEM
#define F(value) value

using byte = uint8_t;

unsigned long millis(void);
unsigned long micros(void);
void delay(unsigned long milliseconds);
void delayMicroseconds(unsigned int microseconds);
void randomSeed(unsigned long seed);
long random(long maximum);
long random(long minimum, long maximum);
char *ltoa(long value, char *buffer, int base);
char *ultoa(unsigned long value, char *buffer, int base);

template <typename T, typename L, typename H>
constexpr std::common_type_t<T, L, H> constrain(T value, L low, H high) {
    using Result = std::common_type_t<T, L, H>;
    const Result converted = static_cast<Result>(value);
    const Result minimum = static_cast<Result>(low);
    const Result maximum = static_cast<Result>(high);
    return converted < minimum ? minimum : (converted > maximum ? maximum : converted);
}

class IdfSerialCompat : public Stream {
public:
    void begin(unsigned long) {}
    int available() const override { return 0; }
    int read() override { return -1; }
    size_t write(uint8_t value) override;

    size_t print(const char *value);
    size_t print(char value);
    size_t print(int value, int base = DEC);
    size_t print(unsigned value, int base = DEC);
    size_t println(const char *value = "");
    size_t println(int value, int base = DEC);

    template <typename... Args>
    int printf(const char *format, Args... args) {
        return std::printf(format, args...);
    }
};

extern IdfSerialCompat Serial;

using std::max;
using std::min;
