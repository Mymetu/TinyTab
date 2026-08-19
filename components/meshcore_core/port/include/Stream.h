#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>

class Stream {
public:
    virtual ~Stream() = default;

    virtual int available() const = 0;
    virtual int read() = 0;
    virtual size_t write(uint8_t value) = 0;

    virtual size_t write(const uint8_t *source, size_t length) {
        if(source == nullptr) return 0;
        size_t written = 0;
        while(written < length && write(source[written]) == 1) ++written;
        return written;
    }

    size_t readBytes(uint8_t *destination, size_t length) {
        if(destination == nullptr) return 0;
        size_t count = 0;
        while(count < length) {
            const int value = read();
            if(value < 0) break;
            destination[count++] = static_cast<uint8_t>(value);
        }
        return count;
    }

    size_t print(const char *value) {
        if(value == nullptr) return 0;
        return write(reinterpret_cast<const uint8_t *>(value), std::strlen(value));
    }

    size_t print(char value) { return write(static_cast<uint8_t>(value)); }
    size_t println() { return print("\r\n"); }
    size_t println(const char *value) { return print(value) + println(); }
};
