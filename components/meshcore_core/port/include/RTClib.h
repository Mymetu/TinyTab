#pragma once

#include <cstdint>

class DateTime {
public:
    explicit DateTime(uint32_t timestamp = 0) : timestamp_(timestamp) {}
    uint32_t unixtime() const { return timestamp_; }

private:
    uint32_t timestamp_;
};
