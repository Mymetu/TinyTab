#pragma once

#include <stdint.h>

/* The source font was authored for Arduino, where PROGMEM is provided by
 * Arduino.h. ESP-IDF keeps these immutable tables in flash by default. */
#ifndef PROGMEM
#define PROGMEM
#endif

typedef struct {
    const uint16_t *map;
    const uint8_t *data;
    uint16_t count;
    uint8_t w;
    uint8_t h;
} FontUTF8;
