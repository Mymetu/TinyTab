
#pragma once

#include <stdbool.h>
#include <stdint.h>

extern uint32_t SDCard_Size;
void SD_Init(void);
bool SD_Get_Usage(uint64_t *total_bytes, uint64_t *used_bytes);
