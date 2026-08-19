#pragma once

#include "RadioLib.h"
#include "meshcore_radio.h"

namespace meshcore_radio {

SX1262 *radio();
RadioLibHal *hal();

}  // namespace meshcore_radio

