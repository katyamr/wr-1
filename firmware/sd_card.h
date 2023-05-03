#pragma once

#include <stdint.h>

namespace sd_card {
using date_time_callback_t = void(uint16_t* date, uint16_t* time);

bool init();
void date_time(date_time_callback_t *cb);

}
