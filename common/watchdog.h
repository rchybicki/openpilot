#pragma once

#include <cstdint>

bool watchdog_kick(uint64_t ts);
bool watchdog_set_phase(const char *phase);
