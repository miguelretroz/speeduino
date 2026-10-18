#pragma once

/**
 * @file
 * @brief Cold start auxiliary fuel system for ethanol engines (pump + valve fed by a small gasoline tank).
 *
 * Fork specific. Up to three phases inject auxiliary fuel, each driven by a coolant based curve:
 * - Prime: once per key on (detected from the battery voltage, so USB power alone never injects)
 * - Cranking: pulsed while the engine cranks
 * - After start: pulsed for a short time once the engine starts
 *
 * Each attempt is counted in EEPROM, so the count survives switching the key off. The dose drops
 * with each attempt, until the engine runs long enough or the driver performs the reset gesture.
 * Holding the throttle above the flood clear threshold cuts the output in every phase.
 */

#include "statuses.h"
#include "config_pages.h"
#include "src/pins/pinNumbers_t.h"

/** @brief Set the cold start fields of page 15 to safe defaults (disabled) if they have never been initialised */
void initialiseColdStartConfig(config15 &page15);

void initialiseColdStart(statuses &current, const config15 &page15, const pinNumbers_t &pins);

void coldStartControl(statuses &current, const config4 &page4, const config15 &page15);

/**
 * @brief Cold start status byte for the live data/log
 *
 * - Bits 0-1: phase (0 idle, 1 prime, 2 cranking, 3 after start)
 * - Bits 2-4: attempts made without the engine starting
 * - Bit 5: output cut by low battery
 */
uint8_t getColdStartLogStatus(void);
