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
 *
 * Manual hold ('E' command, group 0x42: a button held on the telemetry dash): each HOLD keeps the output on for
 * COLD_START_PUMP_HOLD_MS and has to be renewed; RELEASE (or silence) turns it off. On top of the phases, and only
 * with the system enabled, the key on, a valid coolant reading below coldStartManualMaxClt and less than
 * coldStartManualMaxTime of on time in the current press. The flood clear and low battery cuts still win. Allowed
 * with the engine running (a cold ethanol engine stumbling).
 */

#include "statuses.h"
#include "config_pages.h"
#include "src/pins/pinNumbers_t.h"

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

/** @name Fork: the manual pump hold ('E' command, group 0x42)
 * @{ */
constexpr uint16_t TS_CMD_COLD_START_PUMP_RELEASE = 0x4200U;
constexpr uint16_t TS_CMD_COLD_START_PUMP_HOLD    = 0x4201U;
constexpr uint16_t COLD_START_PUMP_HOLD_MS        = 400U;
/** @brief Handles an 'E' command. Returns false when it is not from the cold start pump group (0x42). */
bool handleColdStartPumpCommand(uint16_t command);
/** @brief The output is on because of the manual hold */
bool coldStartManualActive(void);
/** @} */
