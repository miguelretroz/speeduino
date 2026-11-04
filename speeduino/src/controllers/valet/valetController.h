#pragma once

/**
 * @file
 * @brief Valet mode: performance limits on top of the tune, for when the car is handed to someone else. Fork specific.
 *
 * Three levels (1 Light, 2 Medium, 3 Strong), each with its own limits in page 15: max RPM, max boost (absolute kPa),
 * an ignition retard and whether launch / flat shift stay allowed. A limit of 0 is no limit. The level survives power
 * cycles (EEPROM, outside the pages, so loading a tune never changes it). Selected over the serial ('E' command,
 * group 0x43):
 * - 0x4301..0x4303: switch on / raise to level 1..3. Always accepted.
 * - 0x4300 (off) or a lower level: only within VALET_UNLOCK_WINDOW_MS after 0x4310. The telemetry transmitter checks
 *   the driver's PIN before sending the pair; the ECU only needs the sequence, so a single stray byte never unlocks.
 * A new level whose RPM limit is below the current RPM waits until the engine drops VALET_PENDING_MARGIN_RPM below
 * it (no jolt), and all of the level's limits wait with it. With the engine protection Off the Valet RPM limit still
 * cuts, by ignition only (full cut).
 */

#include "statuses.h"
#include "config_pages.h"

constexpr uint16_t TS_CMD_VALET_OFF       = 0x4300U;
constexpr uint16_t TS_CMD_VALET_UNLOCK    = 0x4310U;
constexpr uint8_t  VALET_LEVELS           = 3U;
constexpr uint16_t VALET_UNLOCK_WINDOW_MS = 2000U;
constexpr uint16_t VALET_PENDING_MARGIN_RPM = 200U;
constexpr uint8_t  VALET_BOOST_HYSTERESIS = 5U; ///< kPa: the open loop boost cut clears this far below the limit

/** @brief Loads the level from EEPROM. Call it after the config has been loaded. */
void initialiseValet(void);

/** @brief Handles an 'E' command. Returns false when it is not from the Valet group (0x43). */
bool handleValetCommand(uint16_t command);

/** @brief Applies a pending level once the RPM allows it. Call it every loop. */
void valetUpdate(const statuses &current, const config15 &page15);

/** @brief The level selected (0 = off) */
uint8_t valetLevel(void);
/** @brief The selected level isn't in force yet (its RPM limit is below the current RPM) */
bool valetPending(void);

/** @name The limits in force (0 = none)
 * @{ */
uint8_t valetRevLimit(const config15 &page15);   ///< RPM/100
uint8_t valetBoostLimit(const config15 &page15); ///< Absolute kPa
uint8_t valetIgnRetard(const config15 &page15);  ///< Degrees
bool valetLaunchAllowed(const config15 &page15);
bool valetFlatShiftAllowed(const config15 &page15);
/** @} */

/** @brief The soft rev limit (RPM/100) with the Valet: as far below the Valet limit as the tune's soft limit is below its hard limit */
uint8_t valetSoftRevLimit(uint8_t softRevLim, uint8_t hardRevLim, const config15 &page15);
/** @brief Closed loop boost: the target, capped by the Valet (kPa) */
uint16_t valetBoostTarget(uint16_t target, const config15 &page15);
/** @brief Open loop boost: at or above the Valet limit the duty goes to 0 (the wastegate on its spring), with hysteresis */
bool valetBoostOverLimit(uint16_t mapKpa, const config15 &page15);
