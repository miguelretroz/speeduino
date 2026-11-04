#pragma once

/**
 * @file
 * @brief Valet mode: performance limits on top of the tune, for when the car is handed to someone else. Fork specific.
 *
 * Three levels (1 Light, 2 Medium, 3 Strong), each with its own limits in page 15: max RPM (0 = no limit), max boost
 * as kPa above the ambient pressure (0 = no boost, VALET_BOOST_NO_LIMIT = no limit), an ignition retard and whether
 * launch / flat shift stay allowed. The level survives power cycles (EEPROM, outside the pages, so loading a tune never
 * changes it). Selected over the serial ('E' command, group 0x43):
 * - 0x4301..0x4303: switch on / raise to level 1..3. Always accepted.
 * - 0x4300 (off) or a lower level: only within VALET_UNLOCK_WINDOW_MS after 0x4310. The telemetry transmitter checks
 *   the driver's PIN before sending the pair; the ECU only needs the sequence, so a single stray byte never unlocks.
 * A new level whose RPM limit is below the current RPM waits until the engine drops VALET_PENDING_MARGIN_RPM below
 * it (no jolt), and all of the level's limits wait with it. With the engine protection Off the Valet RPM limit still
 * cuts, by ignition only (full cut).
 *
 * The boost limit is held two ways:
 * - with boost control on, the boost solenoid opens the wastegate a little before the limit (open loop: duty 0 from
 *   VALET_BOOST_SOLENOID_LEAD kPa below it; closed loop: the target capped there and no high "duty when disabled"
 *   below the control threshold, which would spool the turbo against the limit);
 * - always, its own progressive cut, at any RPM: VALET_BOOST_CUT_START kPa above the limit it cuts 25% of the
 *   sparks, rising to a full cut at VALET_BOOST_CUT_FULL kPa above, and once cutting it keeps the minimum until the
 *   pressure drops VALET_BOOST_CUT_HYST kPa below the limit (no on/off chatter). The cut type is the engine
 *   protection's; with the protection Off, ignition only. It isn't an engine protection: it doesn't light the
 *   protection warning, it shows as bit 5 of the live byte 160.
 * Without a boost solenoid the turbo still builds its wastegate spring pressure, so the cut is what holds the limit.
 */

#include "statuses.h"
#include "config_pages.h"

constexpr uint16_t TS_CMD_VALET_OFF       = 0x4300U;
constexpr uint16_t TS_CMD_VALET_UNLOCK    = 0x4310U;
constexpr uint8_t  VALET_LEVELS           = 3U;
constexpr uint16_t VALET_UNLOCK_WINDOW_MS = 2000U;
constexpr uint16_t VALET_PENDING_MARGIN_RPM = 200U;
constexpr uint8_t  VALET_BOOST_NO_LIMIT = 255U;      ///< valetBoostLimit: no boost limit
constexpr uint8_t  VALET_BOOST_SOLENOID_LEAD = 3U;   ///< kPa: open loop, duty 0 from this far below the limit
constexpr uint8_t  VALET_BOOST_HYSTERESIS = 5U;      ///< kPa: and back to the tune's duty this far below that
constexpr uint8_t  VALET_BOOST_CUT_START = 1U;       ///< kPa above the limit: the cut starts (25% of the sparks)
constexpr uint8_t  VALET_BOOST_CUT_FULL = 5U;        ///< kPa above the limit: full cut
constexpr uint8_t  VALET_BOOST_CUT_HYST = 3U;        ///< kPa below the limit: the cut clears
constexpr uint8_t  VALET_BOOST_CUT_MIN_PCT = 25U;    ///< The share of sparks cut at the start (and in the hysteresis band)
constexpr uint8_t  VALET_CUT_FULL = 101U;            ///< valetBoostCutPercent(): a full cut

/** @brief Loads the level from EEPROM. Call it after the config has been loaded. */
void initialiseValet(void);

/** @brief Handles an 'E' command. Returns false when it is not from the Valet group (0x43). */
bool handleValetCommand(uint16_t command);

/** @brief Applies a pending level once the RPM allows it, and updates the boost cut. Call it every loop. */
void valetUpdate(const statuses &current, const config15 &page15);

/** @brief The level selected (0 = off) */
uint8_t valetLevel(void);
/** @brief The selected level isn't in force yet (its RPM limit is below the current RPM) */
bool valetPending(void);

/** @name The limits in force (0 = none)
 * @{ */
uint8_t valetRevLimit(const config15 &page15);   ///< RPM/100
uint8_t valetBoostLimit(const config15 &page15); ///< kPa above the ambient pressure (VALET_BOOST_NO_LIMIT = none)
uint8_t valetIgnRetard(const config15 &page15);  ///< Degrees
bool valetLaunchAllowed(const config15 &page15);
bool valetFlatShiftAllowed(const config15 &page15);
/** @} */

/** @brief The soft rev limit (RPM/100) with the Valet: as far below the Valet limit as the tune's soft limit is below its hard limit */
uint8_t valetSoftRevLimit(uint8_t softRevLim, uint8_t hardRevLim, const config15 &page15);
/** @brief The boost limit in force as absolute kPa: the ambient pressure + the level's limit (0 = no limit) */
uint16_t valetBoostLimitAbs(const statuses &current, const config15 &page15);
/** @brief Closed loop boost: the target, capped a little below the Valet limit (kPa) */
uint16_t valetBoostTarget(uint16_t target, const statuses &current, const config15 &page15);
/** @brief Open loop boost: near the Valet limit the duty goes to 0 (the wastegate on its spring), with hysteresis */
bool valetBoostOverLimit(const statuses &current, const config15 &page15);
/** @brief The share of sparks the Valet boost limit cuts now: 0 none, 1..100 %, VALET_CUT_FULL = all (see the file) */
uint8_t valetBoostCutPercent(void);
