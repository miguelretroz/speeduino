#pragma once

/**
 * @file
 * @brief Engine protection with an action per condition, memory and a warning output. Fork specific.
 *
 * Upstream's protections (oil pressure, boost, AFR) all do the same thing: limit the RPM to the "limp home"
 * RPM (engineProtectMaxRPM) by cutting fuel and/or spark. Here each condition picks its action:
 * - Limit RPM: as upstream
 * - Warn only: the condition is shown (live data, warning output) but nothing is cut
 * - Stop engine: fuel and/or spark are cut at any RPM, so the engine stops
 * and can keep its action until the engine stops (memory), so a short dip isn't missed.
 *
 * New conditions: hot coolant, hot oil (oilTempController.h), low fuel pressure across the injectors and a
 * sensor fault (sensorFaultController.h). As upstream, the "Protection Cut" (engineProtectType) must be on
 * for anything to be cut; the warnings work regardless.
 */

#include "statuses.h"
#include "config_pages.h"
#include "src/pins/pinNumbers_t.h"

/** @name Protection conditions: bits of the reason byte and of config15::protLatch */
///@{
constexpr uint8_t PROTECT_OIL_PRESSURE  = 0U;
constexpr uint8_t PROTECT_BOOST         = 1U;
constexpr uint8_t PROTECT_AFR           = 2U;
constexpr uint8_t PROTECT_COOLANT_HOT   = 3U;
constexpr uint8_t PROTECT_OIL_HOT       = 4U;
constexpr uint8_t PROTECT_FUEL_PRESSURE = 5U;
constexpr uint8_t PROTECT_SENSOR_FAULT  = 6U;
constexpr uint8_t PROTECT_CONDITIONS    = 7U;
///@}

/** @brief What a protection does when its condition is met (2 bits per condition in the tune) */
enum class ProtectAction : uint8_t
{
  LimitRpm = 0U,    ///< Cut above the limp home RPM, as upstream
  WarnOnly = 1U,    ///< Show it, cut nothing
  StopEngine = 2U,  ///< Cut at any RPM
};

void initialiseProtection(const config15 &page15, const pinNumbers_t &pins);

/**
 * @brief Apply the actions to the protections found by checkEngineProtection(). Call it right after, every loop.
 *
 * Clears the flags of the "warn only" conditions (so nothing is cut), raises the limit/stop flags for the
 * fork's conditions and the remembered ones, and records what is active for the live data.
 */
void protectionResolve(statuses &current, const config6 &page6, const config15 &page15);

/** @brief The fork's conditions (time based), the memory and the warning output. Call at 10Hz. */
void protectionControl(statuses &current, const config6 &page6, const config10 &page10, const config15 &page15);

/**
 * @brief Protection reason byte for the live data/log
 *
 * - Bits 0-6: the conditions active or remembered (@ref PROTECT_OIL_PRESSURE etc.)
 * - Bit 7: at least one condition is remembered (kept until the engine stops)
 */
uint8_t getProtectionLogReason(void);

/**
 * @brief Protection state byte for the live data/log
 *
 * - Bits 0-1: the action in effect (0 none, 1 warn, 2 limit RPM, 3 stop engine)
 * - Bit 2: warning output on
 */
uint8_t getProtectionLogState(void);
