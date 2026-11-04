/** @file
 * Valet mode. See valetController.h for an overview. The limits are at page 15 bytes 149-160 (of type @ref config15).
 */
#include "valetController.h"
#include "valetController_details.h"
#include "../../../globals.h"
#include "../../../storage.h"
#include "../../../unit_testing.h"
#include "../../../preprocessor.h"

using namespace valet::details;

TESTABLE_STATIC state_t valetState;

namespace valet {

namespace details {

void __attribute__((optimize("Os"))) setDefaults(config15 &page15)
{
  constexpr uint8_t revLimit[VALET_LEVELS] = { 50U, 40U, 30U };      // 5000 / 4000 / 3000 RPM
  constexpr uint8_t boostLimit[VALET_LEVELS] = { 150U, 120U, 100U }; // kPa: ~0.5 bar, ~0.2 bar, no boost
  constexpr uint8_t ignRetard[VALET_LEVELS] = { 0U, 3U, 6U };
  for (uint8_t i = 0U; i < VALET_LEVELS; ++i)
  {
    page15.valetRevLimit[i] = revLimit[i];
    page15.valetBoostLimit[i] = boostLimit[i];
    page15.valetIgnRetard[i] = ignRetard[i];
    page15.valetFlags[i] = 0U; // Launch and flat shift blocked
  }
}

} // namespace details

} // namespace valet

static inline uint8_t decodeLevel(uint8_t stored)
{
  uint8_t level = (uint8_t)(stored & 0x0FU);
  return (((stored & 0xF0U) == EEPROM_MARK) && (level <= VALET_LEVELS)) ? level : 0U;
}

static void setLevel(uint8_t level)
{
  if (level != valetState.level)
  {
    valetState.level = level;
    saveValetLevel((level != 0U) ? (uint8_t)(EEPROM_MARK | level) : 0U); // Written only on change
  }
}

TESTABLE_STATIC bool valetCommandCore(uint16_t command, uint32_t nowMs)
{
  if ((command >> 8U) != (TS_CMD_VALET_OFF >> 8U)) { return false; }
  if (command == TS_CMD_VALET_UNLOCK)
  {
    valetState.unlockArmed = true;
    valetState.unlockUntilMs = nowMs + VALET_UNLOCK_WINDOW_MS;
    return true;
  }
  uint8_t requested = (uint8_t)(command & 0xFFU);
  if (requested <= VALET_LEVELS)
  {
    bool unlocked = valetState.unlockArmed && ((int32_t)(nowMs - valetState.unlockUntilMs) < 0);
    // Switching on or raising the level is always allowed; lowering it or switching off needs the unlock first
    if ((requested > valetState.level) || unlocked) { setLevel(requested); }
    valetState.unlockArmed = false; // The window serves a single selection
  }
  return true; // Anything else in the group: handled, no effect
}

static inline uint8_t limitOf(const uint8_t (&values)[VALET_LEVELS], uint8_t level)
{
  return (level != 0U) ? values[level - 1U] : 0U;
}

TESTABLE_STATIC void valetUpdateCore(uint16_t rpm, const config15 &page15)
{
  if (valetState.inForce != valetState.level)
  {
    uint8_t limit = limitOf(page15.valetRevLimit, valetState.level);
    if ((rpm == 0U) || (limit == 0U) || (((uint32_t)rpm + VALET_PENDING_MARGIN_RPM) <= ((uint32_t)limit * 100U)))
    {
      valetState.inForce = valetState.level;
    }
  }
}

void initialiseValet(void)
{
  valetState = state_t();
  valetState.level = decodeLevel(loadValetLevel());
  valetState.inForce = valetState.level; // At power on the engine is stopped: no need to wait
}

uint8_t valetLevel(void) { return valetState.level; }
bool valetPending(void) { return valetState.inForce != valetState.level; }

uint8_t valetRevLimit(const config15 &page15) { return limitOf(page15.valetRevLimit, valetState.inForce); }
uint8_t valetBoostLimit(const config15 &page15) { return limitOf(page15.valetBoostLimit, valetState.inForce); }
uint8_t valetIgnRetard(const config15 &page15) { return limitOf(page15.valetIgnRetard, valetState.inForce); }
bool valetLaunchAllowed(const config15 &page15) { return (valetState.inForce == 0U) || ((limitOf(page15.valetFlags, valetState.inForce) & 0x01U) != 0U); }
bool valetFlatShiftAllowed(const config15 &page15) { return (valetState.inForce == 0U) || ((limitOf(page15.valetFlags, valetState.inForce) & 0x02U) != 0U); }

uint8_t valetSoftRevLimit(uint8_t softRevLim, uint8_t hardRevLim, const config15 &page15)
{
  uint8_t limit = valetRevLimit(page15);
  if (limit == 0U) { return softRevLim; }
  uint8_t gap = (hardRevLim > softRevLim) ? (uint8_t)(hardRevLim - softRevLim) : 0U;
  uint8_t soft = (limit > gap) ? (uint8_t)(limit - gap) : 0U;
  return (soft < softRevLim) ? soft : softRevLim;
}

uint16_t valetBoostTarget(uint16_t target, const config15 &page15)
{
  uint8_t limit = valetBoostLimit(page15);
  return ((limit != 0U) && (target > limit)) ? limit : target;
}

bool valetBoostOverLimit(uint16_t mapKpa, const config15 &page15)
{
  uint8_t limit = valetBoostLimit(page15);
  if (limit == 0U) { valetState.boostOver = false; }
  else if (mapKpa >= limit) { valetState.boostOver = true; }
  else if (((uint16_t)mapKpa + VALET_BOOST_HYSTERESIS) < limit) { valetState.boostOver = false; }
  else { /* Hysteresis band: keep the last state */ }
  return valetState.boostOver;
}

// LCOV_EXCL_START
bool handleValetCommand(uint16_t command)
{
  return valetCommandCore(command, millis());
}

void valetUpdate(const statuses &current, const config15 &page15)
{
  valetUpdateCore(current.RPM, page15);
}
// LCOV_EXCL_STOP
