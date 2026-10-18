/** @file
 * Cold start auxiliary fuel system. See coldStartController.h for an overview.
 * The config is found at the end of page 15 (of type @ref config15).
 */
#include "coldStartController.h"
#include "coldStartController_details.h"
#include "../../../globals.h"
#include "../../../storage.h"
#include "../../../pages.h"
#include "../../../units.h"
#include "../../../unit_testing.h"
#include "../../../elapsed_time.h"
#include "../../../preprocessor.h"

using namespace coldStartController::details;

/** @brief Battery voltage below which the key is considered off (Unit: 0.1V). Matches the USB power detection in the main loop. */
static constexpr uint8_t KEY_OFF_BATTERY = 55U;
/** @brief Hysteresis for the low battery cut (Unit: 0.1V) */
static constexpr uint8_t LOW_BATTERY_HYSTERESIS = 5U;
/**
 * @brief Saved to EEPROM once the cold start config fields have been initialised. Acts as the version
 * of the fork's config layout, since the upstream data version can't be used.
 */
static constexpr uint8_t CONFIG_MARKER_V1 = 0xC5U; // Bytes 206-255
static constexpr uint8_t CONFIG_MARKER_V2 = 0xC6U; // Added the low battery cut and key on reset (bytes 204-205)

TESTABLE_CONSTEXPR table2D_u8_u8_6 coldStartPrimeTable(&configPage15.coldStartPrimeBins, &configPage15.coldStartPrimeValues);
TESTABLE_CONSTEXPR table2D_u8_u8_6 coldStartCrankTable(&configPage15.coldStartCrankBins, &configPage15.coldStartCrankValues);
TESTABLE_CONSTEXPR table2D_u8_u8_6 coldStartAfterTable(&configPage15.coldStartAfterBins, &configPage15.coldStartAfterValues);

TESTABLE_STATIC state_t coldStartState;

namespace coldStartController {

namespace details {

uint8_t getDosePercent(const config15 &page15, uint8_t attempts)
{
  return (attempts < DOSE_ATTEMPTS) ? page15.coldStartDosePct[attempts] : 0U;
}

void __attribute__((optimize("Os"))) setDefaults(config15 &page15)
{
  // Same coolant bins (C) for all 3 curves
  constexpr int8_t bins[] = { 0, 10, 20, 30, 35, 40 };
  // Pulse on time while cranking (Unit: 10mS). Nothing above 40C.
  constexpr uint8_t crankPulse[] = { 30U, 30U, 30U, 30U, 20U, 0U };
  static_assert(_countof(bins)==_countof(config15::coldStartCrankBins), "Bin count mismatch");

  page15.coldStartEnable = 0U;
  page15.coldStartInv = 0U;
  page15.coldStartUnused1 = 0U;
  page15.coldStartPin = 0U;
  page15.coldStartUnused2 = 0U;
  for (uint8_t i = 0U; i < _countof(bins); ++i)
  {
    page15.coldStartPrimeBins[i] = temperatureAddOffset(bins[i]);
    page15.coldStartPrimeValues[i] = 0U; // Prime is off by default
    page15.coldStartCrankBins[i] = temperatureAddOffset(bins[i]);
    page15.coldStartCrankValues[i] = crankPulse[i];
    page15.coldStartAfterBins[i] = temperatureAddOffset(bins[i]);
    page15.coldStartAfterValues[i] = 0U; // After start is off by default
  }
  page15.coldStartPulsePeriod = 100U;  // 1S
  page15.coldStartPulseMin = 20U;      // 200mS
  page15.coldStartCrankMaxTime = 50U;  // 5S
  page15.coldStartMinBattery = 100U;   // 10V
  page15.coldStartMinValidClt = temperatureAddOffset(-30);
  page15.coldStartDosePct[0] = 100U;
  page15.coldStartDosePct[1] = 60U;
  page15.coldStartDosePct[2] = 30U;
  page15.coldStartDosePct[3] = 0U;
  page15.coldStartRunReset = 10U;      // 10S
  page15.coldStartResetTps = 180U;     // 90%
  page15.coldStartResetTime = 30U;     // 3S
  setV2Defaults(page15);
}

void __attribute__((optimize("Os"))) setV2Defaults(config15 &page15)
{
  page15.coldStartCutBattery = 80U;    // 8V
  page15.coldStartKeyOnReset = 10U;    // 10 minutes
}

} // namespace details

} // namespace coldStartController

void __attribute__((optimize("Os"))) initialiseColdStartConfig(config15 &page15)
{
  uint8_t marker = loadColdStartMarker();
  if (marker != CONFIG_MARKER_V2)
  {
    if (marker == CONFIG_MARKER_V1)
    {
      // Keep the existing settings, only initialise the new fields
      setV2Defaults(page15);
    }
    else
    {
      setDefaults(page15);
      saveColdStartAttempts(0U);
    }
    savePage(boostvvtPage2);
    saveColdStartMarker(CONFIG_MARKER_V2);
  }
}

static void writeOutput(bool isOn, const config15 &page15)
{
  if (isOn != (page15.coldStartInv != 0U)) { coldStartState.pin.setPinHigh(); }
  else { coldStartState.pin.setPinLow(); }
  coldStartState.isOutputOn = isOn;
}

static inline bool isLocked(const config15 &page15)
{
  return getDosePercent(page15, coldStartState.attempts) == 0U;
}

void __attribute__((optimize("Os"))) initialiseColdStart(statuses &current, const config15 &page15, const pinNumbers_t &pins)
{
  coldStartState = state_t();
  // Clamp a corrupt count to the locked state: the safe direction
  coldStartState.attempts = (std::min)(loadColdStartAttempts(), DOSE_ATTEMPTS);

  current.coldStartActive = false;
  current.coldStartLocked = isLocked(page15);

  if ((page15.coldStartEnable != 0U) && (pins.pinColdStart != NOT_A_PIN) && !pinIsReserved(pins.pinColdStart))
  {
    coldStartState.pin.setPin(pins.pinColdStart, OUTPUT);
    writeOutput(false, page15);
  }
}

static bool isKeyOn(const statuses &current, const config15 &page15)
{
  // Never below the USB power level, whatever the tune says: USB power alone must not inject
  uint8_t keyOnBattery = (std::max)(page15.coldStartMinBattery, (uint8_t)(KEY_OFF_BATTERY+1U));
  bool keyOn = coldStartState.isKeyOn;
  if (current.battery10 >= keyOnBattery) { keyOn = true; }
  else if (current.battery10 < KEY_OFF_BATTERY) { keyOn = false; }
  else { /* Hysteresis band: keep the last state */ }
  return keyOn;
}

static bool isBatteryLow(const statuses &current, const config15 &page15)
{
  bool isLow = coldStartState.isBatteryLow;
  // Key off (E.g. USB power only) isn't a low battery: the output is already off
  if ((page15.coldStartCutBattery == 0U) || !coldStartState.isKeyOn) { isLow = false; }
  else if (current.battery10 < page15.coldStartCutBattery) { isLow = true; }
  // Hysteresis stops the relay chattering when the voltage hovers at the threshold
  else if ((uint16_t)current.battery10 >= ((uint16_t)page15.coldStartCutBattery + LOW_BATTERY_HYSTERESIS)) { isLow = false; }
  else { /* Hysteresis band: keep the last state */ }
  return isLow;
}

static inline bool isCoolantValid(const statuses &current, const config15 &page15)
{
  return current.coolant >= temperatureRemoveOffset(page15.coldStartMinValidClt);
}

static inline uint8_t lookupCurve(const table2D_u8_u8_6 &table, const statuses &current)
{
  return table2D_getValue(&table, temperatureAddOffset(current.coolant));
}

/** @brief Pulse on time (mS) at the current coolant temperature. 0 if the curve is zero. */
static uint32_t getPulseOnTimeMs(const statuses &current, const config15 &page15)
{
  uint8_t onTime = lookupCurve(coldStartCrankTable, current);
  if (onTime != 0U) { onTime = (std::max)(onTime, page15.coldStartPulseMin); }
  return (uint32_t)onTime * 10U;
}

static void saveAttempts(uint8_t attempts)
{
  if (attempts != coldStartState.attempts)
  {
    coldStartState.attempts = attempts;
    saveColdStartAttempts(attempts);
  }
}

static void openAttempt(uint8_t dosePct)
{
  coldStartState.isAttemptOpen = true;
  coldStartState.attemptDosePct = dosePct;
  coldStartState.crankOnTimeMs = 0U;
  // Saved before any fuel is injected: switching the key off cuts power without warning
  saveAttempts((std::min)((uint8_t)(coldStartState.attempts+1U), DOSE_ATTEMPTS));
}

static inline void closeAttempt(void)
{
  coldStartState.isAttemptOpen = false;
}

static void clearAttempts(void)
{
  closeAttempt();
  saveAttempts(0U);
}

static Phase startPrime(const statuses &current, const config15 &page15, uint32_t nowMs)
{
  Phase phase = Phase::Idle;
  uint8_t dosePct = getDosePercent(page15, coldStartState.attempts);
  // 0.1S * % == mS
  uint32_t durationMs = (uint32_t)lookupCurve(coldStartPrimeTable, current) * dosePct;
  if (isCoolantValid(current, page15) && (durationMs != 0U) && !coldStartState.isBatteryLow)
  {
    openAttempt(dosePct);
    coldStartState.phaseStartMs = nowMs;
    coldStartState.phaseDurationMs = durationMs;
    phase = Phase::Prime;
  }
  return phase;
}

static Phase startCranking(const statuses &current, const config15 &page15, uint32_t nowMs)
{
  if (!coldStartState.isAttemptOpen)
  {
    uint8_t dosePct = getDosePercent(page15, coldStartState.attempts);
    // Battery too low: don't count an attempt that can't inject. Retried every loop, so it opens once the voltage recovers.
    if (!isCoolantValid(current, page15) || (dosePct == 0U) || (getPulseOnTimeMs(current, page15) == 0U) || coldStartState.isBatteryLow)
    {
      return Phase::Idle;
    }
    openAttempt(dosePct);
  }
  coldStartState.pulseStartMs = nowMs;
  return Phase::Cranking;
}

static Phase startAfterStart(const statuses &current, uint32_t nowMs)
{
  Phase phase = Phase::Idle;
  // 0.1S * % == mS
  uint32_t durationMs = (uint32_t)lookupCurve(coldStartAfterTable, current) * coldStartState.attemptDosePct;
  if (durationMs != 0U)
  {
    coldStartState.phaseStartMs = nowMs;
    coldStartState.phaseDurationMs = durationMs;
    coldStartState.pulseStartMs = nowMs;
    phase = Phase::AfterStart;
  }
  return phase;
}

static inline bool isPhaseExpired(uint32_t nowMs)
{
  return hasIntervalElapsed(nowMs, coldStartState.phaseStartMs, coldStartState.phaseDurationMs);
}

static Phase nextPhase(const statuses &current, const config15 &page15, uint32_t nowMs, bool isKeyOnEdge)
{
  bool hasStopped = (current.rotationStatus == EngineRotationStatus::Stopped)
                 && (coldStartState.lastRotation != EngineRotationStatus::Stopped);
  if (!coldStartState.isKeyOn || hasStopped)
  {
    closeAttempt();
    return Phase::Idle;
  }

  Phase phase = coldStartState.phase;
  switch (phase)
  {
    case Phase::Idle:
      if (current.rotationStatus == EngineRotationStatus::Cranking) { phase = startCranking(current, page15, nowMs); }
      else if (isKeyOnEdge && (current.rotationStatus == EngineRotationStatus::Stopped)) { phase = startPrime(current, page15, nowMs); }
      else { /* Stay idle */ }
      break;

    case Phase::Prime:
      if (current.rotationStatus == EngineRotationStatus::Cranking) { phase = startCranking(current, page15, nowMs); }
      else if ((current.rotationStatus == EngineRotationStatus::Running) || isPhaseExpired(nowMs)) { phase = Phase::Idle; }
      else { /* Keep priming */ }
      break;

    case Phase::Cranking:
      if (current.rotationStatus == EngineRotationStatus::Running) { phase = startAfterStart(current, nowMs); }
      break;

    case Phase::AfterStart:
      // Dropping back below the cranking RPM continues the same attempt
      if (current.rotationStatus == EngineRotationStatus::Cranking) { phase = Phase::Cranking; }
      else if (isPhaseExpired(nowMs)) { phase = Phase::Idle; }
      else { /* Keep pulsing */ }
      break;

    default:
      phase = Phase::Idle;
      break;
  }
  return phase;
}

static void updateCounterResets(const statuses &current, const config15 &page15, uint32_t nowMs)
{
  bool isKeyOnStopped = coldStartState.isKeyOn && (current.rotationStatus == EngineRotationStatus::Stopped);

  // The engine started: running long enough clears the counter
  bool isRunningDone = coldStartState.runningTimer.update(current.rotationStatus == EngineRotationStatus::Running,
                                                          nowMs, (uint32_t)page15.coldStartRunReset * 1000U);
  // Waiting with the key on (E.g. for a flooded engine to clear) clears the counter
  bool isKeyOnDone = coldStartState.keyOnTimer.update((page15.coldStartKeyOnReset != 0U) && isKeyOnStopped,
                                                      nowMs, (uint32_t)page15.coldStartKeyOnReset * 60000UL);
  // The reset gesture: throttle held open
  bool isGestureDone = coldStartState.resetGestureTimer.update((page15.coldStartResetTps != 0U) && isKeyOnStopped && (current.TPS >= page15.coldStartResetTps),
                                                               nowMs, (uint32_t)page15.coldStartResetTime * 100U);

  if (isKeyOnDone || isGestureDone) { clearAttempts(); }
  // The running engine may still be in an attempt (After start), so leave that open
  else if (isRunningDone) { saveAttempts(0U); }
  else { /* Nothing to clear */ }
}

/** @brief Advance the pulse window and check whether the output is in the on part of it */
static bool isPulseOn(const statuses &current, const config15 &page15, uint32_t nowMs)
{
  uint32_t onTimeMs = getPulseOnTimeMs(current, page15);
  uint32_t periodMs = (uint32_t)page15.coldStartPulsePeriod * 10U;
  if (periodMs == 0U) { return onTimeMs != 0U; } // No period: continuous

  uint32_t elapsed = timeElapsed(nowMs, coldStartState.pulseStartMs);
  if (elapsed >= periodMs)
  {
    elapsed = elapsed % periodMs;
    coldStartState.pulseStartMs = nowMs - elapsed;
  }
  return elapsed < onTimeMs;
}

static bool isOutputRequired(const statuses &current, const config4 &page4, const config15 &page15, uint32_t nowMs)
{
  // Throttle held open (flood clear) or battery too low: no cold start fuel in any phase
  if ((current.TPS >= page4.floodClear) || coldStartState.isBatteryLow) { return false; }

  bool isOn = false;
  switch (coldStartState.phase)
  {
    case Phase::Prime:
      isOn = true;
      break;

    case Phase::Cranking:
    {
      // 0.1S * % == mS
      uint32_t budgetMs = (uint32_t)page15.coldStartCrankMaxTime * coldStartState.attemptDosePct;
      isOn = (coldStartState.crankOnTimeMs < budgetMs) && isPulseOn(current, page15, nowMs);
      break;
    }

    case Phase::AfterStart:
      isOn = isPulseOn(current, page15, nowMs);
      break;

    default:
      break;
  }
  return isOn;
}

static void accumulateCrankOnTime(uint32_t nowMs)
{
  if (coldStartState.isOutputOn && (coldStartState.phase == Phase::Cranking))
  {
    uint32_t total = coldStartState.crankOnTimeMs + timeElapsed(nowMs, coldStartState.lastUpdateMs);
    coldStartState.crankOnTimeMs = (uint16_t)(std::min)(total, (uint32_t)UINT16_MAX);
  }
}

TESTABLE_INLINE_STATIC void coldStartControlCore(statuses &current, const config4 &page4, const config15 &page15, uint32_t nowMs)
{
  if (coldStartState.pin.isValid())
  {
    bool wasKeyOn = coldStartState.isKeyOn;
    coldStartState.isKeyOn = isKeyOn(current, page15);
    coldStartState.isBatteryLow = isBatteryLow(current, page15);
    accumulateCrankOnTime(nowMs);
    updateCounterResets(current, page15, nowMs);
    coldStartState.phase = nextPhase(current, page15, nowMs, !wasKeyOn && coldStartState.isKeyOn);

    bool isOn = isOutputRequired(current, page4, page15, nowMs);
    if (isOn != coldStartState.isOutputOn) { writeOutput(isOn, page15); }

    current.coldStartActive = isOn;
    current.coldStartLocked = isLocked(page15);
    coldStartState.lastRotation = current.rotationStatus;
    coldStartState.lastUpdateMs = nowMs;
  }
}

uint8_t getColdStartLogStatus(void)
{
  static_assert((uint8_t)Phase::AfterStart <= 0x03U, "Phase does not fit in 2 bits");
  static_assert(DOSE_ATTEMPTS <= 0x07U, "Attempts do not fit in 3 bits");
  return (uint8_t)((uint8_t)coldStartState.phase
                 | (uint8_t)(coldStartState.attempts << 2U)
                 | (uint8_t)((coldStartState.isBatteryLow ? 1U : 0U) << 5U));
}

// LCOV_EXCL_START
void coldStartControl(statuses &current, const config4 &page4, const config15 &page15)
{
  coldStartControlCore(current, page4, page15, millis());
}
// LCOV_EXCL_STOP
