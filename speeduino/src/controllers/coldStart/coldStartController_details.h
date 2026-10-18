#pragma once

#include <stdint.h>
#include "config_pages.h"
#include "statuses.h"
#include "../../pins/outputPin.h"
#include "../../../elapsed_time.h"

namespace coldStartController {

namespace details {

  /** @brief The number of attempts that have a configurable dose. Later attempts get no dose */
  constexpr uint8_t DOSE_ATTEMPTS = sizeof(config15::coldStartDosePct);

  /** @brief Which part of the start sequence the cold start system is in */
  enum class Phase : uint8_t
  {
    Idle,       ///< Not injecting
    Prime,      ///< Key on, before cranking
    Cranking,   ///< Engine cranking
    AfterStart, ///< Engine just started
  };

  /** @brief Tracks how long a condition has been held */
  struct holdTimer_t
  {
    bool isActive = false;
    uint32_t sinceMs = 0U;

    /** @brief Returns true once the condition has been held for at least durationMs */
    bool update(bool condition, uint32_t nowMs, uint32_t durationMs)
    {
      if (!condition)
      {
        isActive = false;
        return false;
      }
      if (!isActive)
      {
        isActive = true;
        sinceMs = nowMs;
      }
      return hasIntervalElapsed(nowMs, sinceMs, durationMs);
    }
  };

  struct state_t
  {
    outputPin_t pin;
    Phase phase = Phase::Idle;
    EngineRotationStatus lastRotation = EngineRotationStatus::Stopped;
    bool isOutputOn = false;
    bool isKeyOn = false;
    bool isBatteryLow = false;       ///< Output cut by the low battery threshold
    bool isAttemptOpen = false;      ///< An attempt has been counted and has not ended yet
    uint8_t attempts = 0U;           ///< Attempts made without the engine starting (mirrors EEPROM)
    uint8_t attemptDosePct = 0U;     ///< Dose of the open attempt
    uint16_t crankOnTimeMs = 0U;     ///< Total output on time while cranking, for the open attempt
    uint32_t phaseStartMs = 0U;
    uint32_t phaseDurationMs = 0U;
    uint32_t pulseStartMs = 0U;
    uint32_t lastUpdateMs = 0U;
    holdTimer_t runningTimer;        ///< Engine running: clears the counter
    holdTimer_t keyOnTimer;          ///< Key on with the engine stopped: clears the counter
    holdTimer_t resetGestureTimer;   ///< Throttle held open with the engine stopped: clears the counter
  };

  /** @brief The dose (%) for the next attempt, given the number of attempts already made */
  uint8_t getDosePercent(const config15 &page15, uint8_t attempts);

  /** @brief Set the cold start fields of page 15 to their defaults. Leaves the system disabled. */
  void setDefaults(config15 &page15);

  /** @brief Defaults for the fields added in config version 2 (low battery cut and key on reset) */
  void setV2Defaults(config15 &page15);

} // namespace details

} // namespace coldStartController
