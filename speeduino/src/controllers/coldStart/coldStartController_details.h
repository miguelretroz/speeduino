#pragma once

#include <stdint.h>
#include "config_pages.h"
#include "statuses.h"
#include "../../pins/outputPin.h"

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

  struct state_t
  {
    outputPin_t pin;
    Phase phase = Phase::Idle;
    EngineRotationStatus lastRotation = EngineRotationStatus::Stopped;
    bool isOutputOn = false;
    bool isKeyOn = false;
    bool isAttemptOpen = false;      ///< An attempt has been counted and has not ended yet
    bool isResetGestureActive = false;
    uint8_t attempts = 0U;           ///< Attempts made without the engine starting (mirrors EEPROM)
    uint8_t attemptDosePct = 0U;     ///< Dose of the open attempt
    uint16_t crankOnTimeMs = 0U;     ///< Total output on time while cranking, for the open attempt
    uint32_t phaseStartMs = 0U;
    uint32_t phaseDurationMs = 0U;
    uint32_t pulseStartMs = 0U;
    uint32_t lastUpdateMs = 0U;
    uint32_t runningSinceMs = 0U;
    uint32_t resetGestureSinceMs = 0U;
  };

  /** @brief The dose (%) for the next attempt, given the number of attempts already made */
  uint8_t getDosePercent(const config15 &page15, uint8_t attempts);

  /** @brief Set the cold start fields of page 15 to their defaults. Leaves the system disabled. */
  void setDefaults(config15 &page15);

} // namespace details

} // namespace coldStartController
