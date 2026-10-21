#pragma once

#include <stdint.h>
#include "config_pages.h"
#include "relayTuner.h"
#include "pidTuneRules.h"
#include "gainAdapt.h"

namespace pidTuneController {

namespace details {

  /** @brief The idle PID runs every 250mS: the relay runs at the same rate */
  constexpr uint32_t IDLE_SAMPLE_MS = 250UL;
  /** @brief Idle: fail if the relay doesn't switch within this time */
  constexpr uint16_t IDLE_NO_SWITCH_MS = 15000U;
  /** @brief Idle: fail if the RPM gets this far from the setpoint */
  constexpr uint16_t IDLE_MAX_EXCURSION = 300U;
  /** @brief Idle: abort below this RPM, the engine is about to stall */
  constexpr uint16_t IDLE_MIN_RPM = 400U;
  /** @brief Idle: throttle closed. The idle integral reset TPS is used too, when lower (it can be set as high as 100%). (Unit: 0.5%) */
  constexpr uint8_t IDLE_MAX_TPS = 6U;
  /** @brief AFR: fail if the relay doesn't switch within this time */
  constexpr uint16_t EGO_NO_SWITCH_MS = 10000U;
  /** @brief AFR: fail if the AFR gets this far from the setpoint (Unit: 0.1 AFR) */
  constexpr uint16_t EGO_MAX_EXCURSION = 20U;
  /** @brief AFR: fail if the closed loop doesn't step for this long */
  constexpr uint32_t EGO_STEP_TIMEOUT_MS = 8000UL;
  /** @brief AFR: abort if the RPM moves more than this from where the test started (%) */
  constexpr uint8_t EGO_RPM_WINDOW_PCT = 15U;
  /** @brief AFR: abort if the MAP moves more than this from where the test started (kPa) */
  constexpr uint16_t EGO_MAP_WINDOW = 10U;
  /** @brief AFR: abort if the TPS moves more than this from where the test started (Unit: 0.5%) */
  constexpr uint8_t EGO_TPS_WINDOW = 4U;

  /** @brief Gain adaptation: a multiplier is saved once it has been stable for this long (or when the engine stops) */
  constexpr uint32_t ADAPT_SAVE_DELAY_MS = 60000UL;
  /** @brief AFR gain adaptation: a window restarts if the RPM moves more than this from where it started (%) */
  constexpr uint8_t EGO_ADAPT_RPM_WINDOW_PCT = 15U;

  /** @brief The loop being tuned */
  enum class TuneLoop : uint8_t
  {
    Idle,
    Ego,
  };

  /** @brief Gain adaptation of one loop */
  struct adaptLoop_t
  {
    pidTune::GainAdapter adapter;
    uint8_t gains[3] = {};             ///< P, I and D the multiplier applies to: a change resets it
    uint8_t appliedMultiplier = 100U;  ///< Multiplier in use (100 when the adaptation is off)
    uint8_t savedMultiplier = 100U;    ///< Multiplier in EEPROM
    uint32_t changedMs = 0U;           ///< When the multiplier last changed
  };

  struct state_t
  {
    pidTune::RelayTuner tuner;
    pidTune::tuneBytes_t result = {};  ///< Suggested bytes, valid when the tuner is Done
    uint16_t ultimateGainQ8 = 0U;      ///< For display
    TuneLoop loop = TuneLoop::Idle;
    bool isActive = false;             ///< A test is running, or stopped and not handed back to the PID yet
    bool isApplied = false;            ///< The result was written to the tune
    bool isUiRefresh = false;          ///< Ask TunerStudio to reload page 6
    bool isUiRefreshSent = false;      ///< The reload request was in the live data sent last
    bool isVelocityForm = false;       ///< Idle: stepper closed loop when the test started
    bool isIdleBumpless = false;       ///< The next idle gain change must not make the output jump
    bool isEgoBumpless = false;        ///< The next AFR gain change must not make the output jump
    bool wasAirConOn = false;          ///< Load when the test started: a change aborts the test
    bool wasIdleUpOn = false;
    bool wasFanOn = false;
    uint8_t iacAlgorithm = 0U;         ///< Idle algorithm when the test started
    uint8_t startTps = 0U;
    uint16_t startRpm = 0U;
    uint16_t startMap = 0U;
    uint32_t lastSampleMs = 0U;        ///< Last relay sample

    adaptLoop_t idleAdapt;
    adaptLoop_t egoAdapt;
    uint8_t idleAdaptLoads = 0U;       ///< Idle loads (A/C, idle up, fan) at the last adaptation sample: a change isn't steady
    uint16_t egoAdaptRpm = 0U;         ///< RPM the AFR adaptation window started at
    bool wasRunning = false;           ///< The engine was running at the last check
  };

  /** @brief Set the PID auto tune and gain adaptation fields of page 15 to their defaults. Adaptation off. */
  void setDefaults(config15 &page15);

} // namespace details

} // namespace pidTuneController
