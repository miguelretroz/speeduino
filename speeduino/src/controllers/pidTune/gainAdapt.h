#pragma once

/**
 * @file
 * @brief Continuous PID gain adaptation: one multiplier for P, I and D together. Fork specific.
 *
 * The error of a loop is watched over windows of steady operation:
 * - Oscillating (the error keeps crossing from one side of the on target band to the other): lower the gains
 * - Sluggish (the error stays off target, on one side, for the whole window): raise the gains
 * - Otherwise: keep them
 *
 * Scaling P, I and D together keeps the shape of the controller (the integral and derivative times) and only
 * changes the loop gain. The multiplier stays within 50% to 150%.
 */

#include <stdint.h>
#include "src/PID/PidTuningParameters.h"

namespace pidTune {

/** @brief What the last window decided (for display) */
enum class AdaptDecision : uint8_t
{
  None,  ///< No window finished yet (or it was too short)
  Lower, ///< Oscillating: gains lowered
  Raise, ///< Sluggish: gains raised
  Hold,  ///< Fine: gains kept
};

/** @brief Adaptation settings */
struct gainAdaptConfig_t
{
  uint16_t band;     ///< Error band treated as on target (process value units)
  uint32_t windowMs; ///< Length of each window
  uint8_t stepPct;   ///< Multiplier change per window (%)
};

class GainAdapter
{
public:
  static constexpr uint8_t MIN_MULTIPLIER = 50U;
  static constexpr uint8_t MAX_MULTIPLIER = 150U;
  static constexpr uint8_t DEFAULT_MULTIPLIER = 100U;
  /** @brief Side changes of the error in one window that mean it's oscillating */
  static constexpr uint8_t OSCILLATION_CROSSINGS = 4U;
  /** @brief A window with fewer samples is too short to judge */
  static constexpr uint8_t MIN_WINDOW_SAMPLES = 8U;

  /** @brief Back to 100%, forgetting the window in progress */
  void reset(void);

  /** @brief Set the multiplier (E.g. loaded from EEPROM). Out of range values give 100%. */
  void setMultiplier(uint8_t multiplier);

  /**
   * @brief Feed one error sample of the loop
   *
   * @param error Setpoint - process value
   * @param isSteady The loop is running in steady conditions. False restarts the window.
   * @return true if the multiplier changed
   */
  bool sample(int16_t error, bool isSteady, uint32_t nowMs, const gainAdaptConfig_t &config);

  uint8_t getMultiplier(void) const { return _multiplier; }
  AdaptDecision getLastDecision(void) const { return _lastDecision; }
  /** @brief A steady window is being measured */
  bool isWindowRunning(void) const { return _isWindowRunning; }

  /** @brief The gains scaled by the multiplier. A gain that isn't 0 never becomes 0. */
  PidTuningParameters apply(const PidTuningParameters &gains) const;

private:
  void startWindow(uint32_t nowMs);
  AdaptDecision decide(const gainAdaptConfig_t &config) const;

  uint8_t _multiplier = DEFAULT_MULTIPLIER;
  AdaptDecision _lastDecision = AdaptDecision::None;
  bool _isWindowRunning = false;
  int8_t _lastSide = 0;        ///< Side of the band the error was last seen on: -1 below, 1 above, 0 not yet
  uint8_t _crossings = 0U;
  uint16_t _samples = 0U;
  uint16_t _minError = 0U;     ///< Smallest |error| in the window
  uint16_t _maxError = 0U;     ///< Largest |error| in the window
  uint32_t _windowStartMs = 0U;
};

} // namespace pidTune
