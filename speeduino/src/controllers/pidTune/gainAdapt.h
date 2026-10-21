#pragma once

/**
 * @file
 * @brief Continuous PID gain adaptation: one multiplier for P, I and D together. Fork specific.
 *
 * The error of a loop is watched over windows of steady operation:
 * - Oscillating (the error keeps crossing from one side of the on target band to the other): lower the gains.
 *   A side only counts once the error has stayed there for 2 samples in a row, so noise crossing the band on single
 *   samples isn't taken for an oscillation, while hunting with a period of a few samples still is.
 * - Sluggish (the error stays off target on average, without crossing): raise the gains. The average is taken on
 *   a low pass filtered error.
 * - Otherwise: keep them
 *
 * A window restarts when the loop isn't steady, or when the samples stop for a while (E.g. the loop was off).
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
  /** @brief Samples in a row on one side of the band for that side to count */
  static constexpr uint8_t SIDE_SAMPLES = 2U;
  /** @brief A longer gap between samples restarts the window */
  static constexpr uint32_t MAX_SAMPLE_GAP_MS = 2000UL;
  /** @brief Error filter (for the average): each sample moves the filtered error 1/FILTER_DIVIDER of the way */
  static constexpr int8_t FILTER_DIVIDER = 4;

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
  void startWindow(int16_t error, uint32_t nowMs);
  AdaptDecision decide(const gainAdaptConfig_t &config) const;

  uint8_t _multiplier = DEFAULT_MULTIPLIER;
  AdaptDecision _lastDecision = AdaptDecision::None;
  bool _isWindowRunning = false;
  int8_t _side = 0;               ///< Side of the band the error is on: -1 below, 1 above, 0 not known yet
  int8_t _newSide = 0;            ///< Side the error has just moved to, not confirmed yet
  uint8_t _newSideSamples = 0U;   ///< Samples in a row on the new side
  uint8_t _crossings = 0U;
  uint16_t _samples = 0U;
  int32_t _filteredError = 0;     ///< Low pass filtered error (x16)
  uint32_t _absErrorSum = 0U;     ///< Sum of the filtered |error| over the window
  uint32_t _windowStartMs = 0U;
  uint32_t _lastSampleMs = 0U;
};

} // namespace pidTune
