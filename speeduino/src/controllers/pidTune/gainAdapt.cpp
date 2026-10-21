/** @file
 * Continuous PID gain adaptation. See gainAdapt.h. Fork specific.
 */
#include "gainAdapt.h"
#include "../../../elapsed_time.h"

// Runs a few times per second at most, so optimize for size.
#pragma GCC optimize("Os")

namespace pidTune {

static int16_t scaleGain(int16_t gain, uint8_t multiplier)
{
  if (gain == 0) { return 0; }
  const int32_t product = (int32_t)gain * multiplier;
  int32_t scaled = (product + ((product < 0) ? -50L : 50L)) / 100L;
  // Never turn a term off
  if (scaled == 0) { scaled = (gain < 0) ? -1L : 1L; }
  return (int16_t)scaled;
}

void GainAdapter::reset(void)
{
  _multiplier = DEFAULT_MULTIPLIER;
  _lastDecision = AdaptDecision::None;
  _isWindowRunning = false;
}

void GainAdapter::setMultiplier(uint8_t multiplier)
{
  _multiplier = ((multiplier < MIN_MULTIPLIER) || (multiplier > MAX_MULTIPLIER)) ? DEFAULT_MULTIPLIER : multiplier;
}

bool GainAdapter::sample(int16_t error, bool isSteady, uint32_t nowMs, const gainAdaptConfig_t &config)
{
  if (!isSteady)
  {
    _isWindowRunning = false;
    return false;
  }
  if (!_isWindowRunning) { startWindow(error, nowMs); }

  _filteredError += (((int32_t)error * 16L) - _filteredError) / FILTER_DIVIDER;
  const int32_t filtered = _filteredError / 16L;
  _absErrorSum += (uint32_t)((filtered < 0) ? -filtered : filtered);
  if (_samples < UINT16_MAX) { ++_samples; }

  // Crossings are counted from one side of the band to the other, so small swings inside the band don't count
  const uint16_t band = (config.band == 0U) ? 1U : config.band;
  int8_t side = 0;
  if (filtered > (int32_t)band) { side = 1; }
  else if (filtered < -(int32_t)band) { side = -1; }
  else { /* Inside the band */ }
  if (side != 0)
  {
    if ((_lastSide != 0) && (side != _lastSide) && (_crossings < UINT8_MAX)) { ++_crossings; }
    _lastSide = side;
  }

  bool isChanged = false;
  if (hasIntervalElapsed(nowMs, _windowStartMs, config.windowMs))
  {
    const AdaptDecision decision = decide(config);
    if (decision == AdaptDecision::Lower)
    {
      const int16_t lowered = (int16_t)_multiplier - config.stepPct;
      _multiplier = (lowered < (int16_t)MIN_MULTIPLIER) ? MIN_MULTIPLIER : (uint8_t)lowered;
    }
    else if (decision == AdaptDecision::Raise)
    {
      const int16_t raised = (int16_t)_multiplier + config.stepPct;
      _multiplier = (raised > (int16_t)MAX_MULTIPLIER) ? MAX_MULTIPLIER : (uint8_t)raised;
    }
    else
    {
      // Keep the gains
    }
    isChanged = (decision == AdaptDecision::Lower) || (decision == AdaptDecision::Raise);
    if (decision != AdaptDecision::None) { _lastDecision = decision; }
    startWindow(error, nowMs);
  }
  return isChanged;
}

PidTuningParameters GainAdapter::apply(const PidTuningParameters &gains) const
{
  PidTuningParameters scaled = gains;
  scaled.Kp = scaleGain(gains.Kp, _multiplier);
  scaled.Ki = scaleGain(gains.Ki, _multiplier);
  scaled.Kd = scaleGain(gains.Kd, _multiplier);
  return scaled;
}

void GainAdapter::startWindow(int16_t error, uint32_t nowMs)
{
  _isWindowRunning = true;
  _windowStartMs = nowMs;
  _lastSide = 0;
  _crossings = 0U;
  _samples = 0U;
  _filteredError = (int32_t)error * 16L;
  _absErrorSum = 0U;
}

AdaptDecision GainAdapter::decide(const gainAdaptConfig_t &config) const
{
  const uint16_t band = (config.band == 0U) ? 1U : config.band;
  if (_samples < MIN_WINDOW_SAMPLES) { return AdaptDecision::None; }
  if (_crossings >= OSCILLATION_CROSSINGS) { return AdaptDecision::Lower; }
  // Off target on average, on one side
  if ((_crossings == 0U) && (_absErrorSum >= ((uint32_t)band * _samples))) { return AdaptDecision::Raise; }
  return AdaptDecision::Hold;
}

} // namespace pidTune
