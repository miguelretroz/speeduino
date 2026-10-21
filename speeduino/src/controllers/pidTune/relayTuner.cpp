/** @file
 * Relay feedback experiment for PID auto tuning. See relayTuner.h. Fork specific.
 */
#include "relayTuner.h"
#include "../../../elapsed_time.h"

// Runs a few times per second at most, so optimize for size.
#pragma GCC optimize("Os")

namespace pidTune {

static uint16_t saturateU16(uint32_t value)
{
  return (value > UINT16_MAX) ? (uint16_t)UINT16_MAX : (uint16_t)value;
}

void RelayTuner::start(const relayConfig_t &config, uint32_t nowMs)
{
  _config = config;
  if (_config.cycles < RELAY_MIN_CYCLES) { _config.cycles = RELAY_MIN_CYCLES; }
  if (_config.cycles > RELAY_MAX_CYCLES) { _config.cycles = RELAY_MAX_CYCLES; }

  _phase = TunePhase::Settle;
  _failure = TuneFail::None;
  _isHigh = false;
  _isCycleStarted = false;
  _isFirstCycleDone = false;
  _storedCycles = 0U;
  _ringIndex = 0U;
  _setpoint = 0;
  _settleSum = 0;
  _sampleCount = 0U;
  _startMs = nowMs;
  _phaseStartMs = nowMs;
  _lastSwitchMs = nowMs;
  _cycleStartMs = nowMs;
  _lastSampleMs = nowMs;
  _periodMs = 0U;
  _amplitude = 0U;
  _relayAmplitude = 0U;
}

int32_t RelayTuner::sample(int16_t processValue, int32_t realizedOutput, uint32_t nowMs)
{
  update(nowMs);

  if (_phase == TunePhase::Settle)
  {
    // Can't overflow: UINT16_MAX * INT16_MAX < INT32_MAX
    _settleSum += processValue;
    if (_sampleCount < UINT16_MAX) { ++_sampleCount; }
    if (hasIntervalElapsed(nowMs, _phaseStartMs, RELAY_SETTLE_MS) && (_sampleCount >= RELAY_MIN_SETTLE_SAMPLES))
    {
      startRelay(processValue, realizedOutput, nowMs);
    }
  }
  else if (_phase == TunePhase::Relay)
  {
    if (_sampleCount < UINT16_MAX) { ++_sampleCount; }
    _lastSampleMs = nowMs;
    trackExtremes(processValue, realizedOutput);

    const int32_t offset = (int32_t)processValue - (int32_t)_setpoint;
    if ((offset > (int32_t)_config.maxExcursion) || (-offset > (int32_t)_config.maxExcursion))
    {
      fail(TuneFail::AmplitudeTooLarge);
    }
    else
    {
      // Positive when the output needs to go up to bring the process value back to the setpoint
      const int32_t error = (_config.direction < 0) ? offset : -offset;
      if (!_isHigh && (error > (int32_t)_config.hysteresis))
      {
        _isHigh = true;
        _lastSwitchMs = nowMs;
        // A full cycle runs from one low to high switch to the next
        endCycle(nowMs);
        resetExtremes(processValue, realizedOutput);
      }
      else if (_isHigh && (error < -(int32_t)_config.hysteresis))
      {
        _isHigh = false;
        _lastSwitchMs = nowMs;
      }
      else
      {
        // Inside the band: keep the relay where it is
      }
    }
  }
  else
  {
    // Not running
  }

  return getOutput();
}

void RelayTuner::update(uint32_t nowMs)
{
  if (isRunning())
  {
    if (hasIntervalElapsed(nowMs, _startMs, _config.timeoutMs))
    {
      fail(TuneFail::NoConvergence);
    }
    else if ((_phase == TunePhase::Relay) && hasIntervalElapsed(nowMs, _lastSwitchMs, _config.noSwitchMs))
    {
      fail(TuneFail::NoSwitching);
    }
    else
    {
      // Still running
    }
  }
}

void RelayTuner::abort(TuneFail reason)
{
  if (isRunning()) { fail(reason); }
}

void RelayTuner::reset(void)
{
  _phase = TunePhase::Idle;
  _failure = TuneFail::None;
}

int32_t RelayTuner::getOutput(void) const
{
  if (_phase == TunePhase::Relay)
  {
    return _isHigh ? (_config.base + _config.amplitude) : (_config.base - _config.amplitude);
  }
  return _config.base;
}

uint16_t RelayTuner::getSamplePeriodMs(void) const
{
  if (_sampleCount == 0U) { return 0U; }
  return saturateU16(timeElapsed(_lastSampleMs, _phaseStartMs) / _sampleCount);
}

void RelayTuner::fail(TuneFail reason)
{
  _phase = TunePhase::Failed;
  _failure = reason;
}

void RelayTuner::startRelay(int16_t processValue, int32_t realizedOutput, uint32_t nowMs)
{
  _setpoint = (int16_t)(_settleSum / (int32_t)_sampleCount);
  _phase = TunePhase::Relay;
  _phaseStartMs = nowMs;
  _lastSwitchMs = nowMs;
  _lastSampleMs = nowMs;
  _sampleCount = 0U;
  _isHigh = true; // Kick the output up first
  resetExtremes(processValue, realizedOutput);
}

void RelayTuner::resetExtremes(int16_t processValue, int32_t realizedOutput)
{
  _pvMax = processValue;
  _pvMin = processValue;
  _outputMax = realizedOutput;
  _outputMin = realizedOutput;
}

void RelayTuner::trackExtremes(int16_t processValue, int32_t realizedOutput)
{
  if (processValue > _pvMax) { _pvMax = processValue; }
  if (processValue < _pvMin) { _pvMin = processValue; }
  if (realizedOutput > _outputMax) { _outputMax = realizedOutput; }
  if (realizedOutput < _outputMin) { _outputMin = realizedOutput; }
}

void RelayTuner::endCycle(uint32_t nowMs)
{
  if (_isCycleStarted)
  {
    if (_isFirstCycleDone)
    {
      _periods[_ringIndex] = saturateU16(timeElapsed(nowMs, _cycleStartMs));
      _amplitudes[_ringIndex] = saturateU16((uint32_t)(((int32_t)_pvMax - (int32_t)_pvMin) + 1) / 2U);
      _relayAmplitudes[_ringIndex] = saturateU16((uint32_t)(_outputMax - _outputMin) / 2U);
      ++_ringIndex;
      if (_ringIndex >= _config.cycles) { _ringIndex = 0U; }
      if (_storedCycles < _config.cycles) { ++_storedCycles; }
      checkConvergence();
    }
    else
    {
      // The first cycle still has the start up transient
      _isFirstCycleDone = true;
    }
  }
  _isCycleStarted = true;
  _cycleStartMs = nowMs;
}

void RelayTuner::checkConvergence(void)
{
  if (_storedCycles >= _config.cycles)
  {
    uint16_t minPeriod = UINT16_MAX;
    uint16_t maxPeriod = 0U;
    uint16_t minAmplitude = UINT16_MAX;
    uint16_t maxAmplitude = 0U;
    uint32_t periodSum = 0U;
    uint32_t amplitudeSum = 0U;
    uint32_t relayAmplitudeSum = 0U;
    for (uint8_t i = 0U; i < _config.cycles; ++i)
    {
      if (_periods[i] < minPeriod) { minPeriod = _periods[i]; }
      if (_periods[i] > maxPeriod) { maxPeriod = _periods[i]; }
      if (_amplitudes[i] < minAmplitude) { minAmplitude = _amplitudes[i]; }
      if (_amplitudes[i] > maxAmplitude) { maxAmplitude = _amplitudes[i]; }
      periodSum += _periods[i];
      amplitudeSum += _amplitudes[i];
      relayAmplitudeSum += _relayAmplitudes[i];
    }

    // Converged once the periods agree within 20% and the amplitudes within 25%
    if ((((uint32_t)maxPeriod - minPeriod) * 5U <= maxPeriod) && (((uint32_t)maxAmplitude - minAmplitude) * 4U <= maxAmplitude))
    {
      const uint8_t halfCycles = _config.cycles / 2U;
      _periodMs = (uint16_t)((periodSum + halfCycles) / _config.cycles);
      _amplitude = (uint16_t)((amplitudeSum + halfCycles) / _config.cycles);
      _relayAmplitude = (uint16_t)((relayAmplitudeSum + halfCycles) / _config.cycles);

      // An oscillation not much bigger than the hysteresis band says more about the band than the loop
      if ((_relayAmplitude == 0U) || (((uint32_t)_amplitude * 2U) < ((uint32_t)_config.hysteresis * 3U)) || (_amplitude == 0U))
      {
        fail(TuneFail::AmplitudeTooSmall);
      }
      else
      {
        _phase = TunePhase::Done;
      }
    }
  }
}

} // namespace pidTune
