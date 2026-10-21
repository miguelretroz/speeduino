#pragma once

/**
 * @file
 * @brief Relay feedback experiment for PID auto tuning (Astrom-Hagglund, relay with hysteresis). Fork specific.
 *
 * While it runs, the tuner replaces the PID: the actuator is switched between base + amplitude and
 * base - amplitude whenever the process value leaves a band around the setpoint. The loop then oscillates
 * at its ultimate period, with an amplitude that gives the ultimate gain: Ku = 4 * relay amplitude / (pi * amplitude).
 *
 * - Settle: the actuator is held at the base value and the process value is averaged. The average becomes the
 *   relay setpoint, so the oscillation is centred on the point the loop was actually running at.
 * - Relay: switches the actuator. Each full cycle (low to high switch to the next) gives a period and an
 *   amplitude. The first cycle is discarded (start up transient). Done once the last cycles agree.
 *
 * The tuner is plant agnostic and has no timing of its own: the caller samples it at the rate of the loop being
 * tuned. Timeouts are based on the time passed in.
 */

#include <stdint.h>

namespace pidTune {

/** @brief The phases of the experiment */
enum class TunePhase : uint8_t
{
  Idle,   ///< Not running, no result
  Settle, ///< Holding the actuator, measuring the setpoint
  Relay,  ///< Switching the actuator
  Done,   ///< Finished, the result is valid
  Failed, ///< Stopped, see TuneFail
};

/** @brief Why the experiment stopped without a result */
enum class TuneFail : uint8_t
{
  None,
  NotReady,                 ///< The loop wasn't running in closed loop (or the engine wasn't running)
  WrongMode,                ///< The loop isn't in a mode the tune supports
  NoAuthority,              ///< The relay would take the output outside its limits
  AmplitudeBelowHysteresis, ///< The relay amplitude is too small for the stepper hysteresis
  ConditionLost,            ///< The operating condition changed (throttle, DFCO, A/C, closed loop off...)
  NoSwitching,              ///< The process value never left the band: the relay amplitude is too small
  AmplitudeTooSmall,        ///< The oscillation is too small compared to the hysteresis
  AmplitudeTooLarge,        ///< The process value went too far from the setpoint
  NoConvergence,            ///< The oscillation didn't settle before the timeout
  StepPeriodTooLong,        ///< The loop runs too slowly to be tuned
  Aborted,                  ///< Stopped by the user
};

/** @brief The most cycles the tuner can average */
constexpr uint8_t RELAY_MAX_CYCLES = 4U;
/** @brief The fewest cycles the tuner can average */
constexpr uint8_t RELAY_MIN_CYCLES = 2U;
/** @brief How long the actuator is held before the relay starts */
constexpr uint32_t RELAY_SETTLE_MS = 3000UL;
/** @brief The fewest samples the setpoint is averaged over */
constexpr uint8_t RELAY_MIN_SETTLE_SAMPLES = 4U;

/** @brief Experiment settings */
struct relayConfig_t
{
  int32_t base;           ///< Actuator value the relay switches around (output units)
  int32_t amplitude;      ///< Relay amplitude (output units)
  uint16_t hysteresis;    ///< Half width of the band around the setpoint (process value units)
  uint16_t maxExcursion;  ///< Fail if the process value gets further than this from the setpoint
  uint32_t timeoutMs;     ///< Fail if not done within this time
  uint16_t noSwitchMs;    ///< Fail if the relay doesn't switch within this time
  int8_t direction;       ///< 1: more output raises the process value. -1: more output lowers it
  uint8_t cycles;         ///< Cycles to average, after the discarded first one (RELAY_MIN_CYCLES to RELAY_MAX_CYCLES)
};

class RelayTuner
{
public:
  /** @brief Start the experiment (Settle phase). The actuator must be at config.base. */
  void start(const relayConfig_t &config, uint32_t nowMs);

  /**
   * @brief Feed one sample of the loop. Call it at the rate of the loop being tuned.
   *
   * @param processValue The measured value (E.g. RPM)
   * @param realizedOutput Where the actuator really is (E.g. the stepper position). Used to measure the
   * relay amplitude the process actually sees, which can be smaller than the commanded one.
   * @param nowMs Current time
   * @return The actuator value to apply
   */
  int32_t sample(int16_t processValue, int32_t realizedOutput, uint32_t nowMs);

  /** @brief Check the timeouts. Call it regularly, even when the loop isn't sampling. */
  void update(uint32_t nowMs);

  /** @brief Stop the experiment with a failure. No effect unless it's running. */
  void abort(TuneFail reason);

  /** @brief Go to Failed, running or not (E.g. to show why the experiment couldn't start) */
  void fail(TuneFail reason);

  /** @brief Back to Idle, forgetting any result */
  void reset(void);

  TunePhase getPhase(void) const { return _phase; }
  TuneFail getFailure(void) const { return _failure; }
  bool isRunning(void) const { return (_phase == TunePhase::Settle) || (_phase == TunePhase::Relay); }

  /** @brief The actuator value for the current state of the relay */
  int32_t getOutput(void) const;
  int32_t getBase(void) const { return _config.base; }
  int16_t getSetpoint(void) const { return _setpoint; }

  /** @name Results, valid once Done */
  ///@{
  uint16_t getPeriodMs(void) const { return _periodMs; }            ///< Ultimate period (mS)
  uint16_t getAmplitude(void) const { return _amplitude; }          ///< Oscillation amplitude, half peak to peak (process value units)
  uint16_t getRelayAmplitude(void) const { return _relayAmplitude; } ///< Relay amplitude seen by the process (output units)
  /** @brief Average time between samples during the relay (mS). The sample period of loops with no fixed rate. */
  uint16_t getSamplePeriodMs(void) const;
  ///@}

  /** @brief Cycles measured so far, after the discarded one (for display) */
  uint8_t getCycles(void) const { return _storedCycles; }

private:
  void startRelay(int16_t processValue, int32_t realizedOutput, uint32_t nowMs);
  void resetExtremes(int16_t processValue, int32_t realizedOutput);
  void trackExtremes(int16_t processValue, int32_t realizedOutput);
  void endCycle(uint32_t nowMs);
  void checkConvergence(void);

  relayConfig_t _config = {};
  TunePhase _phase = TunePhase::Idle;
  TuneFail _failure = TuneFail::None;
  bool _isHigh = false;
  bool _isCycleStarted = false;
  bool _isFirstCycleDone = false;
  uint8_t _storedCycles = 0U;
  uint8_t _ringIndex = 0U;

  int16_t _setpoint = 0;
  int32_t _settleSum = 0;
  uint16_t _sampleCount = 0U;  ///< Settle samples, then relay samples

  uint32_t _startMs = 0U;
  uint32_t _phaseStartMs = 0U;
  uint32_t _lastSwitchMs = 0U;
  uint32_t _cycleStartMs = 0U;
  uint32_t _lastSampleMs = 0U;

  int16_t _pvMax = 0;
  int16_t _pvMin = 0;
  int32_t _outputMax = 0;
  int32_t _outputMin = 0;

  uint16_t _periods[RELAY_MAX_CYCLES] = {};
  uint16_t _amplitudes[RELAY_MAX_CYCLES] = {};
  uint16_t _relayAmplitudes[RELAY_MAX_CYCLES] = {};

  uint16_t _periodMs = 0U;
  uint16_t _amplitude = 0U;
  uint16_t _relayAmplitude = 0U;
};

} // namespace pidTune
