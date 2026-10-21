/** @file
 * PID auto tune for the idle and AFR closed loops. See pidTuneController.h. Fork specific.
 * The config is found at the end of page 15 (of type @ref config15).
 */
#include "pidTuneController.h"
#include "pidTuneController_details.h"
#include "../../../globals.h"
#include "../idle/idle.h"
#include "../../../corrections.h"
#include "../../../storage.h"
#include "../../../pages.h"
#include "../../../maths.h"
#include "../../../units.h"
#include "../../../unit_testing.h"
#include "../../../elapsed_time.h"

// Nothing here is time critical (the relay runs a few times per second), so optimize for size.
#pragma GCC optimize("Os")

using namespace pidTuneController::details;
using namespace pidTune;

TESTABLE_STATIC state_t pidTuneState;

namespace pidTuneController {

namespace details {

void setDefaults(config15 &page15)
{
  page15.pidAdaptEgoBand = 2U;        // 0.2 AFR
  page15.pidAdaptIdleBand = 25U;      // 25 RPM
  page15.pidAdaptWindow = 15U;        // 15S
  page15.pidAdaptStep = 5U;           // 5%
  page15.pidTuneTimeout = 90U;        // 90S
  page15.pidTuneCycles = 3U;
  page15.pidTuneEgoHyst = 1U;         // 0.1 AFR
  page15.pidTuneEgoAmplitude = 5U;    // 5%
  page15.pidTuneIdleHyst = 15U;       // 15 RPM
  page15.pidTuneIdleAmplitude = 8U;   // 8 steps or 8% duty
  page15.pidAdaptIdle = 0U;
  page15.pidAdaptEgo = 0U;
  page15.pidTuneRule = (uint8_t)TuneRule::BalancedPI;
  page15.pidTuneUnused = 0U;
}

} // namespace details

} // namespace pidTuneController

static bool isIdleClosedLoop(uint8_t algorithm)
{
  return (algorithm == IAC_ALGORITHM_PWM_CL)
      || (algorithm == IAC_ALGORITHM_PWM_OLCL)
      || (algorithm == IAC_ALGORITHM_STEP_CL)
      || (algorithm == IAC_ALGORITHM_STEP_OLCL);
}

static bool isAirConOn(const statuses &current)
{
  return current.acStatus.turningOn || current.acStatus.compressorOn;
}

static uint16_t absDifference(uint16_t a, uint16_t b)
{
  return (a > b) ? (a - b) : (b - a);
}

/** @brief The throttle is closed, as far as the idle loop is concerned */
static bool isThrottleClosed(const statuses &current, const config2 &page2)
{
  return (current.TPS <= IDLE_MAX_TPS) && (current.TPS <= page2.iacTPSlimit);
}

/** @brief Work out the suggested bytes from the measured ultimate point */
static void computeResult(const config15 &page15)
{
  const uint32_t ultimateGainQ12 = computeUltimateGainQ12(pidTuneState.tuner.getRelayAmplitude(), pidTuneState.tuner.getAmplitude());
  const controllerDesign_t design = designController((TuneRule)page15.pidTuneRule, ultimateGainQ12, pidTuneState.tuner.getPeriodMs());
  if (pidTuneState.loop == TuneLoop::Idle)
  {
    pidTuneState.result = pidTuneState.isVelocityForm ? toIdleVelocityBytes(design) : toIdlePositionalBytes(design);
  }
  else
  {
    pidTuneState.result = toEgoBytes(design, pidTuneState.tuner.getSamplePeriodMs());
  }
  // Q12 to Q8, for display
  const uint32_t ultimateGainQ8 = ultimateGainQ12 >> (GAIN_FRACTION_BITS - 8U);
  pidTuneState.ultimateGainQ8 = (ultimateGainQ8 > UINT16_MAX) ? (uint16_t)UINT16_MAX : (uint16_t)ultimateGainQ8;
  pidTuneState.resultRule = page15.pidTuneRule;
}

/** @brief Once the test has stopped: hand the loop back to its PID and work out the result */
static void finishIfStopped(const config15 &page15)
{
  if (pidTuneState.isActive && !pidTuneState.tuner.isRunning())
  {
    pidTuneState.isActive = false;
    const int32_t base = pidTuneState.tuner.getBase();
    if (pidTuneState.loop == TuneLoop::Idle) { idleTuneHandBack(base); }
    else { egoTuneHandBack((uint8_t)base); }

    if (pidTuneState.tuner.getPhase() == TunePhase::Done) { computeResult(page15); }
  }
}

static void startTune(TuneLoop loop, const relayConfig_t &config, const statuses &current, uint32_t nowMs)
{
  pidTuneState.loop = loop;
  pidTuneState.isActive = true;
  pidTuneState.isApplied = false;
  pidTuneState.result = {};
  pidTuneState.ultimateGainQ8 = 0U;
  pidTuneState.wasAirConOn = isAirConOn(current);
  pidTuneState.wasIdleUpOn = current.idleUpActive;
  pidTuneState.wasFanOn = (current.fanDuty > 0U);
  pidTuneState.startTps = current.TPS;
  pidTuneState.startRpm = current.RPM;
  pidTuneState.startMap = current.MAP;
  // Idle: sample on the first call. AFR: the step timeout starts now.
  pidTuneState.lastSampleMs = nowMs - IDLE_SAMPLE_MS;
  pidTuneState.tuner.start(config, nowMs);
}

/** @brief Show why a test couldn't start */
static void refuseTune(TuneLoop loop, TuneFail reason)
{
  pidTuneState.loop = loop;
  pidTuneState.isApplied = false;
  pidTuneState.result = {};
  pidTuneState.ultimateGainQ8 = 0U;
  pidTuneState.tuner.fail(reason);
}

static TuneFail startIdleTune(const statuses &current, const config2 &page2, const config6 &page6, const config15 &page15, uint32_t nowMs)
{
  if (current.rotationStatus != EngineRotationStatus::Running) { return TuneFail::NotReady; }
  if (!isIdleClosedLoop(page6.iacAlgorithm)) { return TuneFail::WrongMode; }

  const idleTuneInfo_t idle = getIdleTuneInfo();
  if (!idle.isReady || !isThrottleClosed(current, page2) || current.isDFCOActive || (current.RPM < IDLE_MIN_RPM)) { return TuneFail::NotReady; }
  // The stepper stops short of its target by the hysteresis, so smaller moves barely reach the engine
  if (idle.isStepper && (page15.pidTuneIdleAmplitude <= (2U * page6.iacStepHyster))) { return TuneFail::AmplitudeBelowHysteresis; }

  const int32_t amplitude = idle.isStepper ? ((int32_t)page15.pidTuneIdleAmplitude << 2) : (int32_t)percentage(page15.pidTuneIdleAmplitude, (uint32_t)idle.fullScale);
  if ((amplitude <= 0) || ((idle.output - amplitude) < idle.outputMin) || ((idle.output + amplitude) > idle.outputMax)) { return TuneFail::NoAuthority; }

  relayConfig_t config = {};
  config.base = idle.output;
  config.amplitude = amplitude;
  config.hysteresis = page15.pidTuneIdleHyst;
  config.maxExcursion = IDLE_MAX_EXCURSION;
  config.timeoutMs = (uint32_t)page15.pidTuneTimeout * 1000UL;
  config.noSwitchMs = IDLE_NO_SWITCH_MS;
  config.direction = 1; // More air, more RPM
  config.cycles = page15.pidTuneCycles;
  startTune(TuneLoop::Idle, config, current, nowMs);
  pidTuneState.isVelocityForm = idle.isVelocityForm;
  pidTuneState.iacAlgorithm = page6.iacAlgorithm;
  return TuneFail::None;
}

static TuneFail startEgoTune(const statuses &current, const config6 &page6, const config15 &page15, uint32_t nowMs)
{
  if (current.rotationStatus != EngineRotationStatus::Running) { return TuneFail::NotReady; }
  // The relay needs to see how far the AFR moves: a narrowband sensor can't tell
  if ((page6.egoAlgorithm != EGO_ALGORITHM_PID) || (page6.egoType != EGO_TYPE_WIDE)) { return TuneFail::WrongMode; }
  if (!isAfrClosedLoopActive()) { return TuneFail::NotReady; }

  const int16_t base = current.egoCorrection;
  const int16_t offset = base - (int16_t)ONE_HUNDRED_PCT;
  const int16_t amplitude = page15.pidTuneEgoAmplitude;
  const int16_t headroom = (int16_t)page6.egoLimit - ((offset < 0) ? -offset : offset);
  if ((amplitude <= 0) || (amplitude > headroom)) { return TuneFail::NoAuthority; }

  relayConfig_t config = {};
  config.base = base;
  config.amplitude = amplitude;
  config.hysteresis = page15.pidTuneEgoHyst;
  config.maxExcursion = EGO_MAX_EXCURSION;
  config.timeoutMs = (uint32_t)page15.pidTuneTimeout * 1000UL;
  config.noSwitchMs = EGO_NO_SWITCH_MS;
  config.direction = -1; // More fuel, lower AFR
  config.cycles = page15.pidTuneCycles;
  startTune(TuneLoop::Ego, config, current, nowMs);
  pidTuneState.isVelocityForm = false;
  return TuneFail::None;
}

static TuneFail checkIdleConditions(const statuses &current, const config2 &page2, const config6 &page6)
{
  if (page6.iacAlgorithm != pidTuneState.iacAlgorithm) { return TuneFail::WrongMode; }
  if ((current.rotationStatus != EngineRotationStatus::Running) || !getIdleTuneInfo().isReady) { return TuneFail::ConditionLost; }
  if (!isThrottleClosed(current, page2) || current.isDFCOActive || (current.RPM < IDLE_MIN_RPM)) { return TuneFail::ConditionLost; }
  // A load switching on or off would show up as a change of the loop
  if ((isAirConOn(current) != pidTuneState.wasAirConOn) || (current.idleUpActive != pidTuneState.wasIdleUpOn) || ((current.fanDuty > 0U) != pidTuneState.wasFanOn))
  {
    return TuneFail::ConditionLost;
  }
  return TuneFail::None;
}

static TuneFail checkEgoConditions(const statuses &current, const config6 &page6, uint32_t nowMs)
{
  if ((page6.egoAlgorithm != EGO_ALGORITHM_PID) || (page6.egoType != EGO_TYPE_WIDE)) { return TuneFail::WrongMode; }
  if ((current.rotationStatus != EngineRotationStatus::Running) || !isAfrClosedLoopActive()) { return TuneFail::ConditionLost; }
  // The operating point (and so the loop) must stay the same
  const uint16_t rpmWindow = (uint16_t)(((uint32_t)pidTuneState.startRpm * EGO_RPM_WINDOW_PCT) / 100U);
  if ((absDifference(current.RPM, pidTuneState.startRpm) > rpmWindow)
   || (absDifference(current.MAP, pidTuneState.startMap) > EGO_MAP_WINDOW)
   || (absDifference(current.TPS, pidTuneState.startTps) > EGO_TPS_WINDOW))
  {
    return TuneFail::ConditionLost;
  }
  if (hasIntervalElapsed(nowMs, pidTuneState.lastSampleMs, EGO_STEP_TIMEOUT_MS)) { return TuneFail::StepPeriodTooLong; }
  return TuneFail::None;
}

// ---------------------------------------------------------------- Gain adaptation

static void storeGains(adaptLoop_t &loop, uint8_t kp, uint8_t ki, uint8_t kd)
{
  loop.gains[0] = kp;
  loop.gains[1] = ki;
  loop.gains[2] = kd;
}

/** @brief Multiplier in use: 100% when the adaptation is off */
static uint8_t getAppliedMultiplier(const adaptLoop_t &loop, bool isEnabled)
{
  return isEnabled ? loop.adapter.getMultiplier() : GainAdapter::DEFAULT_MULTIPLIER;
}

static void initialiseAdaptation(adaptLoop_t &loop, uint8_t storedMultiplier, uint8_t kp, uint8_t ki, uint8_t kd, bool isEnabled)
{
  loop.adapter.setMultiplier(storedMultiplier);
  loop.savedMultiplier = loop.adapter.getMultiplier();
  loop.appliedMultiplier = getAppliedMultiplier(loop, isEnabled);
  storeGains(loop, kp, ki, kd);
}

/** @brief Back to 100%, saved straight away (E.g. reset by the user) */
static void resetAdaptation(adaptLoop_t &loop, void (*pSave)(uint8_t))
{
  loop.adapter.reset();
  if (loop.savedMultiplier != loop.adapter.getMultiplier())
  {
    loop.savedMultiplier = loop.adapter.getMultiplier();
    pSave(loop.savedMultiplier);
  }
}

/**
 * @brief Gain adaptation housekeeping
 * - P, I or D changed: back to 100%, the multiplier was for other gains
 * - The multiplier in use changed: the next gain change mustn't make the output jump
 * - Save the multiplier once it has been stable for a while, or when the engine stops
 */
static void updateAdaptation(adaptLoop_t &loop, bool isEnabled, uint8_t kp, uint8_t ki, uint8_t kd, bool isStopping, uint32_t nowMs, bool &isBumpless, void (*pSave)(uint8_t))
{
  if ((loop.gains[0] != kp) || (loop.gains[1] != ki) || (loop.gains[2] != kd))
  {
    storeGains(loop, kp, ki, kd);
    loop.adapter.reset();
  }

  const uint8_t applied = getAppliedMultiplier(loop, isEnabled);
  if (applied != loop.appliedMultiplier)
  {
    loop.appliedMultiplier = applied;
    loop.changedMs = nowMs;
    isBumpless = true;
  }

  const uint8_t multiplier = loop.adapter.getMultiplier();
  const bool isSaveDue = !loop.hasSaved || hasIntervalElapsed(nowMs, loop.savedMs, ADAPT_SAVE_INTERVAL_MS);
  if ((multiplier != loop.savedMultiplier) && (isStopping || (isSaveDue && hasIntervalElapsed(nowMs, loop.changedMs, ADAPT_SAVE_DELAY_MS))))
  {
    loop.savedMultiplier = multiplier;
    loop.hasSaved = true;
    loop.savedMs = nowMs;
    pSave(multiplier);
  }
}

static uint8_t getIdleLoads(const statuses &current)
{
  return (uint8_t)((isAirConOn(current) ? 1U : 0U) | (current.idleUpActive ? 2U : 0U) | ((current.fanDuty > 0U) ? 4U : 0U));
}

/** @brief The result still fits the loop: same algorithm kind as when the test ran */
static bool isResultValidFor(const config6 &page6)
{
  if (pidTuneState.loop == TuneLoop::Idle)
  {
    // The bytes are in the units of the valve (PWM counts or steps) and mean something else in stepper closed loop
    return page6.iacAlgorithm == pidTuneState.iacAlgorithm;
  }
  return (page6.egoAlgorithm == EGO_ALGORITHM_PID) && (page6.egoType == EGO_TYPE_WIDE);
}

static void applyResult(config6 &page6)
{
  if ((pidTuneState.tuner.getPhase() == TunePhase::Done) && !pidTuneState.isApplied)
  {
    if (!isResultValidFor(page6))
    {
      // The loop was changed since the test
      pidTuneState.tuner.fail(TuneFail::WrongMode);
    }
    else if (pidTuneState.loop == TuneLoop::Idle)
    {
      page6.idleKP = pidTuneState.result.kp;
      page6.idleKI = pidTuneState.result.ki;
      page6.idleKD = pidTuneState.result.kd;
      pidTuneState.isIdleBumpless = true;
      // New gains: the adaptation starts again from 100%
      storeGains(pidTuneState.idleAdapt, page6.idleKP, page6.idleKI, page6.idleKD);
      resetAdaptation(pidTuneState.idleAdapt, &savePidAdaptIdle);
    }
    else
    {
      page6.egoKP = pidTuneState.result.kp;
      page6.egoKI = pidTuneState.result.ki;
      page6.egoKD = pidTuneState.result.kd;
      pidTuneState.isEgoBumpless = true;
      storeGains(pidTuneState.egoAdapt, page6.egoKP, page6.egoKI, page6.egoKD);
      resetAdaptation(pidTuneState.egoAdapt, &savePidAdaptEgo);
    }

    if (pidTuneState.tuner.getPhase() == TunePhase::Done)
    {
      // TunerStudio doesn't burn values it reloads from the controller: burn them here, and only them (the rest of
      // the page may have changes not burned yet). Then ask TunerStudio to reload the page.
      const byte *pGains = (pidTuneState.loop == TuneLoop::Idle) ? &page6.idleKP : &page6.egoKP;
      saveConfig6Bytes(page6, pGains, 3U);
      pidTuneState.isApplied = true;
      pidTuneState.isUiRefresh = true;
    }
  }
}

TESTABLE_INLINE_STATIC void pidTuneControlCore(statuses &current, const config2 &page2, const config6 &page6, const config15 &page15, uint32_t nowMs)
{
  if (pidTuneState.tuner.isRunning())
  {
    const TuneFail reason = (pidTuneState.loop == TuneLoop::Idle) ? checkIdleConditions(current, page2, page6) : checkEgoConditions(current, page6, nowMs);
    if (reason != TuneFail::None) { pidTuneState.tuner.abort(reason); }
    else { pidTuneState.tuner.update(nowMs); }
  }
  finishIfStopped(page15);
  // Another rule chosen after the test: the same measurements give other values, so rules can be compared before applying
  if ((pidTuneState.tuner.getPhase() == TunePhase::Done) && !pidTuneState.isApplied && (page15.pidTuneRule != pidTuneState.resultRule))
  {
    computeResult(page15);
  }

  const bool isRunning = (current.rotationStatus == EngineRotationStatus::Running);
  const bool isStopping = pidTuneState.wasRunning && !isRunning;
  pidTuneState.wasRunning = isRunning;
  updateAdaptation(pidTuneState.idleAdapt, page15.pidAdaptIdle != 0U, page6.idleKP, page6.idleKI, page6.idleKD, isStopping, nowMs, pidTuneState.isIdleBumpless, &savePidAdaptIdle);
  updateAdaptation(pidTuneState.egoAdapt, page15.pidAdaptEgo != 0U, page6.egoKP, page6.egoKI, page6.egoKD, isStopping, nowMs, pidTuneState.isEgoBumpless, &savePidAdaptEgo);
}

TESTABLE_INLINE_STATIC bool pidTuneCommandCore(uint16_t command, statuses &current, const config2 &page2, config6 &page6, const config15 &page15, uint32_t nowMs)
{
  if (highByte(command) != highByte(TS_CMD_PIDTUNE_IDLE_START)) { return false; }

  switch (command)
  {
    case TS_CMD_PIDTUNE_IDLE_START:
    case TS_CMD_PIDTUNE_EGO_START:
      // One test at a time
      if (!pidTuneState.tuner.isRunning())
      {
        const TuneLoop loop = (command == TS_CMD_PIDTUNE_IDLE_START) ? TuneLoop::Idle : TuneLoop::Ego;
        const TuneFail reason = (loop == TuneLoop::Idle) ? startIdleTune(current, page2, page6, page15, nowMs) : startEgoTune(current, page6, page15, nowMs);
        if (reason != TuneFail::None) { refuseTune(loop, reason); }
      }
      break;

    case TS_CMD_PIDTUNE_ABORT:
      pidTuneState.tuner.abort(TuneFail::Aborted);
      finishIfStopped(page15);
      break;

    case TS_CMD_PIDTUNE_APPLY:
      applyResult(page6);
      break;

    case TS_CMD_PIDTUNE_ADAPT_RESET_IDLE:
      resetAdaptation(pidTuneState.idleAdapt, &savePidAdaptIdle);
      break;

    case TS_CMD_PIDTUNE_ADAPT_RESET_EGO:
      resetAdaptation(pidTuneState.egoAdapt, &savePidAdaptEgo);
      break;

    default:
      // Unknown command of this group: nothing for anyone else
      break;
  }
  return true;
}

void initialisePidTune(const config6 &page6, const config15 &page15)
{
  pidTuneState = state_t();
  initialiseAdaptation(pidTuneState.idleAdapt, loadPidAdaptIdle(), page6.idleKP, page6.idleKI, page6.idleKD, page15.pidAdaptIdle != 0U);
  initialiseAdaptation(pidTuneState.egoAdapt, loadPidAdaptEgo(), page6.egoKP, page6.egoKI, page6.egoKD, page15.pidAdaptEgo != 0U);
}

bool isPidTuneIdleRunning(void)
{
  return pidTuneState.tuner.isRunning() && (pidTuneState.loop == TuneLoop::Idle);
}

bool pidTuneIdleRelayStep(uint32_t nowMs, uint16_t rpm, int32_t realizedOutput, int32_t &output)
{
  // Same rate as the idle PID
  if (!hasIntervalElapsed(nowMs, pidTuneState.lastSampleMs, IDLE_SAMPLE_MS)) { return false; }
  pidTuneState.lastSampleMs = nowMs;

  const int16_t processValue = (rpm > (uint16_t)INT16_MAX) ? INT16_MAX : (int16_t)rpm;
  output = pidTuneState.tuner.sample(processValue, realizedOutput, nowMs);
  finishIfStopped(configPage15);
  return true;
}

PidTuningParameters pidTuneIdleGains(const config6 &page6)
{
  const PidTuningParameters gains(page6.idleKP, page6.idleKI, page6.idleKD);
  return (configPage15.pidAdaptIdle != 0U) ? pidTuneState.idleAdapt.adapter.apply(gains) : gains;
}

void pidTuneIdleAdaptSample(const statuses &current, int16_t error, int32_t output, uint32_t nowMs)
{
  if (configPage15.pidAdaptIdle == 0U) { return; }

  const idleTuneInfo_t idle = getIdleTuneInfo();
  const uint8_t loads = getIdleLoads(current);
  // A load switching on or off disturbs the loop. At the output limits the loop can't do more, whatever the gains.
  const bool isSteady = idle.isReady
                     && !pidTuneState.tuner.isRunning()
                     && isThrottleClosed(current, configPage2)
                     && !current.isDFCOActive
                     && (loads == pidTuneState.idleAdaptLoads)
                     && (output > idle.outputMin)
                     && (output < idle.outputMax);
  pidTuneState.idleAdaptLoads = loads;

  gainAdaptConfig_t config;
  config.band = configPage15.pidAdaptIdleBand;
  config.windowMs = (uint32_t)configPage15.pidAdaptWindow * 1000UL;
  config.stepPct = configPage15.pidAdaptStep;
  if (pidTuneState.idleAdapt.adapter.sample(error, isSteady, nowMs, config)) { pidTuneState.isIdleBumpless = true; }
}

bool pidTuneIdleTakeBumpless(void)
{
  const bool isBumpless = pidTuneState.isIdleBumpless;
  pidTuneState.isIdleBumpless = false;
  return isBumpless;
}

bool isPidTuneEgoRunning(void)
{
  return pidTuneState.tuner.isRunning() && (pidTuneState.loop == TuneLoop::Ego);
}

uint8_t pidTuneEgoRelayStep(const statuses &current, uint32_t nowMs)
{
  pidTuneState.lastSampleMs = nowMs;
  // The correction reaches the injectors as commanded. Not the last correction: a step outside the closed loop window gives 100.
  const int32_t output = pidTuneState.tuner.sample(current.O2, pidTuneState.tuner.getOutput(), nowMs);
  finishIfStopped(configPage15);
  return (uint8_t)output;
}

PidTuningParameters pidTuneEgoGains(const config6 &page6)
{
  const PidTuningParameters gains(page6.egoKP, page6.egoKI, page6.egoKD);
  return (configPage15.pidAdaptEgo != 0U) ? pidTuneState.egoAdapt.adapter.apply(gains) : gains;
}

void pidTuneEgoAdaptSample(const statuses &current, const config6 &page6, uint8_t correction, uint32_t nowMs)
{
  if (configPage15.pidAdaptEgo == 0U) { return; }

  const int16_t offset = (int16_t)correction - (int16_t)ONE_HUNDRED_PCT;
  const bool isSaturated = (((offset < 0) ? -offset : offset) >= (int16_t)page6.egoLimit);
  // The loop behaves differently at another operating point: start a new window
  if (!pidTuneState.egoAdapt.adapter.isWindowRunning()) { pidTuneState.egoAdaptRpm = current.RPM; }
  const uint16_t rpmWindow = (uint16_t)(((uint32_t)pidTuneState.egoAdaptRpm * EGO_ADAPT_RPM_WINDOW_PCT) / 100U);
  const bool isSteady = !pidTuneState.tuner.isRunning()
                     && !isSaturated
                     && !current.isAcceleratingTPS
                     && !current.isDeceleratingTPS
                     && !current.aseIsActive
                     && (absDifference(current.RPM, pidTuneState.egoAdaptRpm) <= rpmWindow);

  gainAdaptConfig_t config;
  config.band = configPage15.pidAdaptEgoBand;
  config.windowMs = (uint32_t)configPage15.pidAdaptWindow * 1000UL;
  config.stepPct = configPage15.pidAdaptStep;
  const int16_t error = (int16_t)current.afrTarget - (int16_t)current.O2;
  if (pidTuneState.egoAdapt.adapter.sample(error, isSteady, nowMs, config)) { pidTuneState.isEgoBumpless = true; }
}

bool pidTuneEgoTakeBumpless(void)
{
  const bool isBumpless = pidTuneState.isEgoBumpless;
  pidTuneState.isEgoBumpless = false;
  return isBumpless;
}

static uint8_t getStatusByte(void)
{
  static_assert((uint8_t)TunePhase::Failed <= 0x07U, "Phase does not fit in 3 bits");
  uint8_t status = (uint8_t)pidTuneState.tuner.getPhase();
  if (pidTuneState.loop == TuneLoop::Ego) { status |= (1U << 3U); }
  if (pidTuneState.isUiRefresh) { status |= (1U << 4U); }
  if (pidTuneState.result.saturated) { status |= (1U << 5U); }
  if (pidTuneState.isApplied) { status |= (1U << 6U); }
  if (pidTuneState.result.quantised) { status |= (1U << 7U); }
  return status;
}

static uint8_t getAdaptStatusByte(void)
{
  static_assert((uint8_t)AdaptDecision::Hold <= 0x07U, "Decision does not fit in 3 bits");
  uint8_t status = (uint8_t)((uint8_t)pidTuneState.idleAdapt.adapter.getLastDecision() << 2U)
                 | (uint8_t)((uint8_t)pidTuneState.egoAdapt.adapter.getLastDecision() << 5U);
  if (pidTuneState.idleAdapt.adapter.isWindowRunning()) { status |= 0x01U; }
  if (pidTuneState.egoAdapt.adapter.isWindowRunning()) { status |= 0x02U; }
  return status;
}

uint8_t getPidTuneLogByte(uint8_t index)
{
  uint8_t value = 0U;
  switch (index)
  {
    case 0U: value = getStatusByte(); break;
    case 1U: value = (uint8_t)pidTuneState.tuner.getFailure(); break;
    case 2U: value = pidTuneState.result.kp; break;
    case 3U: value = pidTuneState.result.ki; break;
    case 4U: value = pidTuneState.result.kd; break;
    case 5U: value = lowByte(pidTuneState.tuner.getPeriodMs()); break;
    case 6U: value = highByte(pidTuneState.tuner.getPeriodMs()); break;
    case 7U: value = lowByte(pidTuneState.tuner.getAmplitude()); break;
    case 8U: value = highByte(pidTuneState.tuner.getAmplitude()); break;
    case 9U: value = lowByte(pidTuneState.ultimateGainQ8); break;
    case 10U: value = highByte(pidTuneState.ultimateGainQ8); break;
    case 11U: value = pidTuneState.tuner.getCycles(); break;
    case 12U: value = pidTuneState.idleAdapt.appliedMultiplier; break;
    case 13U: value = pidTuneState.egoAdapt.appliedMultiplier; break;
    case 14U: value = getAdaptStatusByte(); break;
    default: value = 0U; break;
  }
  return value;
}

void pidTuneAckUiRefresh(uint16_t offset, uint16_t length)
{
  if ((offset <= PIDTUNE_LOG_FIRST_BYTE) && (((uint32_t)offset + length) > PIDTUNE_LOG_FIRST_BYTE))
  {
    pidTuneState.isUiRefresh = false;
  }
}

// LCOV_EXCL_START
void pidTuneControl(statuses &current, const config2 &page2, const config6 &page6, const config15 &page15)
{
  pidTuneControlCore(current, page2, page6, page15, millis());
}

bool handlePidTuneCommand(uint16_t command, statuses &current, config6 &page6, const config15 &page15)
{
  return pidTuneCommandCore(command, current, configPage2, page6, page15, millis());
}
// LCOV_EXCL_STOP
