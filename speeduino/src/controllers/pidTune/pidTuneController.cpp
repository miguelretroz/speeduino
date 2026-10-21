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
  page15.pidTuneRule = (uint8_t)TuneRule::TyreusLuybenPI;
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
  if (!idle.isReady || (current.TPS > page2.iacTPSlimit) || current.isDFCOActive || (current.RPM < IDLE_MIN_RPM)) { return TuneFail::NotReady; }
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
  if ((current.TPS > page2.iacTPSlimit) || current.isDFCOActive || (current.RPM < IDLE_MIN_RPM)) { return TuneFail::ConditionLost; }
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

static void applyResult(config6 &page6)
{
  if ((pidTuneState.tuner.getPhase() == TunePhase::Done) && !pidTuneState.isApplied)
  {
    if (pidTuneState.loop == TuneLoop::Idle)
    {
      page6.idleKP = pidTuneState.result.kp;
      page6.idleKI = pidTuneState.result.ki;
      page6.idleKD = pidTuneState.result.kd;
      pidTuneState.isIdleBumpless = true;
    }
    else
    {
      page6.egoKP = pidTuneState.result.kp;
      page6.egoKI = pidTuneState.result.ki;
      page6.egoKD = pidTuneState.result.kd;
      pidTuneState.isEgoBumpless = true;
    }
    // TunerStudio won't burn these, as the firmware has priority over them: burn them here and ask it to reload the page
    savePage(afrSetPage);
    pidTuneState.isApplied = true;
    pidTuneState.isUiRefresh = true;
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

    default:
      // Unknown command of this group (E.g. the gain adaptation, when disabled): nothing for anyone else
      break;
  }
  return true;
}

void initialisePidTune(void)
{
  pidTuneState = state_t();
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
  return PidTuningParameters(page6.idleKP, page6.idleKI, page6.idleKD);
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
  const int32_t output = pidTuneState.tuner.sample(current.O2, current.egoCorrection, nowMs);
  finishIfStopped(configPage15);
  return (uint8_t)output;
}

PidTuningParameters pidTuneEgoGains(const config6 &page6)
{
  return PidTuningParameters(page6.egoKP, page6.egoKI, page6.egoKD);
}

bool pidTuneEgoTakeBumpless(void)
{
  const bool isBumpless = pidTuneState.isEgoBumpless;
  pidTuneState.isEgoBumpless = false;
  return isBumpless;
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
