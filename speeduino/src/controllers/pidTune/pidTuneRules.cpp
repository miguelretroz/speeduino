/** @file
 * PID tuning rules. See pidTuneRules.h. Fork specific.
 */
#include "pidTuneRules.h"

// Runs once per tune, so optimize for size.
#pragma GCC optimize("Os")

namespace pidTune {

/** @brief 4/pi (Q12) */
static constexpr uint32_t FOUR_OVER_PI_Q12 = 5215UL;
/** @brief Keeps the products below in range. An ultimate gain of 1024 is far beyond any real loop. */
static constexpr uint32_t MAX_ULTIMATE_GAIN_Q12 = 1UL << 22U;
/** @brief Shortest ultimate period accepted: anything shorter is noise */
static constexpr uint16_t MIN_PERIOD_MS = 100U;
/** @brief Longest AFR closed loop step accepted. Keeps the products below in range. */
static constexpr uint16_t MAX_SAMPLE_PERIOD_MS = 30000U;

/** @brief Highest idle byte */
static constexpr uint8_t IDLE_BYTE_LIMIT = 255U;
/** @brief Highest AFR byte (the limit of the TunerStudio fields) */
static constexpr uint8_t EGO_BYTE_LIMIT = 200U;
/** @brief Idle gain that gives a KP of 255 (Q12): 255 / 32 */
static constexpr uint32_t IDLE_MAX_KC_Q12 = ((uint32_t)IDLE_BYTE_LIMIT << GAIN_FRACTION_BITS) / 32U;
/** @brief AFR gain that gives a KP of 200 (Q12): 200 / 1000 */
static constexpr uint32_t EGO_MAX_KC_Q12 = ((uint32_t)EGO_BYTE_LIMIT << GAIN_FRACTION_BITS) / 1000U;

/** @brief Rule factors (x1024): Kc/Ku, Ti/Tu, Td/Tu */
struct ruleFactors_t
{
  uint16_t kc;
  uint16_t ti;
  uint16_t td;
};

static ruleFactors_t getRuleFactors(TuneRule rule)
{
  switch (rule)
  {
    case TuneRule::TyreusLuybenPID:   return { 465U, 2253U, 163U }; // 1/2.2, 2.2, 1/6.3
    case TuneRule::ZieglerNicholsPI:  return { 461U, 853U, 0U };    // 0.45, 1/1.2
    case TuneRule::ZieglerNicholsPID: return { 614U, 512U, 128U };  // 0.6, 0.5, 0.125
    case TuneRule::TyreusLuybenPI:
    default:                          return { 320U, 2253U, 0U };   // 1/3.2, 2.2
  }
}

static uint32_t divideRounded(uint32_t numerator, uint32_t denominator)
{
  return (numerator + (denominator / 2U)) / denominator;
}

static uint32_t maxOf(uint32_t a, uint32_t b)
{
  return (a > b) ? a : b;
}

/**
 * @brief Limit the gain to the one that gives the highest proportional byte
 *
 * Every byte is proportional to the gain, so this is the same as scaling them all down together.
 */
static uint32_t limitGain(uint32_t kcQ12, uint32_t maxKcQ12, tuneBytes_t &bytes)
{
  if (kcQ12 > maxKcQ12)
  {
    bytes.saturated = true;
    return maxKcQ12;
  }
  return kcQ12;
}

/**
 * @brief Store the bytes, scaling all three down together if one is past the limit (that keeps Ti and Td).
 * The bytes that set the proportional and integral action are raised to 1 if they rounded to 0.
 */
static void storeBytes(uint32_t kp, uint32_t ki, uint32_t kd, uint8_t limit, bool hasGain, tuneBytes_t &bytes, bool isVelocityForm)
{
  const uint32_t largest = maxOf(kp, maxOf(ki, kd));
  if (largest > limit)
  {
    kp = divideRounded(kp * limit, largest);
    ki = divideRounded(ki * limit, largest);
    kd = divideRounded(kd * limit, largest);
    bytes.saturated = true;
  }

  if (hasGain)
  {
    // In the velocity form the P byte is the integral action and the D byte the proportional one
    uint32_t &proportional = isVelocityForm ? kd : kp;
    uint32_t &integral = isVelocityForm ? kp : ki;
    if (proportional == 0U) { proportional = 1U; bytes.quantised = true; }
    if (integral == 0U) { integral = 1U; bytes.quantised = true; }
  }

  bytes.kp = (uint8_t)kp;
  bytes.ki = (uint8_t)ki;
  bytes.kd = (uint8_t)kd;
}

uint32_t computeUltimateGainQ12(uint32_t relayAmplitude, uint16_t amplitude)
{
  if (amplitude == 0U) { return MAX_ULTIMATE_GAIN_Q12; }
  // Can't overflow: UINT16_MAX * 5215 < UINT32_MAX
  const uint32_t relay = (relayAmplitude > UINT16_MAX) ? (uint32_t)UINT16_MAX : relayAmplitude;
  const uint32_t gain = divideRounded(relay * FOUR_OVER_PI_Q12, amplitude);
  return (gain > MAX_ULTIMATE_GAIN_Q12) ? MAX_ULTIMATE_GAIN_Q12 : gain;
}

controllerDesign_t designController(TuneRule rule, uint32_t ultimateGainQ12, uint16_t ultimatePeriodMs)
{
  const ruleFactors_t factors = getRuleFactors(rule);
  const uint32_t gain = (ultimateGainQ12 > MAX_ULTIMATE_GAIN_Q12) ? MAX_ULTIMATE_GAIN_Q12 : ultimateGainQ12;
  const uint32_t period = (ultimatePeriodMs < MIN_PERIOD_MS) ? MIN_PERIOD_MS : ultimatePeriodMs;

  controllerDesign_t design;
  design.kcQ12 = ((gain * factors.kc) + 512U) >> 10U;
  design.tiMs = ((period * factors.ti) + 512U) >> 10U;
  design.tdMs = ((period * factors.td) + 512U) >> 10U;
  return design;
}

tuneBytes_t toIdlePositionalBytes(const controllerDesign_t &design)
{
  tuneBytes_t bytes = {};
  const uint32_t kc = limitGain(design.kcQ12, IDLE_MAX_KC_Q12, bytes);
  const uint32_t ti = maxOf(design.tiMs, 1U);

  const uint32_t kp = divideRounded(kc, 128U);                // 32 Kc
  const uint32_t ki = divideRounded(kc * 125U, 16U * ti);     // 32 Kc / Ti[s] = 32000 Kc / Ti[mS]
  const uint32_t kd = divideRounded(kc * design.tdMs, 32000U); // 128 Kc Td[s] = 0.128 Kc Td[mS]
  storeBytes(kp, ki, kd, IDLE_BYTE_LIMIT, kc > 0U, bytes, false);
  return bytes;
}

tuneBytes_t toIdleVelocityBytes(const controllerDesign_t &design)
{
  tuneBytes_t bytes = {};
  const uint32_t kc = limitGain(design.kcQ12, IDLE_MAX_KC_Q12, bytes);
  const uint32_t ti = maxOf(design.tiMs, 1U);

  const uint32_t kd = divideRounded(kc, 128U);            // 32 Kc
  const uint32_t kp = divideRounded(kc * 125U, 64U * ti); // 8 Kc / Ti[s] = 8000 Kc / Ti[mS]
  storeBytes(kp, 0U, kd, IDLE_BYTE_LIMIT, kc > 0U, bytes, true);
  return bytes;
}

tuneBytes_t toEgoBytes(const controllerDesign_t &design, uint16_t samplePeriodMs)
{
  tuneBytes_t bytes = {};
  const uint32_t kc = limitGain(design.kcQ12, EGO_MAX_KC_Q12, bytes);
  const uint32_t ti = maxOf(design.tiMs, 1U);
  const uint32_t ts = (samplePeriodMs == 0U) ? 1U : ((samplePeriodMs > MAX_SAMPLE_PERIOD_MS) ? MAX_SAMPLE_PERIOD_MS : samplePeriodMs);

  const uint32_t kp = divideRounded(kc * 125U, 512U);                   // 1000 Kc
  const uint32_t ki = divideRounded(kc * 125U * ts, 512U * ti);         // 1000 Kc Ts / Ti
  const uint32_t kd = divideRounded(kc * 25U * design.tdMs, 1024U * ts); // 100 Kc Td / Ts
  storeBytes(kp, ki, kd, EGO_BYTE_LIMIT, kc > 0U, bytes, false);
  return bytes;
}

} // namespace pidTune
