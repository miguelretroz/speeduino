#pragma once

/**
 * @file
 * @brief PID tuning rules: from the ultimate gain and period of a relay test to the P, I and D bytes of a loop. Fork specific.
 *
 * Gains are fixed point with 12 fractional bits (Q12) and in the units of the loop: output units per process
 * value unit (E.g. quarter steps per RPM). All the maths is integer.
 *
 * The byte conversions follow the scaling of each loop's PID:
 * - Idle (integerPID, 250mS sample): KP = 32 Kc, KI = 32 Kc / Ti[s], KD = 128 Kc Td[s]
 * - AFR (PID, one sample per closed loop step of Ts): KP = 1000 Kc, KI = 1000 Kc Ts / Ti, KD = 100 Kc Td / Ts
 */

#include <stdint.h>

namespace pidTune {

/** @brief Tuning rules, as stored in the config */
enum class TuneRule : uint8_t
{
  TyreusLuybenPI,    ///< Kc = Ku/3.2, Ti = 2.2 Tu. Conservative, little overshoot (default)
  TyreusLuybenPID,   ///< Kc = Ku/2.2, Ti = 2.2 Tu, Td = Tu/6.3
  ZieglerNicholsPI,  ///< Kc = 0.45 Ku, Ti = Tu/1.2. Faster, oscillates more
  ZieglerNicholsPID, ///< Kc = 0.6 Ku, Ti = Tu/2, Td = Tu/8. Aggressive
};

/** @brief Number of fractional bits of the gains */
constexpr uint8_t GAIN_FRACTION_BITS = 12U;

/** @brief Controller designed by a rule */
struct controllerDesign_t
{
  uint32_t kcQ12; ///< Proportional gain (Q12, output units per process value unit)
  uint32_t tiMs;  ///< Integral time
  uint32_t tdMs;  ///< Derivative time. 0 = no derivative
};

/** @brief PID bytes for the config page */
struct tuneBytes_t
{
  uint8_t kp;
  uint8_t ki;
  uint8_t kd;
  bool saturated; ///< A byte was past its limit, so all three were scaled down together (less gain than designed)
  bool quantised; ///< A byte that rounded to 0 was raised to 1 (that term is stronger than designed)
};

/** @brief Ultimate gain from a relay test: Ku = 4 * relay amplitude / (pi * amplitude) (Q12) */
uint32_t computeUltimateGainQ12(uint32_t relayAmplitude, uint16_t amplitude);

/** @brief Controller gain and times given by a rule */
controllerDesign_t designController(TuneRule rule, uint32_t ultimateGainQ12, uint16_t ultimatePeriodMs);

/** @brief Idle bytes for the positional modes: PWM closed loop, PWM and stepper open+closed loop */
tuneBytes_t toIdlePositionalBytes(const controllerDesign_t &design);

/**
 * @brief Idle bytes for stepper closed loop
 *
 * That mode feeds the previous output back as the feed forward term, so the PID works on the change of the
 * output (velocity form): the P byte acts as the integral, the D byte as the proportional (on the RPM) and the
 * I byte would be a double integral. So the result is a PI: the D of the rule can't be used and I is 0.
 */
tuneBytes_t toIdleVelocityBytes(const controllerDesign_t &design);

/** @brief AFR closed loop bytes, given the average time between closed loop steps */
tuneBytes_t toEgoBytes(const controllerDesign_t &design, uint16_t samplePeriodMs);

} // namespace pidTune
