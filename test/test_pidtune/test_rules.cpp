#include <unity.h>
#include "src/controllers/pidTune/pidTuneRules.h"
#include "src/controllers/pidTune/relayTuner.h"
#include "../test_utils.h"

using namespace pidTune;

static void assert_bytes(uint8_t kp, uint8_t ki, uint8_t kd, bool saturated, bool quantised, const tuneBytes_t &bytes)
{
    TEST_ASSERT_EQUAL_UINT8(kp, bytes.kp);
    TEST_ASSERT_EQUAL_UINT8(ki, bytes.ki);
    TEST_ASSERT_EQUAL_UINT8(kd, bytes.kd);
    TEST_ASSERT_EQUAL(saturated, bytes.saturated);
    TEST_ASSERT_EQUAL(quantised, bytes.quantised);
}

static void test_ultimate_gain(void)
{
    // 4 * 20 / (pi * 45) = 0.566
    TEST_ASSERT_EQUAL_UINT32(2318U, computeUltimateGainQ12(20U, 45U));
    // 4 * 125 / (pi * 50) = 3.183
    TEST_ASSERT_EQUAL_UINT32(13038U, computeUltimateGainQ12(125U, 50U));
    // 4 * 4 / (pi * 6) = 0.849
    TEST_ASSERT_EQUAL_UINT32(3477U, computeUltimateGainQ12(4U, 6U));
    // No oscillation measured: as high as it goes
    TEST_ASSERT_EQUAL_UINT32(1UL << 22U, computeUltimateGainQ12(4U, 0U));
    TEST_ASSERT_EQUAL_UINT32(1UL << 22U, computeUltimateGainQ12(UINT32_MAX, 1U));
}

static void test_design_rules(void)
{
    // Ku 3.183, Tu 2S
    controllerDesign_t design = designController(TuneRule::BalancedPI, 13038U, 2000U);
    TEST_ASSERT_EQUAL_UINT32(3909U, design.kcQ12); // 0.3 Ku
    TEST_ASSERT_EQUAL_UINT32(2000U, design.tiMs);  // Tu
    TEST_ASSERT_EQUAL_UINT32(0U, design.tdMs);

    design = designController(TuneRule::GentlePI, 13038U, 2000U);
    TEST_ASSERT_EQUAL_UINT32(4074U, design.kcQ12); // Ku/3.2
    TEST_ASSERT_EQUAL_UINT32(4400U, design.tiMs);  // 2.2 Tu
    TEST_ASSERT_EQUAL_UINT32(0U, design.tdMs);

    design = designController(TuneRule::FastPI, 13038U, 2000U);
    TEST_ASSERT_EQUAL_UINT32(5870U, design.kcQ12); // 0.45 Ku
    TEST_ASSERT_EQUAL_UINT32(1666U, design.tiMs);  // Tu/1.2
    TEST_ASSERT_EQUAL_UINT32(0U, design.tdMs);

    // Not a rule (the config has 2 bits): the default one
    design = designController((TuneRule)3U, 13038U, 2000U);
    TEST_ASSERT_EQUAL_UINT32(3909U, design.kcQ12);
}

static void test_design_limits_the_period(void)
{
    controllerDesign_t design = designController(TuneRule::GentlePI, 4096U, 50U);
    TEST_ASSERT_EQUAL_UINT32(220U, design.tiMs); // 100mS * 2.2
}

static void test_idle_positional_bytes(void)
{
    // PWM valve: 5% relay (125), +/-50 RPM, 2S period
    assert_bytes(31U, 15U, 0U, false, false, toIdlePositionalBytes(designController(TuneRule::BalancedPI, 13038U, 2000U)));
    assert_bytes(32U, 7U, 0U, false, false, toIdlePositionalBytes(designController(TuneRule::GentlePI, 13038U, 2000U)));
    // With a derivative time (no rule has one, but the conversion supports it): KD = 128 Kc Td
    const controllerDesign_t withD = { 5921U, 4400U, 318U };
    assert_bytes(46U, 11U, 59U, false, false, toIdlePositionalBytes(withD));
}

static void test_idle_velocity_bytes(void)
{
    // Stepper: +/-5 steps realized (20 quarter steps), +/-45 RPM, 2.5S period.
    // The I action (P byte) rounds to 0, so it's raised to 1.
    controllerDesign_t design = designController(TuneRule::GentlePI, computeUltimateGainQ12(20U, 45U), 2500U);
    TEST_ASSERT_EQUAL_UINT32(724U, design.kcQ12);
    TEST_ASSERT_EQUAL_UINT32(5500U, design.tiMs);
    assert_bytes(1U, 0U, 6U, false, true, toIdleVelocityBytes(design));

    // A shorter integral time fits without rounding to 0: KP = 8 * Kc / Ti = 8 * 0.5 / 0.5
    controllerDesign_t fast = { 2048U, 500U, 0U };
    assert_bytes(8U, 0U, 16U, false, false, toIdleVelocityBytes(fast));
}

static void test_ego_bytes(void)
{
    // Wideband: 4% relay, +/-0.6 AFR, 1.2S period, a closed loop step every 133mS. The P byte saturates.
    assert_bytes(200U, 10U, 0U, true, false, toEgoBytes(designController(TuneRule::GentlePI, 3477U, 1200U), 133U));
    assert_bytes(200U, 27U, 0U, true, false, toEgoBytes(designController(TuneRule::FastPI, 3477U, 1200U), 133U));
    // With a derivative time: KD = 100 Kc Td / Ts
    const controllerDesign_t withD = { 2085U, 600U, 150U };
    assert_bytes(200U, 44U, 23U, true, false, toEgoBytes(withD, 133U));

    // A less sensitive loop doesn't saturate: 2% relay, +/-2.0 AFR
    assert_bytes(40U, 2U, 0U, false, false, toEgoBytes(designController(TuneRule::GentlePI, computeUltimateGainQ12(2U, 20U), 1200U), 133U));
}

static void test_saturation_scales_all_bytes(void)
{
    // KI would be 320: all three are scaled by 254/320, which keeps Ti. 254 is the limit of the TunerStudio fields.
    controllerDesign_t design = { 4096U, 100U, 0U };
    assert_bytes(25U, 254U, 0U, true, false, toIdlePositionalBytes(design));

    // The gain is past what KP can hold: limited to KP = 254
    design = { 40000U, 100000U, 0U };
    assert_bytes(254U, 3U, 0U, true, false, toIdlePositionalBytes(design));
}

static void test_quantised_bytes_are_raised(void)
{
    // Tiny gain: P and I both round to 0
    controllerDesign_t design = { 50U, 100000U, 0U };
    assert_bytes(1U, 1U, 0U, false, true, toIdlePositionalBytes(design));

    // No gain at all: nothing to raise
    design = { 0U, 100000U, 0U };
    assert_bytes(0U, 0U, 0U, false, false, toIdlePositionalBytes(design));
}

static void test_ego_sample_period_guards(void)
{
    const controllerDesign_t design = { 819U, 2640U, 191U };
    // No division by zero: treated as 1mS, so the D term is huge and the bytes are scaled down to fit it
    tuneBytes_t bytes = toEgoBytes(design, 0U);
    TEST_ASSERT_EQUAL_UINT8(200U, bytes.kd);
    TEST_ASSERT_TRUE(bytes.saturated);
}

// First order plant with dead time: K = 1, time constant 1S, dead time 0.5S.
// Ultimate point: w*0.5 + atan(w*1) = pi -> w = 3.674 rad/S, Tu = 1.71S, Ku = sqrt(1 + w^2) = 3.81
static void test_relay_identifies_a_first_order_plant(void)
{
    constexpr uint32_t STEP_MS = 50U;
    constexpr uint8_t DELAY_STEPS = 10U;
    constexpr float FILTER = (float)STEP_MS / 1000.0f;

    relayConfig_t config = {};
    config.base = 0;
    config.amplitude = 100;
    config.hysteresis = 1U;
    config.maxExcursion = 300U;
    config.timeoutMs = 90000UL;
    config.noSwitchMs = 15000U;
    config.direction = 1;
    config.cycles = 3U;

    RelayTuner tuner;
    int32_t delayLine[DELAY_STEPS] = {};
    uint8_t delayIndex = 0U;
    float plantOutput = 0.0f;
    int32_t output = 0;
    uint32_t nowMs = 0U;
    tuner.start(config, nowMs);
    while (tuner.isRunning())
    {
        const int16_t processValue = (int16_t)(1000.5f + plantOutput);
        output = tuner.sample(processValue, output, nowMs);

        const int32_t delayedOutput = delayLine[delayIndex];
        delayLine[delayIndex] = output;
        delayIndex = (uint8_t)((delayIndex + 1U) % DELAY_STEPS);
        plantOutput += ((float)delayedOutput - plantOutput) * FILTER;
        nowMs += STEP_MS;
    }

    TEST_ASSERT_EQUAL(TunePhase::Done, tuner.getPhase());
    TEST_ASSERT_UINT_WITHIN(260U, 1710U, tuner.getPeriodMs());
    TEST_ASSERT_EQUAL_UINT16(100U, tuner.getRelayAmplitude());
    // The relay method assumes a sine wave, so with a dead time it underestimates Ku: the gains come out
    // lower than the true ultimate point would give, never higher. Within 30% of 3.81 (Q12).
    const uint32_t ultimateGain = computeUltimateGainQ12(tuner.getRelayAmplitude(), tuner.getAmplitude());
    TEST_ASSERT_LESS_OR_EQUAL_UINT32(15606U, ultimateGain);
    TEST_ASSERT_GREATER_OR_EQUAL_UINT32((15606U * 7U) / 10U, ultimateGain);
}

void testPidTuneRules(void)
{
  SET_UNITY_FILENAME() {
    RUN_TEST_P(test_ultimate_gain);
    RUN_TEST_P(test_design_rules);
    RUN_TEST_P(test_design_limits_the_period);
    RUN_TEST_P(test_idle_positional_bytes);
    RUN_TEST_P(test_idle_velocity_bytes);
    RUN_TEST_P(test_ego_bytes);
    RUN_TEST_P(test_saturation_scales_all_bytes);
    RUN_TEST_P(test_quantised_bytes_are_raised);
    RUN_TEST_P(test_ego_sample_period_guards);
    RUN_TEST_P(test_relay_identifies_a_first_order_plant);
  }
}
