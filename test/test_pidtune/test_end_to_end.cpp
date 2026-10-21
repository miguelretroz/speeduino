#include <unity.h>
#include "../test_utils.h"
#include "src/controllers/pidTune/relayTuner.h"
#include "src/controllers/pidTune/pidTuneRules.h"
#include "src/PID/integerPID.h"
#include "src/PID/PID.h"

// The whole chain on simulated plants: relay test -> rule -> bytes -> the firmware's own PID classes,
// which must then hold the loop on target without oscillating.

using namespace pidTune;

/** @brief First order plant with dead time, sampled at a fixed rate. Output = base + gain * filtered input. */
struct plant_t
{
    static constexpr uint8_t MAX_DELAY = 8U;
    float gain;            ///< Process value units per output unit, once settled
    float filter;          ///< Sample time / time constant
    uint8_t delaySamples;
    float baseOutput;
    float baseValue;
    float state = 0.0f;
    float delayLine[MAX_DELAY] = {};
    uint8_t delayIndex = 0U;

    void reset(void)
    {
        state = 0.0f;
        for (float &value : delayLine) { value = 0.0f; }
        delayIndex = 0U;
    }

    /** @brief The process value for this sample, then apply the output (it's seen after the dead time) */
    int16_t step(int32_t output)
    {
        const float value = baseValue + (gain * state);
        const float delayed = delayLine[delayIndex];
        delayLine[delayIndex] = (float)output - baseOutput;
        delayIndex = (uint8_t)((delayIndex + 1U) % delaySamples);
        state += (delayed - state) * filter;
        return (int16_t)(value + ((value < 0.0f) ? -0.5f : 0.5f));
    }
};

static relayConfig_t makeRelayConfig(int32_t base, int32_t amplitude, uint16_t hysteresis, int8_t direction)
{
    relayConfig_t config = {};
    config.base = base;
    config.amplitude = amplitude;
    config.hysteresis = hysteresis;
    config.maxExcursion = 300U;
    config.timeoutMs = 90000UL;
    config.noSwitchMs = 15000U;
    config.direction = direction;
    config.cycles = 3U;
    return config;
}

/** @brief Run the relay test on the plant, sampled every sampleMs. Returns false if it didn't finish. */
static bool runRelay(RelayTuner &tuner, plant_t &plant, const relayConfig_t &config, uint32_t sampleMs)
{
    plant.reset();
    uint32_t nowMs = 0U;
    int32_t output = config.base;
    tuner.start(config, nowMs);
    while (tuner.isRunning())
    {
        const int16_t value = plant.step(output);
        output = tuner.sample(value, output, nowMs);
        nowMs += sampleMs;
    }
    return tuner.getPhase() == TunePhase::Done;
}

// Idle: the stepper valve (quarter steps) against the RPM. 8 RPM per step, 1S lag, 0.5S dead time.
// The idle PID runs every 250mS.
static plant_t makeIdlePlant(void)
{
    plant_t plant;
    plant.gain = 2.0f;
    plant.filter = 0.25f;
    plant.delaySamples = 2U;
    plant.baseOutput = 400.0f;
    plant.baseValue = 900.0f;
    return plant;
}

/** @brief Run the idle PID with the bytes on the plant for 2 minutes: step the target from 900 to 950 RPM, return the worst error of the last 20S */
static uint16_t runIdlePid(plant_t &plant, const tuneBytes_t &bytes, int16_t &overshoot)
{
    constexpr uint32_t SAMPLE_MS = 250U;
    constexpr uint32_t START_MS = 10000UL;
    constexpr int32_t TARGET = 950;

    integerPID pid;
    pid.setOutputLimits(0, 1200);
    pid.setTunings(PidTuningParameters(bytes.kp, bytes.ki, bytes.kd), START_MS, SAMPLE_MS);
    pid.setSetPoint(TARGET);
    plant.reset();
    int16_t value = plant.step(400);
    pid.activate(value);
    pid.presetOutput(400, value);

    int32_t output = 400;
    uint16_t worstError = 0U;
    overshoot = 0;
    for (uint16_t index = 0U; index < 480U; ++index)
    {
        const uint32_t nowMs = START_MS + (index * SAMPLE_MS);
        TEST_ASSERT_TRUE(pid.compute(nowMs, value, &output));
        value = plant.step(output);
        if ((value - TARGET) > overshoot) { overshoot = (int16_t)(value - TARGET); }
        // The last 20S: settled
        if (index >= 400U)
        {
            const uint16_t error = (uint16_t)abs(value - TARGET);
            if (error > worstError) { worstError = error; }
        }
    }
    return worstError;
}

static void test_idle_end_to_end(TuneRule rule)
{
    plant_t plant = makeIdlePlant();
    RelayTuner tuner;
    // 8 steps, 15 RPM band: the defaults
    TEST_ASSERT_TRUE(runRelay(tuner, plant, makeRelayConfig(400, 32, 15U, 1), 250U));

    const controllerDesign_t design = designController(rule, computeUltimateGainQ12(tuner.getRelayAmplitude(), tuner.getAmplitude()), tuner.getPeriodMs());
    const tuneBytes_t bytes = toIdlePositionalBytes(design);
    TEST_ASSERT_GREATER_THAN_UINT8(0U, bytes.kp);
    TEST_ASSERT_GREATER_THAN_UINT8(0U, bytes.ki);

    int16_t overshoot = 0;
    const uint16_t worstError = runIdlePid(plant, bytes, overshoot);
    // Settled within 5 RPM of the target, with less than 50% overshoot of the 50 RPM step
    TEST_ASSERT_LESS_OR_EQUAL_UINT16(5U, worstError);
    TEST_ASSERT_LESS_THAN_INT16(25, overshoot);
}

static void test_idle_end_to_end_balanced_pi(void) { test_idle_end_to_end(TuneRule::BalancedPI); }
static void test_idle_end_to_end_gentle_pi(void) { test_idle_end_to_end(TuneRule::GentlePI); }
static void test_idle_end_to_end_fast_pi(void) { test_idle_end_to_end(TuneRule::FastPI); }

// Stepper closed loop: the same plant, with the PID in velocity form (the previous output is the feed forward).
// Each change of the output is truncated to whole quarter steps, so errors below 32/P RPM stay: stable, but not exact.
static void test_idle_end_to_end_velocity_form(void)
{
    plant_t plant = makeIdlePlant();
    RelayTuner tuner;
    TEST_ASSERT_TRUE(runRelay(tuner, plant, makeRelayConfig(400, 32, 15U, 1), 250U));
    const controllerDesign_t design = designController(TuneRule::BalancedPI, computeUltimateGainQ12(tuner.getRelayAmplitude(), tuner.getAmplitude()), tuner.getPeriodMs());
    const tuneBytes_t bytes = toIdleVelocityBytes(design);
    TEST_ASSERT_EQUAL_UINT8(0U, bytes.ki);
    TEST_ASSERT_GREATER_THAN_UINT8(0U, bytes.kp);

    constexpr uint32_t SAMPLE_MS = 250U;
    constexpr uint32_t START_MS = 10000UL;
    constexpr int32_t TARGET = 950;
    integerPID pid;
    pid.setOutputLimits(0, 1200);
    pid.setTunings(PidTuningParameters(bytes.kp, bytes.ki, bytes.kd), START_MS, SAMPLE_MS);
    pid.setSetPoint(TARGET);
    plant.reset();
    int16_t value = plant.step(400);
    pid.activate(value);

    int32_t output = 400;
    uint16_t worstError = 0U;
    int16_t lowest = INT16_MAX;
    int16_t highest = INT16_MIN;
    for (uint16_t index = 0U; index < 480U; ++index)
    {
        // As idle.cpp does for stepper closed loop
        pid.setFeedForwardTerm(output);
        TEST_ASSERT_TRUE(pid.compute(START_MS + (index * SAMPLE_MS), value, &output));
        value = plant.step(output);
        if (index >= 400U)
        {
            const uint16_t error = (uint16_t)abs(value - TARGET);
            if (error > worstError) { worstError = error; }
            if (value < lowest) { lowest = value; }
            if (value > highest) { highest = value; }
        }
    }
    TEST_ASSERT_LESS_OR_EQUAL_UINT16((32U / bytes.kp) + 2U, worstError);
    // No oscillation
    TEST_ASSERT_LESS_OR_EQUAL_INT16(4, highest - lowest);
}

/** @brief Deterministic noise, uniform in +/-amplitude */
static int16_t noiseSample(uint32_t &seed, int16_t amplitude)
{
    seed = (seed * 1103515245UL) + 12345UL;
    const int32_t value = (int32_t)((seed >> 16) % (uint32_t)((2 * amplitude) + 1)) - amplitude;
    return (int16_t)value;
}

// The same idle with +/-12 RPM of noise: the test still converges, and the loop then holds the target
static void test_idle_end_to_end_with_noise(void)
{
    constexpr int16_t NOISE = 12;
    plant_t plant = makeIdlePlant();
    uint32_t seed = 1U;

    RelayTuner tuner;
    const relayConfig_t config = makeRelayConfig(400, 32, 15U, 1);
    plant.reset();
    uint32_t nowMs = 0U;
    int32_t output = config.base;
    tuner.start(config, nowMs);
    while (tuner.isRunning())
    {
        const int16_t value = plant.step(output) + noiseSample(seed, NOISE);
        output = tuner.sample(value, output, nowMs);
        nowMs += 250U;
    }
    TEST_ASSERT_EQUAL(TunePhase::Done, tuner.getPhase());
    // Noiseless: Ku 0.91 (Q12 3708). Noise widens the swing, so it reads a little low.
    const uint32_t ultimateGain = computeUltimateGainQ12(tuner.getRelayAmplitude(), tuner.getAmplitude());
    TEST_ASSERT_UINT32_WITHIN(1100U, 3708U, ultimateGain);

    const tuneBytes_t bytes = toIdlePositionalBytes(designController(TuneRule::BalancedPI, ultimateGain, tuner.getPeriodMs()));
    integerPID pid;
    pid.setOutputLimits(0, 1200);
    pid.setTunings(PidTuningParameters(bytes.kp, bytes.ki, bytes.kd), 10000UL, 250U);
    pid.setSetPoint(950);
    plant.reset();
    int16_t value = plant.step(400);
    pid.activate(value);
    pid.presetOutput(400, value);
    uint32_t errorSum = 0U;
    for (uint16_t index = 0U; index < 480U; ++index)
    {
        TEST_ASSERT_TRUE(pid.compute(10000UL + (index * 250UL), value, &output));
        const int16_t trueValue = plant.step(output);
        value = trueValue + noiseSample(seed, NOISE);
        if (index >= 360U) { errorSum += (uint32_t)abs(trueValue - 950); }
    }
    // The last 30S: on target, give or take the noise it reacts to
    TEST_ASSERT_LESS_OR_EQUAL_UINT32(8U * 120U, errorSum);
}

// AFR: the fuel correction (%) against the AFR (x10). 1% fuel = 0.9 x10 AFR (stoich 9), 0.3S lag,
// 0.5S dead time, one closed loop step every 125mS.
static void test_ego_end_to_end(void)
{
    plant_t plant;
    plant.gain = -0.9f;
    plant.filter = 125.0f / 300.0f;
    plant.delaySamples = 4U;
    plant.baseOutput = 100.0f;
    plant.baseValue = 100.0f; // AFR 10.0

    RelayTuner tuner;
    TEST_ASSERT_TRUE(runRelay(tuner, plant, makeRelayConfig(100, 5, 1U, -1), 125U));
    const controllerDesign_t design = designController(TuneRule::BalancedPI, computeUltimateGainQ12(tuner.getRelayAmplitude(), tuner.getAmplitude()), tuner.getPeriodMs());
    const tuneBytes_t bytes = toEgoBytes(design, tuner.getSamplePeriodMs());
    TEST_ASSERT_GREATER_THAN_UINT8(0U, bytes.ki);

    // The firmware's AFR PID: negated gains, +/-15% authority, target 9.0 from 10.0 (needs about +11% fuel)
    PID pid;
    pid.setOutputLimits(-15, 15);
    pid.setTunings(PidTuningParameters(bytes.kp, bytes.ki, bytes.kd) * -1);
    pid.setSetPoint(90U);
    plant.reset();
    int16_t value = plant.step(100);
    pid.resetIntegral(value);

    uint16_t worstError = 0U;
    for (uint16_t index = 0U; index < 480U; ++index) // 60S
    {
        const int32_t correction = 100 + pid.compute(value);
        value = plant.step(correction);
        if (index >= 320U)
        {
            const uint16_t error = (uint16_t)abs(value - 90);
            if (error > worstError) { worstError = error; }
        }
    }
    // Within 0.2 AFR at the end
    TEST_ASSERT_LESS_OR_EQUAL_UINT16(2U, worstError);
}

void testPidTuneEndToEnd(void)
{
  SET_UNITY_FILENAME() {
    RUN_TEST_P(test_idle_end_to_end_balanced_pi);
    RUN_TEST_P(test_idle_end_to_end_gentle_pi);
    RUN_TEST_P(test_idle_end_to_end_fast_pi);
    RUN_TEST_P(test_idle_end_to_end_velocity_form);
    RUN_TEST_P(test_idle_end_to_end_with_noise);
    RUN_TEST_P(test_ego_end_to_end);
  }
}
