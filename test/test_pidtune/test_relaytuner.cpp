#include <unity.h>
#include "src/controllers/pidTune/relayTuner.h"
#include "../test_utils.h"

using namespace pidTune;

static constexpr uint32_t SAMPLE_MS = 250U;
static constexpr int16_t SETPOINT = 1000;
static constexpr int32_t BASE = 100;
static constexpr int32_t AMPLITUDE = 10;

static relayConfig_t makeConfig(void)
{
    relayConfig_t config = {};
    config.base = BASE;
    config.amplitude = AMPLITUDE;
    config.hysteresis = 5U;
    config.maxExcursion = 300U;
    config.timeoutMs = 90000UL;
    config.noSwitchMs = 15000U;
    config.direction = 1;
    config.cycles = 3U;
    return config;
}

// Holds the process value at the setpoint until the relay starts. Returns the time of the last sample.
static uint32_t settle(RelayTuner &tuner, const relayConfig_t &config)
{
    uint32_t nowMs = 0U;
    tuner.start(config, nowMs);
    while (tuner.getPhase() == TunePhase::Settle)
    {
        int32_t output = tuner.sample(SETPOINT, BASE, nowMs);
        // The sample that ends the settle phase already switches the relay
        if (tuner.getPhase() == TunePhase::Settle) { TEST_ASSERT_EQUAL(BASE, output); }
        nowMs += SAMPLE_MS;
    }
    TEST_ASSERT_EQUAL(TunePhase::Relay, tuner.getPhase());
    return nowMs - SAMPLE_MS;
}

// A triangle wave: 3000mS period, +/-40 around the setpoint, sampled every 250mS
static constexpr int8_t TRIANGLE[] = { 0, 13, 27, 40, 27, 13, 0, -13, -27, -40, -27, -13 };

/**
 * @brief Feed the triangle wave until the tuner stops (or the time runs out)
 *
 * @param realizedOffset Simulates an actuator that stops short of the commanded value (E.g. stepper hysteresis)
 */
static void runTriangle(RelayTuner &tuner, uint32_t nowMs, uint32_t untilMs, int32_t realizedOffset)
{
    int32_t output = tuner.getOutput();
    for (uint8_t index = 1U; tuner.isRunning() && (nowMs < untilMs); ++index)
    {
        nowMs += SAMPLE_MS;
        int32_t realized = (output > BASE) ? (output - realizedOffset) : (output + realizedOffset);
        output = tuner.sample(SETPOINT + TRIANGLE[index % _countof(TRIANGLE)], realized, nowMs);
    }
}

static void test_relay_idle_until_started(void)
{
    RelayTuner tuner;
    TEST_ASSERT_EQUAL(TunePhase::Idle, tuner.getPhase());
    TEST_ASSERT_FALSE(tuner.isRunning());

    // Not running: samples are ignored
    TEST_ASSERT_EQUAL(0, tuner.sample(SETPOINT, 0, 0U));
    TEST_ASSERT_EQUAL(TunePhase::Idle, tuner.getPhase());
}

static void test_relay_settle_holds_base_and_averages(void)
{
    RelayTuner tuner;
    relayConfig_t config = makeConfig();
    tuner.start(config, 0U);
    TEST_ASSERT_TRUE(tuner.isRunning());

    // 13 samples over 3S, centred on 1000. The actuator is held until the last one.
    uint32_t nowMs = 0U;
    for (int16_t value = 994; value < 1006; ++value)
    {
        TEST_ASSERT_EQUAL(BASE, tuner.sample(value, BASE, nowMs));
        TEST_ASSERT_EQUAL(TunePhase::Settle, tuner.getPhase());
        nowMs += SAMPLE_MS;
    }
    (void)tuner.sample(1006, BASE, nowMs);

    TEST_ASSERT_EQUAL(TunePhase::Relay, tuner.getPhase());
    TEST_ASSERT_EQUAL(1000, tuner.getSetpoint());
    // The relay kicks the output up first
    TEST_ASSERT_EQUAL(BASE + AMPLITUDE, tuner.getOutput());
}

static void test_relay_switches_outside_the_band(void)
{
    RelayTuner tuner;
    relayConfig_t config = makeConfig();
    uint32_t nowMs = settle(tuner, config);

    // Above the setpoint, but inside the band: stays high
    TEST_ASSERT_EQUAL(BASE + AMPLITUDE, tuner.sample(SETPOINT + 5, BASE, nowMs += SAMPLE_MS));
    // Out of the band: output down
    TEST_ASSERT_EQUAL(BASE - AMPLITUDE, tuner.sample(SETPOINT + 6, BASE, nowMs += SAMPLE_MS));
    // Back inside the band: stays low
    TEST_ASSERT_EQUAL(BASE - AMPLITUDE, tuner.sample(SETPOINT - 5, BASE, nowMs += SAMPLE_MS));
    // Below the band: output up
    TEST_ASSERT_EQUAL(BASE + AMPLITUDE, tuner.sample(SETPOINT - 6, BASE, nowMs += SAMPLE_MS));
}

static void test_relay_switches_reverse_direction(void)
{
    // E.g. AFR: more fuel lowers the AFR
    RelayTuner tuner;
    relayConfig_t config = makeConfig();
    config.direction = -1;
    uint32_t nowMs = settle(tuner, config);

    // Below the band: output down
    TEST_ASSERT_EQUAL(BASE - AMPLITUDE, tuner.sample(SETPOINT - 6, BASE, nowMs += SAMPLE_MS));
    // Above the band: output up
    TEST_ASSERT_EQUAL(BASE + AMPLITUDE, tuner.sample(SETPOINT + 6, BASE, nowMs += SAMPLE_MS));
}

static void test_relay_measures_period_and_amplitude(void)
{
    RelayTuner tuner;
    relayConfig_t config = makeConfig();
    uint32_t nowMs = settle(tuner, config);

    runTriangle(tuner, nowMs, 60000UL, 0);

    TEST_ASSERT_EQUAL(TunePhase::Done, tuner.getPhase());
    TEST_ASSERT_EQUAL(TuneFail::None, tuner.getFailure());
    TEST_ASSERT_EQUAL(3000U, tuner.getPeriodMs());
    TEST_ASSERT_EQUAL(40U, tuner.getAmplitude());
    TEST_ASSERT_EQUAL(AMPLITUDE, tuner.getRelayAmplitude());
    TEST_ASSERT_EQUAL(SAMPLE_MS, tuner.getSamplePeriodMs());
    TEST_ASSERT_EQUAL(3U, tuner.getCycles());
    // Done: back to the base value
    TEST_ASSERT_EQUAL(BASE, tuner.getOutput());
}

static void test_relay_discards_the_first_cycle(void)
{
    RelayTuner tuner;
    relayConfig_t config = makeConfig();
    uint32_t nowMs = settle(tuner, config);

    // The first low to high switch starts a cycle, the next one ends it (and it's discarded),
    // then 3 cycles are measured: 1 + 4 cycles of 12 samples after the first switch high, at sample 7.
    runTriangle(tuner, nowMs, nowMs + (SAMPLE_MS * (7U + (4U * 12U) - 1U)), 0);
    TEST_ASSERT_EQUAL(TunePhase::Relay, tuner.getPhase());
    TEST_ASSERT_EQUAL(2U, tuner.getCycles());
}

static void test_relay_measures_the_realized_amplitude(void)
{
    // An actuator that stops 3 short of the commanded value
    RelayTuner tuner;
    relayConfig_t config = makeConfig();
    uint32_t nowMs = settle(tuner, config);

    runTriangle(tuner, nowMs, 60000UL, 3);

    TEST_ASSERT_EQUAL(TunePhase::Done, tuner.getPhase());
    TEST_ASSERT_EQUAL(AMPLITUDE - 3, tuner.getRelayAmplitude());
}

static void test_relay_cycles_are_clamped(void)
{
    RelayTuner tuner;
    relayConfig_t config = makeConfig();
    config.cycles = 9U;
    uint32_t nowMs = settle(tuner, config);
    runTriangle(tuner, nowMs, 60000UL, 0);
    TEST_ASSERT_EQUAL(TunePhase::Done, tuner.getPhase());
    TEST_ASSERT_EQUAL(RELAY_MAX_CYCLES, tuner.getCycles());

    config.cycles = 0U;
    nowMs = settle(tuner, config);
    runTriangle(tuner, nowMs, 60000UL, 0);
    TEST_ASSERT_EQUAL(TunePhase::Done, tuner.getPhase());
    TEST_ASSERT_EQUAL(RELAY_MIN_CYCLES, tuner.getCycles());
}

static void test_relay_fails_without_switching(void)
{
    RelayTuner tuner;
    relayConfig_t config = makeConfig();
    uint32_t relayStartMs = settle(tuner, config);

    // The process value doesn't move
    uint32_t nowMs = relayStartMs;
    while (nowMs < (relayStartMs + config.noSwitchMs - SAMPLE_MS))
    {
        nowMs += SAMPLE_MS;
        (void)tuner.sample(SETPOINT, BASE, nowMs);
    }
    TEST_ASSERT_EQUAL(TunePhase::Relay, tuner.getPhase());

    // Even without samples (E.g. the loop stopped running), the timeout is checked
    tuner.update(relayStartMs + config.noSwitchMs);
    TEST_ASSERT_EQUAL(TunePhase::Failed, tuner.getPhase());
    TEST_ASSERT_EQUAL(TuneFail::NoSwitching, tuner.getFailure());
    TEST_ASSERT_EQUAL(BASE, tuner.getOutput());
}

static void test_relay_fails_on_large_excursion(void)
{
    RelayTuner tuner;
    relayConfig_t config = makeConfig();
    uint32_t nowMs = settle(tuner, config);

    (void)tuner.sample(SETPOINT - 300, BASE, nowMs += SAMPLE_MS);
    TEST_ASSERT_EQUAL(TunePhase::Relay, tuner.getPhase());

    TEST_ASSERT_EQUAL(BASE, tuner.sample(SETPOINT + 301, BASE, nowMs += SAMPLE_MS));
    TEST_ASSERT_EQUAL(TunePhase::Failed, tuner.getPhase());
    TEST_ASSERT_EQUAL(TuneFail::AmplitudeTooLarge, tuner.getFailure());
}

static void test_relay_fails_on_small_amplitude(void)
{
    // The oscillation (+/-40) is less than 1.5 times the band (+/-30)
    RelayTuner tuner;
    relayConfig_t config = makeConfig();
    config.hysteresis = 30U;
    uint32_t nowMs = settle(tuner, config);

    runTriangle(tuner, nowMs, 60000UL, 0);

    TEST_ASSERT_EQUAL(TunePhase::Failed, tuner.getPhase());
    TEST_ASSERT_EQUAL(TuneFail::AmplitudeTooSmall, tuner.getFailure());
}

static void test_relay_fails_without_convergence(void)
{
    // Square wave whose period alternates between 2S and 3S. Each cycle starts low, so the low to
    // high switches (where cycles are measured) are 2S and 3S apart.
    RelayTuner tuner;
    relayConfig_t config = makeConfig();
    config.timeoutMs = 40000UL;
    uint32_t nowMs = settle(tuner, config);

    bool isLongCycle = false;
    while (tuner.isRunning())
    {
        const uint32_t halfPeriodMs = isLongCycle ? 1500U : 1000U;
        for (uint32_t elapsed = 0U; tuner.isRunning() && (elapsed < (2U * halfPeriodMs)); elapsed += SAMPLE_MS)
        {
            nowMs += SAMPLE_MS;
            (void)tuner.sample((elapsed < halfPeriodMs) ? (SETPOINT - 20) : (SETPOINT + 20), BASE, nowMs);
        }
        isLongCycle = !isLongCycle;
    }

    TEST_ASSERT_EQUAL(TunePhase::Failed, tuner.getPhase());
    TEST_ASSERT_EQUAL(TuneFail::NoConvergence, tuner.getFailure());
    TEST_ASSERT_GREATER_OR_EQUAL(config.timeoutMs, nowMs);
}

static void test_relay_abort_and_reset(void)
{
    RelayTuner tuner;
    relayConfig_t config = makeConfig();
    (void)settle(tuner, config);

    tuner.abort(TuneFail::Aborted);
    TEST_ASSERT_EQUAL(TunePhase::Failed, tuner.getPhase());
    TEST_ASSERT_EQUAL(TuneFail::Aborted, tuner.getFailure());

    // Aborting a stopped tuner keeps the first failure
    tuner.abort(TuneFail::ConditionLost);
    TEST_ASSERT_EQUAL(TuneFail::Aborted, tuner.getFailure());

    tuner.reset();
    TEST_ASSERT_EQUAL(TunePhase::Idle, tuner.getPhase());
    TEST_ASSERT_EQUAL(TuneFail::None, tuner.getFailure());
}

void testRelayTuner(void)
{
  SET_UNITY_FILENAME() {
    RUN_TEST_P(test_relay_idle_until_started);
    RUN_TEST_P(test_relay_settle_holds_base_and_averages);
    RUN_TEST_P(test_relay_switches_outside_the_band);
    RUN_TEST_P(test_relay_switches_reverse_direction);
    RUN_TEST_P(test_relay_measures_period_and_amplitude);
    RUN_TEST_P(test_relay_discards_the_first_cycle);
    RUN_TEST_P(test_relay_measures_the_realized_amplitude);
    RUN_TEST_P(test_relay_cycles_are_clamped);
    RUN_TEST_P(test_relay_fails_without_switching);
    RUN_TEST_P(test_relay_fails_on_large_excursion);
    RUN_TEST_P(test_relay_fails_on_small_amplitude);
    RUN_TEST_P(test_relay_fails_without_convergence);
    RUN_TEST_P(test_relay_abort_and_reset);
  }
}
