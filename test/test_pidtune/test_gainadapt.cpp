#include <unity.h>
#include <math.h>
#include "src/controllers/pidTune/gainAdapt.h"
#include "../test_utils.h"

using namespace pidTune;

static constexpr uint32_t SAMPLE_MS = 250U;
static constexpr uint32_t WINDOW_MS = 15000UL;

static gainAdaptConfig_t makeConfig(void)
{
    gainAdaptConfig_t config;
    config.band = 10U;
    config.windowMs = WINDOW_MS;
    config.stepPct = 5U;
    return config;
}

/** @brief Feed one window of samples given by a function of the sample index. Returns the time after it. */
template <typename errorFn_t>
static uint32_t runWindow(GainAdapter &adapter, uint32_t nowMs, errorFn_t errorAt)
{
    const gainAdaptConfig_t config = makeConfig();
    for (uint32_t elapsed = 0U; elapsed <= WINDOW_MS; elapsed += SAMPLE_MS)
    {
        (void)adapter.sample(errorAt(elapsed / SAMPLE_MS), true, nowMs + elapsed, config);
    }
    return nowMs + WINDOW_MS + SAMPLE_MS;
}

static int16_t oscillating(uint32_t index) { return ((index / 4U) % 2U == 0U) ? 30 : -30; } // 2S period
static int16_t offTarget(uint32_t index) { (void)index; return 20; }
static int16_t onTarget(uint32_t index) { return (index % 2U == 0U) ? 5 : -5; }
static int16_t oneCrossing(uint32_t index) { return (index < 30U) ? 20 : -20; }
static int16_t noise(uint32_t index) { return (index % 2U == 0U) ? 30 : -30; } // Past the band on every sample

static void test_adapt_starts_at_100(void)
{
    GainAdapter adapter;
    TEST_ASSERT_EQUAL_UINT8(100U, adapter.getMultiplier());
    TEST_ASSERT_EQUAL(AdaptDecision::None, adapter.getLastDecision());
    TEST_ASSERT_FALSE(adapter.isWindowRunning());
}

static void test_adapt_lowers_when_oscillating(void)
{
    GainAdapter adapter;
    (void)runWindow(adapter, 0U, oscillating);
    TEST_ASSERT_EQUAL_UINT8(95U, adapter.getMultiplier());
    TEST_ASSERT_EQUAL(AdaptDecision::Lower, adapter.getLastDecision());
}

static void test_adapt_raises_when_off_target(void)
{
    GainAdapter adapter;
    (void)runWindow(adapter, 0U, offTarget);
    TEST_ASSERT_EQUAL_UINT8(105U, adapter.getMultiplier());
    TEST_ASSERT_EQUAL(AdaptDecision::Raise, adapter.getLastDecision());
}

static void test_adapt_holds_on_target(void)
{
    // Noise inside the band never counts as a crossing
    GainAdapter adapter;
    (void)runWindow(adapter, 0U, onTarget);
    TEST_ASSERT_EQUAL_UINT8(100U, adapter.getMultiplier());
    TEST_ASSERT_EQUAL(AdaptDecision::Hold, adapter.getLastDecision());

    // One crossing: settling, not oscillating
    (void)runWindow(adapter, 20000UL, oneCrossing);
    TEST_ASSERT_EQUAL_UINT8(100U, adapter.getMultiplier());
    TEST_ASSERT_EQUAL(AdaptDecision::Hold, adapter.getLastDecision());
}

/** @brief Feed one window of a sine wave error. Returns the multiplier after it. */
static uint8_t runSine(int16_t amplitude, uint32_t periodMs, uint32_t sampleMs, uint16_t band)
{
    GainAdapter adapter;
    gainAdaptConfig_t config = makeConfig();
    config.band = band;
    for (uint32_t nowMs = 0U; nowMs <= WINDOW_MS; nowMs += sampleMs)
    {
        const float phase = 6.2831853f * (float)(nowMs % periodMs) / (float)periodMs;
        const int16_t error = (int16_t)floorf(((float)amplitude * sinf(phase)) + 0.5f);
        (void)adapter.sample(error, true, nowMs, config);
    }
    return adapter.getMultiplier();
}

// Hunting is a smooth wave, not a square one: it must still be seen with a period of a few samples
static void test_adapt_lowers_sine_hunting(void)
{
    // Idle: a sample every 250mS, 25 RPM band
    TEST_ASSERT_EQUAL_UINT8(95U, runSine(60, 1500U, 250U, 25U));
    TEST_ASSERT_EQUAL_UINT8(95U, runSine(60, 2000U, 250U, 25U));
    TEST_ASSERT_EQUAL_UINT8(95U, runSine(60, 2500U, 250U, 25U));
    TEST_ASSERT_EQUAL_UINT8(95U, runSine(40, 2000U, 250U, 25U));
    // AFR: a step every 125mS, 0.2 AFR band
    TEST_ASSERT_EQUAL_UINT8(95U, runSine(5, 1000U, 125U, 2U));
    TEST_ASSERT_EQUAL_UINT8(95U, runSine(3, 1500U, 125U, 2U));
    // Inside the band: kept
    TEST_ASSERT_EQUAL_UINT8(100U, runSine(20, 2000U, 250U, 25U));
}

// The loop stops (E.g. the AFR closed loop leaves its window): when it comes back, the window starts again
static void test_adapt_gap_restarts_the_window(void)
{
    GainAdapter adapter;
    const gainAdaptConfig_t config = makeConfig();
    uint32_t nowMs = 0U;
    for (; nowMs < 10000UL; nowMs += SAMPLE_MS) { (void)adapter.sample(offTarget(0U), true, nowMs, config); }

    // 3S without samples
    nowMs += 3000UL;
    const uint32_t restartMs = nowMs;
    for (; nowMs < (restartMs + WINDOW_MS); nowMs += SAMPLE_MS) { (void)adapter.sample(offTarget(0U), true, nowMs, config); }
    TEST_ASSERT_EQUAL_UINT8(100U, adapter.getMultiplier());
    (void)adapter.sample(offTarget(0U), true, nowMs, config);
    TEST_ASSERT_EQUAL_UINT8(105U, adapter.getMultiplier());
}

static void test_adapt_ignores_noise(void)
{
    // Fast noise is filtered out: it doesn't look like an oscillation
    GainAdapter adapter;
    (void)runWindow(adapter, 0U, noise);
    TEST_ASSERT_EQUAL_UINT8(100U, adapter.getMultiplier());
    TEST_ASSERT_EQUAL(AdaptDecision::Hold, adapter.getLastDecision());
}

static void test_adapt_reports_a_change(void)
{
    GainAdapter adapter;
    const gainAdaptConfig_t config = makeConfig();
    uint32_t nowMs = 0U;
    bool isChanged = false;
    uint8_t changes = 0U;
    for (; nowMs <= WINDOW_MS; nowMs += SAMPLE_MS)
    {
        isChanged = adapter.sample(offTarget(0U), true, nowMs, config);
        if (isChanged) { ++changes; }
    }
    // Only the sample that ends the window
    TEST_ASSERT_TRUE(isChanged);
    TEST_ASSERT_EQUAL_UINT8(1U, changes);
}

static void test_adapt_limits(void)
{
    GainAdapter adapter;
    uint32_t nowMs = 0U;
    for (uint8_t window = 0U; window < 15U; ++window) { nowMs = runWindow(adapter, nowMs, oscillating); }
    TEST_ASSERT_EQUAL_UINT8(GainAdapter::MIN_MULTIPLIER, adapter.getMultiplier());

    for (uint8_t window = 0U; window < 25U; ++window) { nowMs = runWindow(adapter, nowMs, offTarget); }
    TEST_ASSERT_EQUAL_UINT8(GainAdapter::MAX_MULTIPLIER, adapter.getMultiplier());
}

static void test_adapt_unsteady_restarts_the_window(void)
{
    GainAdapter adapter;
    const gainAdaptConfig_t config = makeConfig();
    uint32_t nowMs = 0U;
    for (; nowMs < 10000UL; nowMs += SAMPLE_MS) { (void)adapter.sample(offTarget(0U), true, nowMs, config); }
    TEST_ASSERT_TRUE(adapter.isWindowRunning());

    (void)adapter.sample(offTarget(0U), false, nowMs, config);
    TEST_ASSERT_FALSE(adapter.isWindowRunning());

    // 15S from the restart, not from the first window
    nowMs += SAMPLE_MS;
    const uint32_t restartMs = nowMs;
    for (; nowMs < (restartMs + WINDOW_MS); nowMs += SAMPLE_MS) { (void)adapter.sample(offTarget(0U), true, nowMs, config); }
    TEST_ASSERT_EQUAL_UINT8(100U, adapter.getMultiplier());
    (void)adapter.sample(offTarget(0U), true, nowMs, config);
    TEST_ASSERT_EQUAL_UINT8(105U, adapter.getMultiplier());
}

static void test_adapt_short_window_is_ignored(void)
{
    // A sample every 3S: too few to judge
    GainAdapter adapter;
    const gainAdaptConfig_t config = makeConfig();
    for (uint32_t nowMs = 0U; nowMs <= WINDOW_MS; nowMs += 3000U) { (void)adapter.sample(offTarget(0U), true, nowMs, config); }
    TEST_ASSERT_EQUAL_UINT8(100U, adapter.getMultiplier());
    TEST_ASSERT_EQUAL(AdaptDecision::None, adapter.getLastDecision());
}

static void test_adapt_set_and_reset(void)
{
    GainAdapter adapter;
    adapter.setMultiplier(120U);
    TEST_ASSERT_EQUAL_UINT8(120U, adapter.getMultiplier());
    // Erased or corrupt EEPROM
    adapter.setMultiplier(0xFFU);
    TEST_ASSERT_EQUAL_UINT8(100U, adapter.getMultiplier());
    adapter.setMultiplier(49U);
    TEST_ASSERT_EQUAL_UINT8(100U, adapter.getMultiplier());

    (void)runWindow(adapter, 0U, oscillating);
    adapter.reset();
    TEST_ASSERT_EQUAL_UINT8(100U, adapter.getMultiplier());
    TEST_ASSERT_EQUAL(AdaptDecision::None, adapter.getLastDecision());
}

static void test_adapt_apply(void)
{
    GainAdapter adapter;
    PidTuningParameters gains(255U, 3U, 1U);

    PidTuningParameters scaled = adapter.apply(gains);
    TEST_ASSERT_EQUAL(255, scaled.Kp);
    TEST_ASSERT_EQUAL(3, scaled.Ki);
    TEST_ASSERT_EQUAL(1, scaled.Kd);

    adapter.setMultiplier(150U);
    scaled = adapter.apply(gains);
    TEST_ASSERT_EQUAL(383, scaled.Kp); // Past a byte: the gains are 16 bit
    TEST_ASSERT_EQUAL(5, scaled.Ki);   // 4.5, rounded
    TEST_ASSERT_EQUAL(2, scaled.Kd);   // 1.5, rounded

    // A term that isn't 0 never becomes 0
    adapter.setMultiplier(50U);
    scaled = adapter.apply(PidTuningParameters(0U, 1U, 1U) * -1);
    TEST_ASSERT_EQUAL(0, scaled.Kp);
    TEST_ASSERT_EQUAL(-1, scaled.Ki);
    TEST_ASSERT_EQUAL(-1, scaled.Kd);
}

void testGainAdapter(void)
{
  SET_UNITY_FILENAME() {
    RUN_TEST_P(test_adapt_starts_at_100);
    RUN_TEST_P(test_adapt_lowers_when_oscillating);
    RUN_TEST_P(test_adapt_raises_when_off_target);
    RUN_TEST_P(test_adapt_holds_on_target);
    RUN_TEST_P(test_adapt_ignores_noise);
    RUN_TEST_P(test_adapt_lowers_sine_hunting);
    RUN_TEST_P(test_adapt_gap_restarts_the_window);
    RUN_TEST_P(test_adapt_reports_a_change);
    RUN_TEST_P(test_adapt_limits);
    RUN_TEST_P(test_adapt_unsteady_restarts_the_window);
    RUN_TEST_P(test_adapt_short_window_is_ignored);
    RUN_TEST_P(test_adapt_set_and_reset);
    RUN_TEST_P(test_adapt_apply);
  }
}
