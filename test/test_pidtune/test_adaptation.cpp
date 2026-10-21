#include <unity.h>
#include "../test_utils.h"
#include "pidtune_fixture.h"
#include "logger.h"

using namespace pidTune;
using namespace pidTuneController::details;

static constexpr uint32_t SAMPLE_MS = 250U;

/** @brief Idle closed loop with the adaptation on, initialised from the config */
static void setup_idle_adaptation(void)
{
    setup_idle_tune();
    configPage15.pidAdaptIdle = 1U;
    initialisePidTune(configPage6, configPage15);
}

/** @brief Feed the idle adaptation with an oscillating RPM error (+/-60, 2S period) for one window. Returns the time. */
static uint32_t oscillate_idle(uint32_t nowMs)
{
    const uint32_t windowMs = (uint32_t)configPage15.pidAdaptWindow * 1000UL;
    for (uint32_t elapsed = 0U; elapsed <= windowMs; elapsed += SAMPLE_MS)
    {
        const int16_t error = (((elapsed / 1000U) % 2U) == 0U) ? 60 : -60;
        pidTuneIdleAdaptSample(currentStatus, error, IDLE_BASE, nowMs + elapsed);
    }
    return nowMs + windowMs;
}

static void test_adapt_idle_gains(void)
{
    setup_idle_adaptation();
    pidTuneState.idleAdapt.adapter.setMultiplier(150U);

    PidTuningParameters gains = pidTuneIdleGains(configPage6);
    TEST_ASSERT_EQUAL(48, gains.Kp); // 32 * 150%
    TEST_ASSERT_EQUAL(12, gains.Ki); // 8 * 150%

    // Off: the config bytes, whatever the multiplier
    configPage15.pidAdaptIdle = 0U;
    gains = pidTuneIdleGains(configPage6);
    TEST_ASSERT_EQUAL(32, gains.Kp);
    TEST_ASSERT_EQUAL(8, gains.Ki);
}

static void test_adapt_idle_lowers_an_oscillating_loop(void)
{
    setup_idle_adaptation();

    (void)oscillate_idle(TEST_START_MS);

    TEST_ASSERT_EQUAL_UINT8(95U, pidTuneState.idleAdapt.adapter.getMultiplier());
    // The new gains are applied without a jump
    TEST_ASSERT_TRUE(pidTuneIdleTakeBumpless());
    TEST_ASSERT_EQUAL(30, pidTuneIdleGains(configPage6).Kp); // 32 * 95% = 30.4
    // Live data: the multiplier in use, and the last decision (lowered)
    run_control(TEST_START_MS + 20000UL);
    TEST_ASSERT_EQUAL_UINT8(95U, getTSLogEntry(154U));
    TEST_ASSERT_EQUAL_UINT8((uint8_t)AdaptDecision::Lower << 2U, getTSLogEntry(156U) & 0x1CU);
}

static void test_adapt_idle_needs_steady_conditions(void)
{
    setup_idle_adaptation();
    pidTuneIdleAdaptSample(currentStatus, 10, IDLE_BASE, TEST_START_MS);
    TEST_ASSERT_TRUE(pidTuneState.idleAdapt.adapter.isWindowRunning());
    TEST_ASSERT_BITS_HIGH(0x01U, getTSLogEntry(156U));

    // Throttle open
    currentStatus.TPS = configPage2.iacTPSlimit + 1U;
    pidTuneIdleAdaptSample(currentStatus, 10, IDLE_BASE, TEST_START_MS + SAMPLE_MS);
    TEST_ASSERT_FALSE(pidTuneState.idleAdapt.adapter.isWindowRunning());
    currentStatus.TPS = 0U;

    // Output at its limit: the loop can't do more, whatever the gains
    pidTuneIdleAdaptSample(currentStatus, 10, 300 * 4, TEST_START_MS + (2U * SAMPLE_MS));
    TEST_ASSERT_FALSE(pidTuneState.idleAdapt.adapter.isWindowRunning());

    // A load switching on
    pidTuneIdleAdaptSample(currentStatus, 10, IDLE_BASE, TEST_START_MS + (3U * SAMPLE_MS));
    TEST_ASSERT_TRUE(pidTuneState.idleAdapt.adapter.isWindowRunning());
    currentStatus.fanDuty = 100U;
    pidTuneIdleAdaptSample(currentStatus, 10, IDLE_BASE, TEST_START_MS + (4U * SAMPLE_MS));
    TEST_ASSERT_FALSE(pidTuneState.idleAdapt.adapter.isWindowRunning());
    // The fan staying on is steady again
    pidTuneIdleAdaptSample(currentStatus, 10, IDLE_BASE, TEST_START_MS + (5U * SAMPLE_MS));
    TEST_ASSERT_TRUE(pidTuneState.idleAdapt.adapter.isWindowRunning());

    // Off: nothing is measured
    configPage15.pidAdaptIdle = 0U;
    pidTuneState.idleAdapt.adapter.reset();
    pidTuneIdleAdaptSample(currentStatus, 10, IDLE_BASE, TEST_START_MS + (6U * SAMPLE_MS));
    TEST_ASSERT_FALSE(pidTuneState.idleAdapt.adapter.isWindowRunning());
}

static void test_adapt_ego(void)
{
    setup_ego_tune();
    configPage15.pidAdaptEgo = 1U;
    initialisePidTune(configPage6, configPage15);

    // AFR swinging +/-0.6 around the target, 2S period
    const uint32_t windowMs = (uint32_t)configPage15.pidAdaptWindow * 1000UL;
    for (uint32_t elapsed = 0U; elapsed <= windowMs; elapsed += 125U)
    {
        currentStatus.O2 = (((elapsed / 1000U) % 2U) == 0U) ? 153U : 141U;
        pidTuneEgoAdaptSample(currentStatus, configPage6, 100U, TEST_START_MS + elapsed);
    }
    TEST_ASSERT_EQUAL_UINT8(95U, pidTuneState.egoAdapt.adapter.getMultiplier());
    TEST_ASSERT_TRUE(pidTuneEgoTakeBumpless());
    TEST_ASSERT_EQUAL(-((35 * 95 + 50) / 100), (pidTuneEgoGains(configPage6) * -1).Kp);

    // At the authority limit: not steady
    pidTuneEgoAdaptSample(currentStatus, configPage6, 100U + configPage6.egoLimit, TEST_START_MS + windowMs + 125U);
    TEST_ASSERT_FALSE(pidTuneState.egoAdapt.adapter.isWindowRunning());
    // Acceleration: not steady
    pidTuneEgoAdaptSample(currentStatus, configPage6, 100U, TEST_START_MS + windowMs + 250U);
    TEST_ASSERT_TRUE(pidTuneState.egoAdapt.adapter.isWindowRunning());
    currentStatus.isAcceleratingTPS = true;
    pidTuneEgoAdaptSample(currentStatus, configPage6, 100U, TEST_START_MS + windowMs + 375U);
    TEST_ASSERT_FALSE(pidTuneState.egoAdapt.adapter.isWindowRunning());
    currentStatus.isAcceleratingTPS = false;

    // Another operating point: the window starts again there
    pidTuneEgoAdaptSample(currentStatus, configPage6, 100U, TEST_START_MS + windowMs + 500U);
    TEST_ASSERT_TRUE(pidTuneState.egoAdapt.adapter.isWindowRunning());
    currentStatus.RPM = IDLE_RPM + 200U;
    pidTuneEgoAdaptSample(currentStatus, configPage6, 100U, TEST_START_MS + windowMs + 625U);
    TEST_ASSERT_FALSE(pidTuneState.egoAdapt.adapter.isWindowRunning());
    pidTuneEgoAdaptSample(currentStatus, configPage6, 100U, TEST_START_MS + windowMs + 750U);
    TEST_ASSERT_TRUE(pidTuneState.egoAdapt.adapter.isWindowRunning());
}

static void test_adapt_saved_once_stable(void)
{
    setup_idle_adaptation();
    currentStatus.rotationStatus = EngineRotationStatus::Running;
    run_control(TEST_START_MS);
    uint32_t nowMs = oscillate_idle(TEST_START_MS);
    TEST_ASSERT_EQUAL_UINT8(95U, pidTuneState.idleAdapt.adapter.getMultiplier());

    // Seen at the next 10Hz check, saved a minute later if it hasn't changed again
    run_control(nowMs);
    TEST_ASSERT_EQUAL_HEX8(0xFFU, fakeForkStorage.pidAdaptIdle);
    run_control(nowMs + ADAPT_SAVE_DELAY_MS - 1U);
    TEST_ASSERT_EQUAL_HEX8(0xFFU, fakeForkStorage.pidAdaptIdle);
    run_control(nowMs + ADAPT_SAVE_DELAY_MS);
    TEST_ASSERT_EQUAL_UINT8(95U, fakeForkStorage.pidAdaptIdle);
    TEST_ASSERT_EQUAL_UINT16(1U, fakeForkStorage.pidAdaptWrites);

    // Not written again while unchanged
    run_control(nowMs + (2U * ADAPT_SAVE_DELAY_MS));
    TEST_ASSERT_EQUAL_UINT16(1U, fakeForkStorage.pidAdaptWrites);
}

static void test_adapt_saved_when_the_engine_stops(void)
{
    setup_idle_adaptation();
    currentStatus.rotationStatus = EngineRotationStatus::Running;
    run_control(TEST_START_MS);
    uint32_t nowMs = oscillate_idle(TEST_START_MS);
    run_control(nowMs);
    TEST_ASSERT_EQUAL_HEX8(0xFFU, fakeForkStorage.pidAdaptIdle);

    currentStatus.rotationStatus = EngineRotationStatus::Stopped;
    run_control(nowMs + 100U);
    TEST_ASSERT_EQUAL_UINT8(95U, fakeForkStorage.pidAdaptIdle);
}

static void test_adapt_loaded_at_startup(void)
{
    setup_idle_tune();
    configPage15.pidAdaptIdle = 1U;
    configPage15.pidAdaptEgo = 1U;
    fakeForkStorage.pidAdaptIdle = 120U;
    fakeForkStorage.pidAdaptEgo = 0xFFU; // Never saved

    initialisePidTune(configPage6, configPage15);

    TEST_ASSERT_EQUAL_UINT8(120U, pidTuneState.idleAdapt.adapter.getMultiplier());
    TEST_ASSERT_EQUAL_UINT8(100U, pidTuneState.egoAdapt.adapter.getMultiplier());
    TEST_ASSERT_EQUAL_UINT8(120U, getTSLogEntry(154U));
    TEST_ASSERT_EQUAL_UINT8(100U, getTSLogEntry(155U));
    // Nothing to save
    run_control(TEST_START_MS + ADAPT_SAVE_DELAY_MS);
    TEST_ASSERT_EQUAL_UINT16(0U, fakeForkStorage.pidAdaptWrites);
}

static void test_adapt_reset_by_new_gains(void)
{
    setup_idle_tune();
    configPage15.pidAdaptIdle = 1U;
    fakeForkStorage.pidAdaptIdle = 120U;
    initialisePidTune(configPage6, configPage15);

    // The user changes P: the multiplier was for other gains
    configPage6.idleKP = 40U;
    run_control(TEST_START_MS);
    TEST_ASSERT_EQUAL_UINT8(100U, pidTuneState.idleAdapt.adapter.getMultiplier());
    TEST_ASSERT_TRUE(pidTuneIdleTakeBumpless());
    TEST_ASSERT_EQUAL(40, pidTuneIdleGains(configPage6).Kp);
}

static void test_adapt_reset_command(void)
{
    setup_idle_tune();
    configPage15.pidAdaptIdle = 1U;
    configPage15.pidAdaptEgo = 1U;
    fakeForkStorage.pidAdaptIdle = 120U;
    fakeForkStorage.pidAdaptEgo = 80U;
    initialisePidTune(configPage6, configPage15);

    TEST_ASSERT_TRUE(send_command(TS_CMD_PIDTUNE_ADAPT_RESET_IDLE, TEST_START_MS));
    TEST_ASSERT_EQUAL_UINT8(100U, pidTuneState.idleAdapt.adapter.getMultiplier());
    TEST_ASSERT_EQUAL_UINT8(100U, fakeForkStorage.pidAdaptIdle); // Saved straight away
    TEST_ASSERT_EQUAL_UINT8(80U, pidTuneState.egoAdapt.adapter.getMultiplier());

    TEST_ASSERT_TRUE(send_command(TS_CMD_PIDTUNE_ADAPT_RESET_EGO, TEST_START_MS));
    TEST_ASSERT_EQUAL_UINT8(100U, pidTuneState.egoAdapt.adapter.getMultiplier());
    TEST_ASSERT_EQUAL_UINT8(100U, fakeForkStorage.pidAdaptEgo);
}

static void test_adapt_reset_by_apply(void)
{
    run_idle_tune_to_done();
    configPage15.pidAdaptIdle = 1U;
    pidTuneState.idleAdapt.adapter.setMultiplier(130U);
    pidTuneState.egoAdapt.adapter.setMultiplier(80U);

    (void)send_command(TS_CMD_PIDTUNE_APPLY, TEST_START_MS);

    TEST_ASSERT_EQUAL_UINT8(100U, pidTuneState.idleAdapt.adapter.getMultiplier());
    TEST_ASSERT_EQUAL_UINT8(80U, pidTuneState.egoAdapt.adapter.getMultiplier());
    // Not seen as a change by the user
    run_control(TEST_START_MS);
    TEST_ASSERT_EQUAL_UINT8(10U, pidTuneState.idleAdapt.gains[0]);
}

static void test_adapt_turning_it_on_is_smooth(void)
{
    setup_idle_tune();
    fakeForkStorage.pidAdaptIdle = 120U;
    initialisePidTune(configPage6, configPage15);
    run_control(TEST_START_MS);
    TEST_ASSERT_FALSE(pidTuneIdleTakeBumpless());
    TEST_ASSERT_EQUAL_UINT8(100U, getTSLogEntry(154U));

    configPage15.pidAdaptIdle = 1U;
    run_control(TEST_START_MS + 100U);
    TEST_ASSERT_TRUE(pidTuneIdleTakeBumpless());
    TEST_ASSERT_EQUAL_UINT8(120U, getTSLogEntry(154U));
}

void testPidTuneAdaptation(void)
{
  SET_UNITY_FILENAME() {
    RUN_TEST_P(test_adapt_idle_gains);
    RUN_TEST_P(test_adapt_idle_lowers_an_oscillating_loop);
    RUN_TEST_P(test_adapt_idle_needs_steady_conditions);
    RUN_TEST_P(test_adapt_ego);
    RUN_TEST_P(test_adapt_saved_once_stable);
    RUN_TEST_P(test_adapt_saved_when_the_engine_stops);
    RUN_TEST_P(test_adapt_loaded_at_startup);
    RUN_TEST_P(test_adapt_reset_by_new_gains);
    RUN_TEST_P(test_adapt_reset_command);
    RUN_TEST_P(test_adapt_reset_by_apply);
    RUN_TEST_P(test_adapt_turning_it_on_is_smooth);
  }
}
