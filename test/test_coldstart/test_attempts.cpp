#include "../test_utils.h"
#include "coldstart_fixture.h"

// Defaults: doses 100%, 60%, 30%, 0%. Cleared after 10S running, or 3S of throttle above 90% (stopped, key on)

static coldstart_context_t setup_attempts(void)
{
    auto context = setup_coldstart();
    context.page15.coldStartCrankMaxTime = 10U; // 1S: makes the dose visible in the cranking on time
    context.keyOn();
    return context;
}

static uint32_t crankAndMeasure(coldstart_context_t &context)
{
    context.crank();
    uint32_t onTime = context.runFor(5000U);
    context.engineStopped();
    (void)context.runFor(100U);
    return onTime;
}

static void test_dose_drops_each_attempt(void)
{
    auto context = setup_attempts();

    TEST_ASSERT_EQUAL(1000U, crankAndMeasure(context));
    TEST_ASSERT_EQUAL(600U, crankAndMeasure(context));
    TEST_ASSERT_EQUAL(300U, crankAndMeasure(context));
    TEST_ASSERT_EQUAL(3U, fakeForkStorage.attempts);
}

static void test_locked_after_last_dose(void)
{
    auto context = setup_attempts();

    context.failedCrankAttempt();
    context.failedCrankAttempt();
    TEST_ASSERT_FALSE(context.current.coldStartLocked);
    context.failedCrankAttempt();
    TEST_ASSERT_TRUE(context.current.coldStartLocked);

    TEST_ASSERT_EQUAL(0U, crankAndMeasure(context));
    TEST_ASSERT_EQUAL(0U, crankAndMeasure(context));
    TEST_ASSERT_EQUAL(3U, fakeForkStorage.attempts);
}

static void test_attempts_survive_power_cycle(void)
{
    auto context = setup_attempts();
    context.failedCrankAttempt();
    context.failedCrankAttempt();

    // Key off: the board loses power
    context.keyOff();
    context.initialise();
    context.keyOn();

    TEST_ASSERT_EQUAL(300U, crankAndMeasure(context)); // 3rd attempt
    TEST_ASSERT_EQUAL(3U, fakeForkStorage.attempts);
}

static void test_corrupt_count_is_locked(void)
{
    auto context = setup_attempts();
    fakeForkStorage.attempts = 200U;
    context.initialise();

    context.run();
    TEST_ASSERT_TRUE(context.current.coldStartLocked);
    TEST_ASSERT_EQUAL(0U, crankAndMeasure(context));
}

static void test_running_clears_counter(void)
{
    auto context = setup_attempts();
    context.failedCrankAttempt();
    context.failedCrankAttempt();

    context.crank();
    (void)context.runFor(1000U);
    context.engineRunning();
    (void)context.runFor(9000U);
    TEST_ASSERT_EQUAL(3U, fakeForkStorage.attempts); // Not running for long enough yet
    (void)context.runFor(1100U);
    TEST_ASSERT_EQUAL(0U, fakeForkStorage.attempts);
    TEST_ASSERT_FALSE(context.current.coldStartLocked);
}

static void test_short_run_keeps_counter(void)
{
    auto context = setup_attempts();
    context.failedCrankAttempt();

    context.crank();
    (void)context.runFor(1000U);
    context.engineRunning();
    (void)context.runFor(5000U);
    context.engineStopped(); // Stalled
    (void)context.runFor(100U);

    TEST_ASSERT_EQUAL(2U, fakeForkStorage.attempts);
    TEST_ASSERT_EQUAL(300U, crankAndMeasure(context)); // 3rd attempt
}

static void test_reset_gesture(void)
{
    auto context = setup_attempts();
    context.failedCrankAttempt();
    context.failedCrankAttempt();
    context.failedCrankAttempt();

    context.current.TPS = 180U; // 90%
    (void)context.runFor(2900U);
    TEST_ASSERT_EQUAL(3U, fakeForkStorage.attempts);
    (void)context.runFor(200U);
    TEST_ASSERT_EQUAL(0U, fakeForkStorage.attempts);

    context.current.TPS = 0U;
    TEST_ASSERT_EQUAL(1000U, crankAndMeasure(context)); // Full dose again
}

static void test_reset_gesture_released_early(void)
{
    auto context = setup_attempts();
    context.failedCrankAttempt();

    context.current.TPS = 180U;
    (void)context.runFor(2000U);
    context.current.TPS = 0U;
    (void)context.runFor(100U);
    context.current.TPS = 180U;
    (void)context.runFor(2000U);

    TEST_ASSERT_EQUAL(1U, fakeForkStorage.attempts);
}

static void test_reset_gesture_disabled(void)
{
    auto context = setup_attempts();
    context.page15.coldStartResetTps = 0U;
    context.failedCrankAttempt();

    (void)context.runFor(5000U); // TPS is 0, which would otherwise satisfy a 0 threshold
    TEST_ASSERT_EQUAL(1U, fakeForkStorage.attempts);
}

static void test_reset_gesture_needs_key_on(void)
{
    auto context = setup_attempts();
    context.failedCrankAttempt();

    context.keyOff(); // USB powered
    context.current.TPS = 180U;
    (void)context.runFor(5000U);
    TEST_ASSERT_EQUAL(1U, fakeForkStorage.attempts);
}

// Default: 10 minutes with the key on and the engine stopped
static void test_key_on_wait_clears_counter(void)
{
    auto context = setup_attempts();
    context.failedCrankAttempt();
    context.failedCrankAttempt();

    context.runForMinutes(9U);
    TEST_ASSERT_EQUAL(2U, fakeForkStorage.attempts);
    context.runForMinutes(2U);
    TEST_ASSERT_EQUAL(0U, fakeForkStorage.attempts);
    TEST_ASSERT_EQUAL(1000U, crankAndMeasure(context)); // Full dose again
}

static void test_key_on_wait_restarts_after_cranking(void)
{
    auto context = setup_attempts();
    context.failedCrankAttempt();

    context.runForMinutes(6U);
    context.failedCrankAttempt(); // Restarts the wait
    context.runForMinutes(6U);
    TEST_ASSERT_EQUAL(2U, fakeForkStorage.attempts);
    context.runForMinutes(5U);
    TEST_ASSERT_EQUAL(0U, fakeForkStorage.attempts);
}

static void test_key_on_wait_disabled(void)
{
    auto context = setup_attempts();
    context.page15.coldStartKeyOnReset = 0U;
    context.failedCrankAttempt();

    context.runForMinutes(30U);
    TEST_ASSERT_EQUAL(1U, fakeForkStorage.attempts);
}

static void test_key_on_wait_needs_key_on(void)
{
    auto context = setup_attempts();
    context.failedCrankAttempt();

    context.keyOff(); // USB powered
    context.runForMinutes(30U);
    TEST_ASSERT_EQUAL(1U, fakeForkStorage.attempts);
}

static void test_eeprom_written_only_on_change(void)
{
    auto context = setup_attempts();

    context.failedCrankAttempt();
    context.failedCrankAttempt();
    context.failedCrankAttempt();
    context.failedCrankAttempt(); // Locked: not counted
    TEST_ASSERT_EQUAL(3U, fakeForkStorage.attemptWrites);

    context.crank();
    (void)context.runFor(1000U);
    context.engineRunning();
    (void)context.runFor(30000U);
    TEST_ASSERT_EQUAL(4U, fakeForkStorage.attemptWrites); // Cleared once
}

void testColdStartAttempts(void)
{
    SET_UNITY_FILENAME() {
        RUN_TEST_P(test_dose_drops_each_attempt);
        RUN_TEST_P(test_locked_after_last_dose);
        RUN_TEST_P(test_attempts_survive_power_cycle);
        RUN_TEST_P(test_corrupt_count_is_locked);
        RUN_TEST_P(test_running_clears_counter);
        RUN_TEST_P(test_short_run_keeps_counter);
        RUN_TEST_P(test_reset_gesture);
        RUN_TEST_P(test_reset_gesture_released_early);
        RUN_TEST_P(test_reset_gesture_disabled);
        RUN_TEST_P(test_reset_gesture_needs_key_on);
        RUN_TEST_P(test_key_on_wait_clears_counter);
        RUN_TEST_P(test_key_on_wait_restarts_after_cranking);
        RUN_TEST_P(test_key_on_wait_disabled);
        RUN_TEST_P(test_key_on_wait_needs_key_on);
        RUN_TEST_P(test_eeprom_written_only_on_change);
    }
}
