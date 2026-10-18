#include "../test_utils.h"
#include "coldstart_fixture.h"
#include "logger.h"

// Live data byte 139: bits 0-1 phase, bits 2-4 attempts

static uint8_t logStatus(uint8_t phase, uint8_t attempts)
{
    return (uint8_t)(phase | (attempts << 2U));
}

static void assert_log_status(uint8_t phase, uint8_t attempts)
{
    TEST_ASSERT_EQUAL_HEX8(logStatus(phase, attempts), getColdStartLogStatus());
    TEST_ASSERT_EQUAL_HEX8(logStatus(phase, attempts), getTSLogEntry(139U));
}

static void test_log_idle(void)
{
    auto context = setup_coldstart();
    context.run();
    assert_log_status(0U, 0U);
}

static void test_log_prime(void)
{
    auto context = setup_coldstart();
    fill_curve(context.page15.coldStartPrimeValues, 16U);

    context.keyOn();
    context.run();
    assert_log_status(1U, 1U);
}

static void test_log_cranking_and_after_start(void)
{
    auto context = setup_coldstart();
    fill_curve(context.page15.coldStartAfterValues, 20U);
    context.keyOn();
    context.failedCrankAttempt();

    context.crank();
    context.run();
    assert_log_status(2U, 2U);

    context.engineRunning();
    context.run();
    assert_log_status(3U, 2U);
}

static void test_log_locked(void)
{
    auto context = setup_coldstart();
    context.keyOn();
    context.failedCrankAttempt();
    context.failedCrankAttempt();
    context.failedCrankAttempt();

    context.crank();
    context.run();
    assert_log_status(0U, 3U); // No dose left: stays idle while cranking
}

static void test_log_low_battery(void)
{
    auto context = setup_coldstart();
    context.keyOn();
    context.run();
    context.current.battery10 = 75U;
    context.run();

    constexpr uint8_t LOW_BATTERY_BIT = 1U << 5U;
    TEST_ASSERT_EQUAL_HEX8(LOW_BATTERY_BIT, getColdStartLogStatus());
    TEST_ASSERT_EQUAL_HEX8(LOW_BATTERY_BIT, getTSLogEntry(139U));
}

void testColdStartLog(void)
{
    SET_UNITY_FILENAME() {
        RUN_TEST_P(test_log_idle);
        RUN_TEST_P(test_log_prime);
        RUN_TEST_P(test_log_cranking_and_after_start);
        RUN_TEST_P(test_log_locked);
        RUN_TEST_P(test_log_low_battery);
    }
}
