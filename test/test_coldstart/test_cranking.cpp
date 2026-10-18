#include "../test_utils.h"
#include "coldstart_fixture.h"

// Defaults: 300mS on per 1S period up to 30C, 5S max on time per attempt

static void test_cranking_pulses(void)
{
    auto context = setup_coldstart();

    context.keyOn();
    context.crank();
    TEST_ASSERT_EQUAL(300U, context.runFor(300U));
    TEST_ASSERT_EQUAL(0U, context.runFor(700U));
    TEST_ASSERT_EQUAL(300U, context.runFor(1000U));
    TEST_ASSERT_EQUAL(1U, fakeStorage.attempts);
}

static void test_cranking_min_pulse(void)
{
    auto context = setup_coldstart();
    fill_curve(context.page15.coldStartCrankValues, 5U); // 50mS

    context.keyOn();
    context.crank();
    TEST_ASSERT_EQUAL(200U, context.runFor(1000U)); // Lengthened to the 200mS minimum
}

static void test_cranking_no_period_stays_on(void)
{
    auto context = setup_coldstart();
    context.page15.coldStartPulsePeriod = 0U;

    context.keyOn();
    context.crank();
    TEST_ASSERT_EQUAL(2000U, context.runFor(2000U));
}

static void test_cranking_warm_engine(void)
{
    auto context = setup_coldstart();

    context.current.coolant = 45; // Above the 40C point, where the curve is 0
    context.keyOn();
    context.crank();
    TEST_ASSERT_EQUAL(0U, context.runFor(3000U));
    TEST_ASSERT_EQUAL(0U, fakeStorage.attempts);
}

static void test_cranking_max_on_time(void)
{
    auto context = setup_coldstart();
    context.page15.coldStartCrankMaxTime = 10U; // 1S

    context.keyOn();
    context.crank();
    TEST_ASSERT_EQUAL(1000U, context.runFor(10000U));
}

static void test_cranking_flood_clear(void)
{
    auto context = setup_coldstart();

    context.keyOn();
    context.crank();
    context.current.TPS = context.page4.floodClear;
    TEST_ASSERT_EQUAL(0U, context.runFor(3000U));

    context.current.TPS = 0U;
    TEST_ASSERT_EQUAL(300U, context.runFor(1000U));
}

static void test_cranking_after_prime_same_attempt(void)
{
    auto context = setup_coldstart();
    fill_curve(context.page15.coldStartPrimeValues, 10U); // 1S

    context.keyOn();
    (void)context.runFor(2000U);
    context.crank();
    TEST_ASSERT_EQUAL(300U, context.runFor(1000U));
    TEST_ASSERT_EQUAL(1U, fakeStorage.attempts);
}

static void test_cranking_interrupts_prime(void)
{
    auto context = setup_coldstart();
    fill_curve(context.page15.coldStartPrimeValues, 50U); // 5S

    context.keyOn();
    (void)context.runFor(500U);
    context.crank();
    TEST_ASSERT_EQUAL(300U, context.runFor(1000U)); // Now pulsing
}

static void test_engine_start_stops_cranking_output(void)
{
    auto context = setup_coldstart();

    context.keyOn();
    context.crank();
    (void)context.runFor(100U);
    context.engineRunning();
    TEST_ASSERT_EQUAL(0U, context.runFor(3000U)); // After start is off by default
}

static void test_after_start_pulses(void)
{
    auto context = setup_coldstart();
    fill_curve(context.page15.coldStartAfterValues, 20U); // 2S

    context.keyOn();
    context.crank();
    (void)context.runFor(1000U);
    context.engineRunning();
    TEST_ASSERT_EQUAL(600U, context.runFor(2000U));
    TEST_ASSERT_EQUAL(0U, context.runFor(3000U));
}

static void test_after_start_back_to_cranking(void)
{
    auto context = setup_coldstart();
    fill_curve(context.page15.coldStartAfterValues, 20U);

    context.keyOn();
    context.crank();
    (void)context.runFor(1000U);
    context.engineRunning();
    (void)context.runFor(500U);
    context.crank(); // Stumbles below the cranking RPM
    TEST_ASSERT_EQUAL(300U, context.runFor(1000U));
    TEST_ASSERT_EQUAL(1U, fakeStorage.attempts); // Same attempt
}

void testColdStartCranking(void)
{
    SET_UNITY_FILENAME() {
        RUN_TEST_P(test_cranking_pulses);
        RUN_TEST_P(test_cranking_min_pulse);
        RUN_TEST_P(test_cranking_no_period_stays_on);
        RUN_TEST_P(test_cranking_warm_engine);
        RUN_TEST_P(test_cranking_max_on_time);
        RUN_TEST_P(test_cranking_flood_clear);
        RUN_TEST_P(test_cranking_after_prime_same_attempt);
        RUN_TEST_P(test_cranking_interrupts_prime);
        RUN_TEST_P(test_engine_start_stops_cranking_output);
        RUN_TEST_P(test_after_start_pulses);
        RUN_TEST_P(test_after_start_back_to_cranking);
    }
}
