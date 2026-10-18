#include "../test_utils.h"
#include "coldstart_fixture.h"

static coldstart_context_t setup_prime(void)
{
    auto context = setup_coldstart();
    fill_curve(context.page15.coldStartPrimeValues, 16U); // 1.6S
    return context;
}

static void test_prime_off_by_default(void)
{
    auto context = setup_coldstart();

    context.keyOn();
    TEST_ASSERT_EQUAL(0U, context.runFor(5000U));
    TEST_ASSERT_EQUAL(0U, fakeStorage.attempts);
}

static void test_usb_power_never_primes(void)
{
    auto context = setup_prime();

    context.current.battery10 = 50U; // USB only: below 5.5V
    TEST_ASSERT_EQUAL(0U, context.runFor(5000U));
    TEST_ASSERT_EQUAL(0U, fakeStorage.attempts);
}

static void test_usb_power_never_primes_low_threshold(void)
{
    auto context = setup_prime();
    context.page15.coldStartMinBattery = 0U; // A bad tune must not make USB power inject

    context.current.battery10 = 50U;
    TEST_ASSERT_EQUAL(0U, context.runFor(5000U));
}

static void test_prime_on_key_on(void)
{
    auto context = setup_prime();

    context.keyOn();
    context.run();
    TEST_ASSERT_TRUE(context.isOutputOn());
    TEST_ASSERT_TRUE(context.current.coldStartActive);
    TEST_ASSERT_EQUAL(1U, fakeStorage.attempts); // Counted before injecting

    TEST_ASSERT_EQUAL(1600U, context.runFor(5000U));
    TEST_ASSERT_FALSE(context.isOutputOn());
    TEST_ASSERT_FALSE(context.current.coldStartActive);
}

static void test_prime_once_per_key_on(void)
{
    auto context = setup_prime();

    context.keyOn();
    (void)context.runFor(5000U);
    TEST_ASSERT_EQUAL(0U, context.runFor(5000U));
    TEST_ASSERT_EQUAL(1U, fakeStorage.attempts);
}

static void test_prime_again_after_key_cycle(void)
{
    auto context = setup_prime();

    context.keyOn();
    (void)context.runFor(5000U);
    context.keyOff();
    (void)context.runFor(1000U);
    context.keyOn();

    TEST_ASSERT_EQUAL(960U, context.runFor(5000U)); // 2nd attempt: 60% of 1.6S
    TEST_ASSERT_EQUAL(2U, fakeStorage.attempts);
}

static void test_prime_battery_dip_does_not_rearm(void)
{
    auto context = setup_prime();

    context.keyOn();
    (void)context.runFor(5000U);
    context.current.battery10 = 80U; // Sags, but stays above the key off level
    (void)context.runFor(1000U);
    context.keyOn();

    TEST_ASSERT_EQUAL(0U, context.runFor(5000U));
    TEST_ASSERT_EQUAL(1U, fakeStorage.attempts);
}

static void test_prime_key_off_stops_output(void)
{
    auto context = setup_prime();

    context.keyOn();
    (void)context.runFor(500U);
    TEST_ASSERT_TRUE(context.isOutputOn());

    context.keyOff();
    context.run();
    TEST_ASSERT_FALSE(context.isOutputOn());
}

static void test_prime_follows_coolant_curve(void)
{
    auto context = setup_prime();
    const uint8_t values[] = { 30U, 25U, 20U, 16U, 10U, 0U }; // 0, 10, 20, 30, 35 & 40C
    for (uint8_t i = 0U; i < 6U; ++i) { context.page15.coldStartPrimeValues[i] = values[i]; }

    context.current.coolant = 10;
    context.keyOn();
    TEST_ASSERT_EQUAL(2500U, context.runFor(5000U));
}

static void test_prime_warm_engine(void)
{
    auto context = setup_prime();
    context.page15.coldStartPrimeValues[5] = 0U; // Nothing from 40C

    context.current.coolant = 45;
    context.keyOn();
    TEST_ASSERT_EQUAL(0U, context.runFor(5000U));
    TEST_ASSERT_EQUAL(0U, fakeStorage.attempts);
}

static void test_prime_coolant_sensor_fault(void)
{
    auto context = setup_prime();

    context.current.coolant = -40; // Disconnected sensor
    context.keyOn();
    TEST_ASSERT_EQUAL(0U, context.runFor(5000U));
    TEST_ASSERT_EQUAL(0U, fakeStorage.attempts);
}

static void test_prime_throttle_held_open(void)
{
    auto context = setup_prime();

    context.current.TPS = context.page4.floodClear;
    context.keyOn();
    TEST_ASSERT_EQUAL(0U, context.runFor(5000U));
}

void testColdStartPrime(void)
{
    SET_UNITY_FILENAME() {
        RUN_TEST_P(test_prime_off_by_default);
        RUN_TEST_P(test_usb_power_never_primes);
        RUN_TEST_P(test_usb_power_never_primes_low_threshold);
        RUN_TEST_P(test_prime_on_key_on);
        RUN_TEST_P(test_prime_once_per_key_on);
        RUN_TEST_P(test_prime_again_after_key_cycle);
        RUN_TEST_P(test_prime_battery_dip_does_not_rearm);
        RUN_TEST_P(test_prime_key_off_stops_output);
        RUN_TEST_P(test_prime_follows_coolant_curve);
        RUN_TEST_P(test_prime_warm_engine);
        RUN_TEST_P(test_prime_coolant_sensor_fault);
        RUN_TEST_P(test_prime_throttle_held_open);
    }
}
