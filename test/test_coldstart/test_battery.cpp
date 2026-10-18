#include "../test_utils.h"
#include "coldstart_fixture.h"

// Defaults: cut below 8.0V, back on from 8.5V. Pulses 300mS per 1S.

static void test_low_battery_cuts_cranking(void)
{
    auto context = setup_coldstart();
    context.keyOn();
    context.crank();
    TEST_ASSERT_EQUAL(300U, context.runFor(1000U));

    context.current.battery10 = 75U;
    TEST_ASSERT_EQUAL(0U, context.runFor(1000U));
    TEST_ASSERT_TRUE(coldStartState.isBatteryLow);

    context.current.battery10 = 82U; // Inside the hysteresis band: stays cut
    TEST_ASSERT_EQUAL(0U, context.runFor(1000U));

    context.current.battery10 = 85U;
    TEST_ASSERT_EQUAL(300U, context.runFor(1000U));
    TEST_ASSERT_FALSE(coldStartState.isBatteryLow);
}

static void test_low_battery_no_attempt_counted(void)
{
    auto context = setup_coldstart();
    context.keyOn();
    context.run();

    context.current.battery10 = 75U; // Sags as soon as the starter engages
    context.crank();
    TEST_ASSERT_EQUAL(0U, context.runFor(2000U));
    TEST_ASSERT_EQUAL(0U, fakeStorage.attempts);

    context.current.battery10 = 90U; // Recovers while still cranking
    TEST_ASSERT_EQUAL(300U, context.runFor(1000U));
    TEST_ASSERT_EQUAL(1U, fakeStorage.attempts);
}

static void test_low_battery_cuts_prime(void)
{
    auto context = setup_coldstart();
    fill_curve(context.page15.coldStartPrimeValues, 20U); // 2S

    context.keyOn();
    TEST_ASSERT_EQUAL(500U, context.runFor(500U));
    context.current.battery10 = 75U;
    TEST_ASSERT_EQUAL(0U, context.runFor(500U));
    context.current.battery10 = BATTERY_KEY_ON;
    TEST_ASSERT_EQUAL(1000U, context.runFor(3000U)); // The prime time keeps running while cut
}

static void test_low_battery_cut_disabled(void)
{
    auto context = setup_coldstart();
    context.page15.coldStartCutBattery = 0U;
    context.keyOn();
    context.run();

    context.current.battery10 = 60U; // Still above the key off level
    context.crank();
    TEST_ASSERT_EQUAL(300U, context.runFor(1000U));
}

static void test_usb_power_is_not_low_battery(void)
{
    auto context = setup_coldstart();
    context.keyOff(); // Reads close to 0V
    (void)context.runFor(1000U);
    TEST_ASSERT_FALSE(coldStartState.isBatteryLow);
}

void testColdStartBattery(void)
{
    SET_UNITY_FILENAME() {
        RUN_TEST_P(test_usb_power_is_not_low_battery);
        RUN_TEST_P(test_low_battery_cuts_cranking);
        RUN_TEST_P(test_low_battery_no_attempt_counted);
        RUN_TEST_P(test_low_battery_cuts_prime);
        RUN_TEST_P(test_low_battery_cut_disabled);
    }
}
