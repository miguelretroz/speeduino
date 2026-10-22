#include <unity.h>
#include "../test_utils.h"
#include "protection_fixture.h"

static protection_context_t setup_with_output(bool inverted)
{
    protection_context_t context;
    context.pins.pinProtectWarn = TEST_WARN_PIN;
    context.page15.protWarnPin = TEST_WARN_PIN;
    context.page15.protWarnInv = inverted ? 1U : 0U;
    initialiseProtection(context.page15, context.pins);
    return context;
}

static void test_output_off_at_start(void)
{
    auto context = setup_with_output(false);

    TEST_ASSERT_TRUE(protectionState.warnPin.isValid());
    TEST_ASSERT_FALSE(context.isWarnPinHigh());
    TEST_ASSERT_FALSE(protectionState.warnOutputOn);
}

static void test_output_follows_a_warning(void)
{
    auto context = setup_with_output(false);
    context.page15.protCoolantHotEnable = 1U;
    context.current.coolant = 120;

    context.tickAndResolve();
    context.tick();
    TEST_ASSERT_TRUE(context.isWarnPinHigh());
    TEST_ASSERT_TRUE(protectionState.warnOutputOn);

    context.current.coolant = 80;
    context.tickAndResolve();
    context.tick();
    TEST_ASSERT_FALSE(context.isWarnPinHigh());
}

// The output shows upstream's conditions too, whatever their action
static void test_output_follows_an_upstream_condition(void)
{
    auto context = setup_with_output(false);
    context.setUpstream(true, false, false);

    context.resolve();
    context.tick();
    TEST_ASSERT_TRUE(context.isWarnPinHigh());

    context.setUpstream(false, false, false);
    context.resolve();
    context.tick();
    TEST_ASSERT_FALSE(context.isWarnPinHigh());
}

static void test_output_stays_on_with_memory(void)
{
    auto context = setup_with_output(false);
    context.page15.protLatch = 1U << PROTECT_BOOST;
    context.setUpstream(false, true, false);
    context.resolve();
    context.setUpstream(false, false, false);

    context.tickAndResolve();
    context.tick();
    TEST_ASSERT_TRUE(context.isWarnPinHigh());

    context.current.rotationStatus = EngineRotationStatus::Stopped;
    context.tick();
    TEST_ASSERT_FALSE(context.isWarnPinHigh());
}

// A faulted sensor lights the output even when it isn't a protection
static void test_output_follows_a_sensor_fault(void)
{
    auto context = setup_with_output(false);
    context.current.sensorFaults = 1U << SENSOR_FAULT_IAT;

    context.tick();
    TEST_ASSERT_TRUE(context.isWarnPinHigh());
    TEST_ASSERT_EQUAL_HEX8(0U, getProtectionLogReason());

    context.current.sensorFaults = 0U;
    context.tick();
    TEST_ASSERT_FALSE(context.isWarnPinHigh());
}

static void test_output_inverted(void)
{
    auto context = setup_with_output(true);
    TEST_ASSERT_TRUE(context.isWarnPinHigh());

    context.current.sensorFaults = 1U << SENSOR_FAULT_IAT;
    context.tick();
    TEST_ASSERT_FALSE(context.isWarnPinHigh());
    TEST_ASSERT_TRUE(protectionState.warnOutputOn);
}

static void test_no_output_pin(void)
{
    protection_context_t context;
    context.current.sensorFaults = 1U << SENSOR_FAULT_IAT;

    context.tick();

    TEST_ASSERT_FALSE(protectionState.warnPin.isValid());
    TEST_ASSERT_TRUE(protectionState.warnOutputOn); // Still in the live data
}

void testProtectionOutput(void)
{
    SET_UNITY_FILENAME() {
        RUN_TEST_P(test_output_off_at_start);
        RUN_TEST_P(test_output_follows_a_warning);
        RUN_TEST_P(test_output_follows_an_upstream_condition);
        RUN_TEST_P(test_output_stays_on_with_memory);
        RUN_TEST_P(test_output_follows_a_sensor_fault);
        RUN_TEST_P(test_output_inverted);
        RUN_TEST_P(test_no_output_pin);
    }
}
