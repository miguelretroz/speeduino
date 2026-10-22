#include <unity.h>
#include "../test_utils.h"
#include "protection_fixture.h"

extern uint8_t applyEngineProtectionRevLimit(uint8_t curLimit, const statuses &current, const config4 &page4);

static void test_nothing_active(void)
{
    protection_context_t context;

    context.tickAndResolve();

    TEST_ASSERT_FALSE(context.current.engineProtect.isActive());
    TEST_ASSERT_FALSE(context.current.engineProtect.forkLimit);
    TEST_ASSERT_FALSE(context.current.engineProtect.forkStop);
    TEST_ASSERT_EQUAL_UINT8(ACTION_NONE, protectionState.action);
    TEST_ASSERT_EQUAL_HEX8(0U, getProtectionLogReason());
    TEST_ASSERT_EQUAL_UINT8(UINT8_MAX, applyEngineProtectionRevLimit(UINT8_MAX, context.current, context.page4));
}

// The defaults keep upstream's behaviour: the flag stays and the limp home limit applies
static void test_upstream_limit_keeps_the_flag(void)
{
    protection_context_t context;
    context.setUpstream(true, false, false);

    context.resolve();

    TEST_ASSERT_TRUE(context.current.engineProtect.oil);
    TEST_ASSERT_FALSE(context.current.engineProtect.forkStop);
    TEST_ASSERT_EQUAL_UINT8(ACTION_LIMIT, protectionState.action);
    TEST_ASSERT_EQUAL_HEX8(1U << PROTECT_OIL_PRESSURE, getProtectionLogReason());
    TEST_ASSERT_EQUAL_UINT8(TEST_LIMP_RPM, applyEngineProtectionRevLimit(UINT8_MAX, context.current, context.page4));
}

// Warn only: the flag is cleared, so upstream cuts nothing, and the reason still shows
static void test_upstream_warn_clears_the_flag(void)
{
    protection_context_t context;
    context.page15.protActionBoost = (uint8_t)ProtectAction::WarnOnly;
    context.setUpstream(false, true, false);

    context.resolve();

    TEST_ASSERT_FALSE(context.current.engineProtect.boostCut);
    TEST_ASSERT_FALSE(context.current.engineProtect.isActive());
    TEST_ASSERT_EQUAL_UINT8(ACTION_WARN, protectionState.action);
    TEST_ASSERT_EQUAL_HEX8(1U << PROTECT_BOOST, getProtectionLogReason());
    TEST_ASSERT_EQUAL_UINT8(UINT8_MAX, applyEngineProtectionRevLimit(UINT8_MAX, context.current, context.page4));
}

static void test_upstream_stop_cuts_at_any_rpm(void)
{
    protection_context_t context;
    context.page15.protActionAfr = (uint8_t)ProtectAction::StopEngine;
    context.setUpstream(false, false, true);

    context.resolve();

    TEST_ASSERT_TRUE(context.current.engineProtect.afr);
    TEST_ASSERT_TRUE(context.current.engineProtect.forkStop);
    TEST_ASSERT_EQUAL_UINT8(ACTION_STOP, protectionState.action);
    TEST_ASSERT_EQUAL_UINT8(0U, applyEngineProtectionRevLimit(UINT8_MAX, context.current, context.page4));
}

// The fork's conditions have no upstream flag: they limit through their own flag
static void test_fork_condition_limits(void)
{
    protection_context_t context;
    context.page15.protCoolantHotEnable = 1U;
    context.page15.protActionCoolantHot = (uint8_t)ProtectAction::LimitRpm;
    context.current.coolant = 115;

    context.tickAndResolve();

    TEST_ASSERT_TRUE(context.current.engineProtect.forkLimit);
    TEST_ASSERT_FALSE(context.current.engineProtect.forkStop);
    TEST_ASSERT_TRUE(context.current.engineProtect.isActive());
    TEST_ASSERT_EQUAL_UINT8(ACTION_LIMIT, protectionState.action);
    TEST_ASSERT_EQUAL_HEX8(1U << PROTECT_COOLANT_HOT, getProtectionLogReason());
    TEST_ASSERT_EQUAL_UINT8(TEST_LIMP_RPM, applyEngineProtectionRevLimit(UINT8_MAX, context.current, context.page4));
    // A lower limit from elsewhere stays
    TEST_ASSERT_EQUAL_UINT8(20U, applyEngineProtectionRevLimit(20U, context.current, context.page4));
}

static void test_fork_condition_warns(void)
{
    protection_context_t context;
    context.page15.protCoolantHotEnable = 1U;
    context.current.coolant = 115;

    context.tickAndResolve(); // Default action for hot coolant: warn only

    TEST_ASSERT_FALSE(context.current.engineProtect.isActive());
    TEST_ASSERT_EQUAL_UINT8(ACTION_WARN, protectionState.action);
    TEST_ASSERT_EQUAL_HEX8(1U << PROTECT_COOLANT_HOT, getProtectionLogReason());
}

static void test_fork_condition_stops(void)
{
    protection_context_t context;
    context.page15.protCoolantHotEnable = 1U;
    context.page15.protActionCoolantHot = (uint8_t)ProtectAction::StopEngine;
    context.current.coolant = 115;

    context.tickAndResolve();

    TEST_ASSERT_TRUE(context.current.engineProtect.forkStop);
    TEST_ASSERT_EQUAL_UINT8(0U, applyEngineProtectionRevLimit(UINT8_MAX, context.current, context.page4));
}

// Stop beats limit beats warn
static void test_action_precedence(void)
{
    protection_context_t context;
    context.page15.protActionOilPress = (uint8_t)ProtectAction::WarnOnly;
    context.page15.protActionBoost = (uint8_t)ProtectAction::LimitRpm;
    context.page15.protActionAfr = (uint8_t)ProtectAction::StopEngine;

    context.setUpstream(true, true, false);
    context.resolve();
    TEST_ASSERT_EQUAL_UINT8(ACTION_LIMIT, protectionState.action);
    TEST_ASSERT_FALSE(context.current.engineProtect.oil);
    TEST_ASSERT_TRUE(context.current.engineProtect.boostCut);

    context.setUpstream(true, true, true);
    context.resolve();
    TEST_ASSERT_EQUAL_UINT8(ACTION_STOP, protectionState.action);
    TEST_ASSERT_EQUAL_HEX8(0x07U, getProtectionLogReason());
}

// With the protection cut off nothing is cut, as upstream, but the conditions still show
static void test_nothing_cut_with_protection_off(void)
{
    protection_context_t context;
    context.page6.engineProtectType = PROTECT_CUT_OFF;
    context.page15.protCoolantHotEnable = 1U;
    context.page15.protActionCoolantHot = (uint8_t)ProtectAction::StopEngine;
    context.current.coolant = 115;

    context.tickAndResolve();

    TEST_ASSERT_FALSE(context.current.engineProtect.forkLimit);
    TEST_ASSERT_FALSE(context.current.engineProtect.forkStop);
    TEST_ASSERT_EQUAL_UINT8(ACTION_WARN, protectionState.action);
    TEST_ASSERT_EQUAL_HEX8(1U << PROTECT_COOLANT_HOT, getProtectionLogReason());
}

// Memory: the action stays after the condition clears
static void test_memory_keeps_the_action(void)
{
    protection_context_t context;
    context.page15.protLatch = 1U << PROTECT_OIL_PRESSURE;
    context.setUpstream(true, false, false);
    context.resolve();

    context.setUpstream(false, false, false);
    context.tickAndResolve();

    TEST_ASSERT_TRUE(context.current.engineProtect.forkLimit); // The upstream flag is gone: the fork's flag limits
    TEST_ASSERT_EQUAL_UINT8(ACTION_LIMIT, protectionState.action);
    TEST_ASSERT_EQUAL_HEX8(0x80U | (1U << PROTECT_OIL_PRESSURE), getProtectionLogReason());
    TEST_ASSERT_EQUAL_UINT8(TEST_LIMP_RPM, applyEngineProtectionRevLimit(UINT8_MAX, context.current, context.page4));
}

static void test_no_memory_ends_with_the_condition(void)
{
    protection_context_t context;
    context.setUpstream(true, false, false);
    context.resolve();

    context.setUpstream(false, false, false);
    context.tickAndResolve();

    TEST_ASSERT_FALSE(context.current.engineProtect.isActive());
    TEST_ASSERT_EQUAL_UINT8(ACTION_NONE, protectionState.action);
    TEST_ASSERT_EQUAL_HEX8(0U, getProtectionLogReason());
}

// Memory is per condition
static void test_memory_is_per_condition(void)
{
    protection_context_t context;
    context.page15.protLatch = 1U << PROTECT_BOOST;
    context.setUpstream(true, true, false);
    context.resolve();

    context.setUpstream(false, false, false);
    context.tickAndResolve();

    TEST_ASSERT_EQUAL_HEX8(0x80U | (1U << PROTECT_BOOST), getProtectionLogReason());
}

static void test_memory_clears_when_the_engine_stops(void)
{
    protection_context_t context;
    context.page15.protLatch = 1U << PROTECT_OIL_PRESSURE;
    context.setUpstream(true, false, false);
    context.resolve();
    context.setUpstream(false, false, false);
    context.tickAndResolve();
    TEST_ASSERT_EQUAL_HEX8(0x80U | (1U << PROTECT_OIL_PRESSURE), getProtectionLogReason());

    context.current.rotationStatus = EngineRotationStatus::Stopped;
    context.current.setRpm(0U);
    context.tick();

    TEST_ASSERT_EQUAL_HEX8(0U, getProtectionLogReason());
    TEST_ASSERT_EQUAL_UINT8(ACTION_NONE, protectionState.action);
    TEST_ASSERT_FALSE(context.current.engineProtect.isActive());

    // A fresh start judges the conditions again
    context.current.rotationStatus = EngineRotationStatus::Running;
    context.current.setRpm(2000U);
    context.tickAndResolve();
    TEST_ASSERT_EQUAL_HEX8(0U, getProtectionLogReason());
}

// A stopped engine still shows the fork's conditions (E.g. a sensor fault with the key on)
static void test_stopped_engine_shows_fork_conditions(void)
{
    protection_context_t context;
    context.page15.protSensorFaultEnable = 1U;
    context.current.rotationStatus = EngineRotationStatus::Stopped;
    context.current.setRpm(0U);
    context.current.sensorFaults = 1U << SENSOR_FAULT_MAP;

    context.tick();

    TEST_ASSERT_EQUAL_HEX8(1U << PROTECT_SENSOR_FAULT, getProtectionLogReason());
    TEST_ASSERT_EQUAL_UINT8(ACTION_LIMIT, protectionState.action);
}

// An unused action value in the tune can't cut anything
static void test_invalid_action_is_warn_only(void)
{
    protection_context_t context;
    context.page15.protActionOilPress = 3U;
    context.setUpstream(true, false, false);

    context.resolve();

    TEST_ASSERT_FALSE(context.current.engineProtect.oil);
    TEST_ASSERT_EQUAL_UINT8(ACTION_WARN, protectionState.action);
}

static void test_flags_reset_and_is_active(void)
{
    statuses::engine_protect_flags_t flags = {};
    TEST_ASSERT_FALSE(flags.isActive());

    flags.forkLimit = true;
    TEST_ASSERT_TRUE(flags.isActive());
    flags.reset();
    TEST_ASSERT_FALSE(flags.forkLimit);
    TEST_ASSERT_FALSE(flags.isActive());

    flags.forkStop = true;
    TEST_ASSERT_TRUE(flags.isActive());
    flags.reset();
    TEST_ASSERT_FALSE(flags.forkStop);
}

void testProtectionResolve(void)
{
    SET_UNITY_FILENAME() {
        RUN_TEST_P(test_nothing_active);
        RUN_TEST_P(test_upstream_limit_keeps_the_flag);
        RUN_TEST_P(test_upstream_warn_clears_the_flag);
        RUN_TEST_P(test_upstream_stop_cuts_at_any_rpm);
        RUN_TEST_P(test_fork_condition_limits);
        RUN_TEST_P(test_fork_condition_warns);
        RUN_TEST_P(test_fork_condition_stops);
        RUN_TEST_P(test_action_precedence);
        RUN_TEST_P(test_nothing_cut_with_protection_off);
        RUN_TEST_P(test_memory_keeps_the_action);
        RUN_TEST_P(test_no_memory_ends_with_the_condition);
        RUN_TEST_P(test_memory_is_per_condition);
        RUN_TEST_P(test_memory_clears_when_the_engine_stops);
        RUN_TEST_P(test_stopped_engine_shows_fork_conditions);
        RUN_TEST_P(test_invalid_action_is_warn_only);
        RUN_TEST_P(test_flags_reset_and_is_active);
    }
}
