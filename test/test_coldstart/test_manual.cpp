#include "../test_utils.h"
#include "coldstart_fixture.h"

// Fork: the manual pump hold ('E' 0x42, a button held on the telemetry dash). Each HOLD keeps the output on for
// 400 ms; only with the key on, a valid coolant reading below coldStartManualMaxClt and within the on time budget
// of the press. The flood clear and low battery cuts still win.
extern bool coldStartPumpCommandCore(uint16_t command, uint32_t nowMs);

static void hold(coldstart_context_t &c) { (void)coldStartPumpCommandCore(TS_CMD_COLD_START_PUMP_HOLD, c.nowMs); }
static void release(coldstart_context_t &c) { (void)coldStartPumpCommandCore(TS_CMD_COLD_START_PUMP_RELEASE, c.nowMs); }

static coldstart_context_t setup_manual(void)
{
    coldstart_context_t c = setup_coldstart();
    c.keyOn();
    (void)c.runFor(100U); // the key on edge (prime is off by default)
    return c;
}

static void test_group(void)
{
    coldstart_context_t c = setup_manual();
    TEST_ASSERT_FALSE(coldStartPumpCommandCore(0x4101U, c.nowMs)); // the remote clutch
    TEST_ASSERT_FALSE(coldStartPumpCommandCore(0x4301U, c.nowMs)); // the Valet
    TEST_ASSERT_TRUE(coldStartPumpCommandCore(TS_CMD_COLD_START_PUMP_HOLD, c.nowMs));
    TEST_ASSERT_TRUE(coldStartPumpCommandCore(0x42FFU, c.nowMs)); // unknown in the group: releases
    c.run();
    TEST_ASSERT_FALSE(c.isOutputOn());
}

static void test_hold_and_expire(void)
{
    coldstart_context_t c = setup_manual();
    TEST_ASSERT_FALSE(c.isOutputOn());
    hold(c);
    c.run();
    TEST_ASSERT_TRUE(c.isOutputOn());
    TEST_ASSERT_TRUE(coldStartManualActive());
    TEST_ASSERT_TRUE(c.isPinHigh());
    (void)c.runFor(300U);
    TEST_ASSERT_TRUE(c.isOutputOn());
    (void)c.runFor(200U); // not renewed: off 400 ms after the hold
    TEST_ASSERT_FALSE(c.isOutputOn());
    TEST_ASSERT_FALSE(coldStartManualActive());
}

static void test_release(void)
{
    coldstart_context_t c = setup_manual();
    hold(c);
    c.run();
    TEST_ASSERT_TRUE(c.isOutputOn());
    release(c);
    c.run();
    TEST_ASSERT_FALSE(c.isOutputOn());
}

static void test_cuts(void)
{
    coldstart_context_t c = setup_manual();
    c.current.TPS = c.page4.floodClear;
    hold(c);
    c.run();
    TEST_ASSERT_FALSE(c.isOutputOn()); // flood clear
    c.current.TPS = 0U;
    c.current.battery10 = 70U; // below the 8V cut, key still on
    hold(c);
    c.run();
    TEST_ASSERT_FALSE(c.isOutputOn()); // low battery
    c.current.battery10 = BATTERY_KEY_ON;
    (void)c.runFor(20U);
    c.current.coolant = 45; // above the 40C limit
    hold(c);
    c.run();
    TEST_ASSERT_FALSE(c.isOutputOn());
    c.current.coolant = -35; // below the valid reading: a sensor fault
    hold(c);
    c.run();
    TEST_ASSERT_FALSE(c.isOutputOn());
}

static void test_key_off(void)
{
    coldstart_context_t c = setup_coldstart(); // USB power only
    hold(c);
    c.run();
    TEST_ASSERT_FALSE(c.isOutputOn());
}

static void test_budget(void)
{
    coldstart_context_t c = setup_manual();
    uint32_t on = 0U;
    for (uint32_t t = 0U; t < 7000U; t += 100U) // held for 7 s, renewed every 100 ms
    {
        hold(c);
        on += c.runFor(100U);
    }
    TEST_ASSERT_UINT32_WITHIN(150U, 5000U, on); // the 5 s budget of a press
    TEST_ASSERT_FALSE(c.isOutputOn());
    release(c);
    c.run();
    hold(c);
    c.run();
    TEST_ASSERT_TRUE(c.isOutputOn()); // a new press: the budget starts again
}

static void test_engine_running(void)
{
    coldstart_context_t c = setup_manual();
    c.engineRunning();
    hold(c);
    c.run();
    TEST_ASSERT_TRUE(c.isOutputOn()); // a cold ethanol engine stumbling
}

static void test_system_disabled(void)
{
    coldstart_context_t c = setup_coldstart();
    c.page15.coldStartEnable = 0U;
    c.initialise();
    c.keyOn();
    (void)c.runFor(100U);
    hold(c);
    c.run();
    TEST_ASSERT_FALSE(c.isOutputOn());
    TEST_ASSERT_FALSE(coldStartManualActive());
}

void testColdStartManual(void)
{
    SET_UNITY_FILENAME() {
        RUN_TEST_P(test_group);
        RUN_TEST_P(test_hold_and_expire);
        RUN_TEST_P(test_release);
        RUN_TEST_P(test_cuts);
        RUN_TEST_P(test_key_off);
        RUN_TEST_P(test_budget);
        RUN_TEST_P(test_engine_running);
        RUN_TEST_P(test_system_disabled);
    }
}
