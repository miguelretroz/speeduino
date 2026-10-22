#include <unity.h>
#include "../test_utils.h"
#include "protection_fixture.h"
#include "logger.h"

static void test_reason_byte(void)
{
    protection_context_t context;
    protectionState.active = (1U << PROTECT_AFR) | (1U << PROTECT_FUEL_PRESSURE);
    protectionState.latched = 0U;
    TEST_ASSERT_EQUAL_HEX8(0x24U, getProtectionLogReason());

    protectionState.latched = 1U << PROTECT_AFR;
    TEST_ASSERT_EQUAL_HEX8(0xA4U, getProtectionLogReason());
}

static void test_state_byte(void)
{
    protection_context_t context;
    protectionState.action = ACTION_NONE;
    protectionState.warnOutputOn = false;
    TEST_ASSERT_EQUAL_HEX8(0x00U, getProtectionLogState());

    protectionState.action = ACTION_STOP;
    TEST_ASSERT_EQUAL_HEX8(0x03U, getProtectionLogState());

    protectionState.action = ACTION_WARN;
    protectionState.warnOutputOn = true;
    TEST_ASSERT_EQUAL_HEX8(0x05U, getProtectionLogState());
}

// Live data bytes 158 and 159
static void test_live_data_bytes(void)
{
    protection_context_t context;
    protectionState.active = 1U << PROTECT_OIL_HOT;
    protectionState.latched = 1U << PROTECT_OIL_HOT;
    protectionState.action = ACTION_LIMIT;
    protectionState.warnOutputOn = true;

    TEST_ASSERT_EQUAL_HEX8(0x90U, getTSLogEntry(158U));
    TEST_ASSERT_EQUAL_HEX8(0x06U, getTSLogEntry(159U));
    // Past the end of the fork's bytes
    TEST_ASSERT_EQUAL_HEX8(0x00U, getTSLogEntry(160U));
}

void testProtectionLog(void)
{
    SET_UNITY_FILENAME() {
        RUN_TEST_P(test_reason_byte);
        RUN_TEST_P(test_state_byte);
        RUN_TEST_P(test_live_data_bytes);
    }
}
