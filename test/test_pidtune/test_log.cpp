#include <unity.h>
#include "../test_utils.h"
#include "pidtune_fixture.h"
#include "logger.h"

// Live data bytes 142 to 156
static constexpr uint16_t FIRST_LOG_BYTE = 142U;

static uint8_t logByte(uint8_t index)
{
    return getTSLogEntry(FIRST_LOG_BYTE + index);
}

static void test_log_bytes_before_any_tune(void)
{
    setup_idle_tune();

    TEST_ASSERT_EQUAL_HEX8(0x00U, logByte(0U));
    for (uint8_t index = 1U; index < 12U; ++index)
    {
        TEST_ASSERT_EQUAL_UINT8(0U, logByte(index));
    }
    // No gain adaptation: 100%
    TEST_ASSERT_EQUAL_UINT8(100U, logByte(12U));
    TEST_ASSERT_EQUAL_UINT8(100U, logByte(13U));
    TEST_ASSERT_EQUAL_UINT8(0U, logByte(14U));
    // Past the end
    TEST_ASSERT_EQUAL_UINT8(0U, getTSLogEntry(FIRST_LOG_BYTE + PIDTUNE_LOG_BYTES));
}

static void test_log_bytes_after_an_idle_tune(void)
{
    run_idle_tune_to_done();

    TEST_ASSERT_EQUAL_HEX8(0x03U, logByte(0U)); // Done, idle
    TEST_ASSERT_EQUAL_UINT8(0U, logByte(1U));
    TEST_ASSERT_EQUAL_UINT8(10U, logByte(2U));
    TEST_ASSERT_EQUAL_UINT8(3U, logByte(3U));
    TEST_ASSERT_EQUAL_UINT8(0U, logByte(4U));
    TEST_ASSERT_EQUAL_UINT16(3000U, word(logByte(6U), logByte(5U)));
    TEST_ASSERT_EQUAL_UINT16(40U, word(logByte(8U), logByte(7U)));
    TEST_ASSERT_EQUAL_UINT16(260U, word(logByte(10U), logByte(9U)));
    TEST_ASSERT_EQUAL_UINT8(3U, logByte(11U));
}

static void test_log_failure_and_loop(void)
{
    setup_ego_tune();
    configPage6.egoAlgorithm = EGO_ALGORITHM_SIMPLE;
    (void)send_command(TS_CMD_PIDTUNE_EGO_START, TEST_START_MS);

    TEST_ASSERT_EQUAL_HEX8(0x0CU, logByte(0U)); // Failed, AFR
    TEST_ASSERT_EQUAL_UINT8((uint8_t)pidTune::TuneFail::WrongMode, logByte(1U));
}

// TunerStudio reloads page 6 when it sees the request, so it must stay set until a packet including it has gone out
static void test_log_page_reload_request(void)
{
    run_idle_tune_to_done();
    (void)send_command(TS_CMD_PIDTUNE_APPLY, TEST_START_MS);

    // A packet that didn't include byte 142 (E.g. the first part of a split read)
    pidTuneAckUiRefresh();
    TEST_ASSERT_BITS_HIGH(0x10U, logByte(0U));
    TEST_ASSERT_BITS_HIGH(0x40U, logByte(0U)); // Applied

    // Once sent, the request is cleared
    pidTuneAckUiRefresh();
    TEST_ASSERT_BITS_LOW(0x10U, logByte(0U));
    TEST_ASSERT_BITS_HIGH(0x40U, logByte(0U));
}

void testPidTuneLog(void)
{
  SET_UNITY_FILENAME() {
    RUN_TEST_P(test_log_bytes_before_any_tune);
    RUN_TEST_P(test_log_bytes_after_an_idle_tune);
    RUN_TEST_P(test_log_failure_and_loop);
    RUN_TEST_P(test_log_page_reload_request);
  }
}
