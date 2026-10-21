#include <string.h>
#include <unity.h>
#include "../test_utils.h"
#include "../fake_fork_storage.h"
#include "globals.h"
#include "fork_updates.h"
#include "units.h"

// Page 15 starts with the 80 byte boost duty table
static constexpr uint16_t CONFIG15_PAGE_OFFSET = 80U;
// The PID auto tune fields: INI bytes 186 to 196
static constexpr uint16_t PIDTUNE_FIRST_BYTE = 186U;
static constexpr uint16_t PIDTUNE_BYTES = 11U;

static uint8_t *getPidTuneBytes(void)
{
    return (uint8_t*)&configPage15 + (PIDTUNE_FIRST_BYTE - CONFIG15_PAGE_OFFSET);
}

static void assert_pidtune_defaults(void)
{
    TEST_ASSERT_EQUAL(2U, configPage15.pidAdaptEgoBand);
    TEST_ASSERT_EQUAL(25U, configPage15.pidAdaptIdleBand);
    TEST_ASSERT_EQUAL(15U, configPage15.pidAdaptWindow);
    TEST_ASSERT_EQUAL(5U, configPage15.pidAdaptStep);
    TEST_ASSERT_EQUAL(90U, configPage15.pidTuneTimeout);
    TEST_ASSERT_EQUAL(3U, configPage15.pidTuneCycles);
    TEST_ASSERT_EQUAL(1U, configPage15.pidTuneEgoHyst);
    TEST_ASSERT_EQUAL(5U, configPage15.pidTuneEgoAmplitude);
    TEST_ASSERT_EQUAL(15U, configPage15.pidTuneIdleHyst);
    TEST_ASSERT_EQUAL(8U, configPage15.pidTuneIdleAmplitude);
    TEST_ASSERT_EQUAL(0U, configPage15.pidAdaptIdle);
    TEST_ASSERT_EQUAL(0U, configPage15.pidAdaptEgo);
    TEST_ASSERT_EQUAL(0U, configPage15.pidTuneRule);
    TEST_ASSERT_EQUAL(0U, configPage15.pidTuneUnused);
}

static void test_pidtune_defaults_on_fresh_eeprom(void)
{
    resetFakeForkStorage();
    memset(getPidTuneBytes(), 0xFF, PIDTUNE_BYTES);

    doForkUpdates(configPage15);

    assert_pidtune_defaults();
    TEST_ASSERT_EQUAL(FORK_CONFIG_V4, fakeForkStorage.version);
}

// A board on config version 3: keep the cold start and oil settings, add the auto tune defaults
static void test_v3_upgrade_adds_pidtune_defaults(void)
{
    resetFakeForkStorage();
    doForkUpdates(configPage15);
    configPage15.coldStartEnable = 1U;
    configPage15.coldStartPin = 22U;
    configPage15.oilTempEnable = 1U;
    configPage15.oilTempAlertOn = temperatureAddOffset(120);
    fakeForkStorage.attempts = 2U;
    fakeForkStorage.version = FORK_CONFIG_V3;
    // Unused bytes before version 4
    memset(getPidTuneBytes(), 0xFF, PIDTUNE_BYTES);

    doForkUpdates(configPage15);

    assert_pidtune_defaults();
    TEST_ASSERT_EQUAL(FORK_CONFIG_V4, fakeForkStorage.version);
    // Untouched
    TEST_ASSERT_EQUAL(1U, configPage15.coldStartEnable);
    TEST_ASSERT_EQUAL(22U, configPage15.coldStartPin);
    TEST_ASSERT_EQUAL(1U, configPage15.oilTempEnable);
    TEST_ASSERT_EQUAL(temperatureAddOffset(120), configPage15.oilTempAlertOn);
    TEST_ASSERT_EQUAL(2U, fakeForkStorage.attempts);
}

static void test_v4_unchanged(void)
{
    resetFakeForkStorage();
    doForkUpdates(configPage15);
    configPage15.pidTuneIdleAmplitude = 20U;
    configPage15.pidAdaptIdle = 1U;
    configPage15.pidTuneRule = 2U;

    doForkUpdates(configPage15);

    TEST_ASSERT_EQUAL(20U, configPage15.pidTuneIdleAmplitude);
    TEST_ASSERT_EQUAL(1U, configPage15.pidAdaptIdle);
    TEST_ASSERT_EQUAL(2U, configPage15.pidTuneRule);
}

// The INI file addresses these fields by page offset
static void test_pidtune_ini_offsets(void)
{
    const uint8_t *pBase = (const uint8_t*)&configPage15;

    TEST_ASSERT_EQUAL(256U - CONFIG15_PAGE_OFFSET, sizeof(config15));
    TEST_ASSERT_EQUAL(106U - CONFIG15_PAGE_OFFSET, (const uint8_t*)&configPage15.Unused15_106_185 - pBase);
    TEST_ASSERT_EQUAL(80U, sizeof(configPage15.Unused15_106_185));
    TEST_ASSERT_EQUAL(186U - CONFIG15_PAGE_OFFSET, (const uint8_t*)&configPage15.pidAdaptEgoBand - pBase);
    TEST_ASSERT_EQUAL(187U - CONFIG15_PAGE_OFFSET, (const uint8_t*)&configPage15.pidAdaptIdleBand - pBase);
    TEST_ASSERT_EQUAL(188U - CONFIG15_PAGE_OFFSET, (const uint8_t*)&configPage15.pidAdaptWindow - pBase);
    TEST_ASSERT_EQUAL(189U - CONFIG15_PAGE_OFFSET, (const uint8_t*)&configPage15.pidAdaptStep - pBase);
    TEST_ASSERT_EQUAL(190U - CONFIG15_PAGE_OFFSET, (const uint8_t*)&configPage15.pidTuneTimeout - pBase);
    TEST_ASSERT_EQUAL(191U - CONFIG15_PAGE_OFFSET, (const uint8_t*)&configPage15.pidTuneCycles - pBase);
    TEST_ASSERT_EQUAL(192U - CONFIG15_PAGE_OFFSET, (const uint8_t*)&configPage15.pidTuneEgoHyst - pBase);
    TEST_ASSERT_EQUAL(193U - CONFIG15_PAGE_OFFSET, (const uint8_t*)&configPage15.pidTuneEgoAmplitude - pBase);
    TEST_ASSERT_EQUAL(194U - CONFIG15_PAGE_OFFSET, (const uint8_t*)&configPage15.pidTuneIdleHyst - pBase);
    TEST_ASSERT_EQUAL(195U - CONFIG15_PAGE_OFFSET, (const uint8_t*)&configPage15.pidTuneIdleAmplitude - pBase);
    TEST_ASSERT_EQUAL(197U - CONFIG15_PAGE_OFFSET, (const uint8_t*)&configPage15.Unused15_197 - pBase);
    // The oil temperature fields still start at 198
    TEST_ASSERT_EQUAL(201U - CONFIG15_PAGE_OFFSET, (const uint8_t*)&configPage15.oilTempAlertOn - pBase);
}

// Byte 196 holds the bits fields: pidAdaptIdle [0:0], pidAdaptEgo [1:1], pidTuneRule [2:3]
static void test_pidtune_ini_bits(void)
{
    const uint8_t *pBits = (const uint8_t*)&configPage15 + (196U - CONFIG15_PAGE_OFFSET);

    configPage15.pidAdaptIdle = 0U;
    configPage15.pidAdaptEgo = 0U;
    configPage15.pidTuneRule = 0U;
    configPage15.pidTuneUnused = 0U;
    TEST_ASSERT_EQUAL_HEX8(0x00U, *pBits);

    configPage15.pidAdaptIdle = 1U;
    TEST_ASSERT_EQUAL_HEX8(0x01U, *pBits);
    configPage15.pidAdaptEgo = 1U;
    TEST_ASSERT_EQUAL_HEX8(0x03U, *pBits);
    configPage15.pidTuneRule = 3U;
    TEST_ASSERT_EQUAL_HEX8(0x0FU, *pBits);
}

void testPidTuneConfig(void)
{
  SET_UNITY_FILENAME() {
    RUN_TEST_P(test_pidtune_defaults_on_fresh_eeprom);
    RUN_TEST_P(test_v3_upgrade_adds_pidtune_defaults);
    RUN_TEST_P(test_v4_unchanged);
    RUN_TEST_P(test_pidtune_ini_offsets);
    RUN_TEST_P(test_pidtune_ini_bits);
  }
}
