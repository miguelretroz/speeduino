#include <string.h>
#include <unity.h>
#include "../test_utils.h"
#include "../fake_fork_storage.h"
#include "sensorfault_fixture.h"
#include "fork_updates.h"

// Page 15 starts with the 80 byte boost duty table
static constexpr uint16_t CONFIG15_PAGE_OFFSET = 80U;
// The sensor fault fields: INI bytes 172 to 185
static constexpr uint16_t SENSORFAULT_FIRST_BYTE = 172U;
static constexpr uint16_t SENSORFAULT_BYTES = 14U;

static uint8_t *getSensorFaultBytes(void)
{
    return (uint8_t*)&configPage15 + (SENSORFAULT_FIRST_BYTE - CONFIG15_PAGE_OFFSET);
}

static void assert_sensorfault_defaults(void)
{
    TEST_ASSERT_EQUAL(2U, configPage15.cltFaultLow);
    TEST_ASSERT_EQUAL(253U, configPage15.cltFaultHigh);
    TEST_ASSERT_EQUAL(2U, configPage15.iatFaultLow);
    TEST_ASSERT_EQUAL(253U, configPage15.iatFaultHigh);
    TEST_ASSERT_EQUAL(12U, configPage15.mapFaultLow);
    TEST_ASSERT_EQUAL(250U, configPage15.mapFaultHigh);
    TEST_ASSERT_EQUAL(2U, configPage15.tpsFaultLow);
    TEST_ASSERT_EQUAL(253U, configPage15.tpsFaultHigh);
    TEST_ASSERT_EQUAL(temperatureAddOffset(60), configPage15.cltFailsafe);
    TEST_ASSERT_EQUAL(temperatureAddOffset(30), configPage15.iatFailsafe);
    TEST_ASSERT_EQUAL(100U, configPage15.mapFailsafe);
    TEST_ASSERT_EQUAL(20U, configPage15.tpsFailsafe);
    TEST_ASSERT_EQUAL(10U, configPage15.sensorFaultDelay);
    // Detection off, fan option on
    TEST_ASSERT_EQUAL(0U, configPage15.sensorFaultClt);
    TEST_ASSERT_EQUAL(0U, configPage15.sensorFaultIat);
    TEST_ASSERT_EQUAL(0U, configPage15.sensorFaultMap);
    TEST_ASSERT_EQUAL(0U, configPage15.sensorFaultTps);
    TEST_ASSERT_EQUAL(1U, configPage15.sensorFaultFan);
    TEST_ASSERT_EQUAL(0U, configPage15.sensorFaultUnused);
}

static void test_defaults_on_fresh_eeprom(void)
{
    resetFakeForkStorage();
    memset(getSensorFaultBytes(), 0xFF, SENSORFAULT_BYTES);

    doForkUpdates(configPage15);

    assert_sensorfault_defaults();
    TEST_ASSERT_EQUAL(FORK_CONFIG_V5, fakeForkStorage.version);
}

// A board on config version 4: keep the auto tune, oil and cold start settings, add the sensor fault defaults
static void test_v4_upgrade_adds_sensorfault_defaults(void)
{
    resetFakeForkStorage();
    doForkUpdates(configPage15);
    configPage15.coldStartEnable = 1U;
    configPage15.oilTempEnable = 1U;
    configPage15.pidTuneIdleAmplitude = 20U;
    configPage15.pidAdaptIdle = 1U;
    fakeForkStorage.attempts = 2U;
    fakeForkStorage.version = FORK_CONFIG_V4;
    fakeForkStorage.pidAdaptIdle = 120U;
    // Unused bytes before version 5
    memset(getSensorFaultBytes(), 0xFF, SENSORFAULT_BYTES);

    doForkUpdates(configPage15);

    assert_sensorfault_defaults();
    TEST_ASSERT_EQUAL(FORK_CONFIG_V5, fakeForkStorage.version);
    // Untouched
    TEST_ASSERT_EQUAL(1U, configPage15.coldStartEnable);
    TEST_ASSERT_EQUAL(1U, configPage15.oilTempEnable);
    TEST_ASSERT_EQUAL(20U, configPage15.pidTuneIdleAmplitude);
    TEST_ASSERT_EQUAL(1U, configPage15.pidAdaptIdle);
    TEST_ASSERT_EQUAL(2U, fakeForkStorage.attempts);
    TEST_ASSERT_EQUAL_UINT8(120U, fakeForkStorage.pidAdaptIdle);
}

// A board on version 3 goes through version 4 and 5
static void test_v3_upgrade_adds_both(void)
{
    resetFakeForkStorage();
    doForkUpdates(configPage15);
    fakeForkStorage.version = FORK_CONFIG_V3;
    configPage15.pidTuneIdleAmplitude = 0xFFU;
    memset(getSensorFaultBytes(), 0xFF, SENSORFAULT_BYTES);

    doForkUpdates(configPage15);

    assert_sensorfault_defaults();
    TEST_ASSERT_EQUAL(8U, configPage15.pidTuneIdleAmplitude);
    TEST_ASSERT_EQUAL(FORK_CONFIG_V5, fakeForkStorage.version);
}

static void test_v5_unchanged(void)
{
    resetFakeForkStorage();
    doForkUpdates(configPage15);
    configPage15.sensorFaultClt = 1U;
    configPage15.cltFailsafe = temperatureAddOffset(80);
    configPage15.sensorFaultDelay = 30U;

    doForkUpdates(configPage15);

    TEST_ASSERT_EQUAL(1U, configPage15.sensorFaultClt);
    TEST_ASSERT_EQUAL(temperatureAddOffset(80), configPage15.cltFailsafe);
    TEST_ASSERT_EQUAL(30U, configPage15.sensorFaultDelay);
}

// The INI file addresses these fields by page offset
static void test_sensorfault_ini_offsets(void)
{
    const uint8_t *pBase = (const uint8_t*)&configPage15;

    TEST_ASSERT_EQUAL(256U - CONFIG15_PAGE_OFFSET, sizeof(config15));
    TEST_ASSERT_EQUAL(172U - CONFIG15_PAGE_OFFSET, (const uint8_t*)&configPage15.tpsFailsafe - pBase);
    TEST_ASSERT_EQUAL(173U - CONFIG15_PAGE_OFFSET, (const uint8_t*)&configPage15.mapFailsafe - pBase);
    TEST_ASSERT_EQUAL(174U - CONFIG15_PAGE_OFFSET, (const uint8_t*)&configPage15.iatFailsafe - pBase);
    TEST_ASSERT_EQUAL(175U - CONFIG15_PAGE_OFFSET, (const uint8_t*)&configPage15.cltFailsafe - pBase);
    TEST_ASSERT_EQUAL(176U - CONFIG15_PAGE_OFFSET, (const uint8_t*)&configPage15.tpsFaultHigh - pBase);
    TEST_ASSERT_EQUAL(177U - CONFIG15_PAGE_OFFSET, (const uint8_t*)&configPage15.tpsFaultLow - pBase);
    TEST_ASSERT_EQUAL(178U - CONFIG15_PAGE_OFFSET, (const uint8_t*)&configPage15.mapFaultHigh - pBase);
    TEST_ASSERT_EQUAL(179U - CONFIG15_PAGE_OFFSET, (const uint8_t*)&configPage15.mapFaultLow - pBase);
    TEST_ASSERT_EQUAL(180U - CONFIG15_PAGE_OFFSET, (const uint8_t*)&configPage15.iatFaultHigh - pBase);
    TEST_ASSERT_EQUAL(181U - CONFIG15_PAGE_OFFSET, (const uint8_t*)&configPage15.iatFaultLow - pBase);
    TEST_ASSERT_EQUAL(182U - CONFIG15_PAGE_OFFSET, (const uint8_t*)&configPage15.cltFaultHigh - pBase);
    TEST_ASSERT_EQUAL(183U - CONFIG15_PAGE_OFFSET, (const uint8_t*)&configPage15.cltFaultLow - pBase);
    TEST_ASSERT_EQUAL(184U - CONFIG15_PAGE_OFFSET, (const uint8_t*)&configPage15.sensorFaultDelay - pBase);
    // Byte 185 holds the enable bits, in INI bit order
    uint8_t *pBits = getSensorFaultBytes() + 13U;
    configPage15.sensorFaultClt = 1U; configPage15.sensorFaultIat = 0U; configPage15.sensorFaultMap = 0U; configPage15.sensorFaultTps = 0U; configPage15.sensorFaultFan = 0U; configPage15.sensorFaultUnused = 0U;
    TEST_ASSERT_EQUAL_HEX8(0x01U, *pBits);
    configPage15.sensorFaultClt = 0U; configPage15.sensorFaultIat = 1U;
    TEST_ASSERT_EQUAL_HEX8(0x02U, *pBits);
    configPage15.sensorFaultIat = 0U; configPage15.sensorFaultMap = 1U;
    TEST_ASSERT_EQUAL_HEX8(0x04U, *pBits);
    configPage15.sensorFaultMap = 0U; configPage15.sensorFaultTps = 1U;
    TEST_ASSERT_EQUAL_HEX8(0x08U, *pBits);
    configPage15.sensorFaultTps = 0U; configPage15.sensorFaultFan = 1U;
    TEST_ASSERT_EQUAL_HEX8(0x10U, *pBits);
    // The PID auto tune fields still start at 186
    TEST_ASSERT_EQUAL(186U - CONFIG15_PAGE_OFFSET, (const uint8_t*)&configPage15.pidAdaptEgoBand - pBase);
}

void testSensorFaultConfig(void)
{
    SET_UNITY_FILENAME() {
        RUN_TEST_P(test_defaults_on_fresh_eeprom);
        RUN_TEST_P(test_v4_upgrade_adds_sensorfault_defaults);
        RUN_TEST_P(test_v3_upgrade_adds_both);
        RUN_TEST_P(test_v5_unchanged);
        RUN_TEST_P(test_sensorfault_ini_offsets);
    }
}
