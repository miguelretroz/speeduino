#include <string.h>
#include <unity.h>
#include "../test_utils.h"
#include "../fake_fork_storage.h"
#include "protection_fixture.h"
#include "fork_updates.h"

// Page 15 starts with the 80 byte boost duty table
static constexpr uint16_t CONFIG15_PAGE_OFFSET = 80U;
// The protection fields: INI bytes 163 to 171
static constexpr uint16_t PROTECT_FIRST_BYTE = 163U;
static constexpr uint16_t PROTECT_BYTES = 9U;

static uint8_t *getProtectBytes(void)
{
    return (uint8_t*)&configPage15 + (PROTECT_FIRST_BYTE - CONFIG15_PAGE_OFFSET);
}

static void assert_protect_defaults(void)
{
    TEST_ASSERT_EQUAL(0U, configPage15.protWarnPin);
    TEST_ASSERT_EQUAL(0U, configPage15.protUnused1);
    TEST_ASSERT_EQUAL(5U, configPage15.protFuelPressRpm);
    TEST_ASSERT_EQUAL(10U, configPage15.protFuelPressDelay);
    TEST_ASSERT_EQUAL(30U, configPage15.protFuelPressMin);
    TEST_ASSERT_EQUAL(temperatureAddOffset(110), configPage15.protCoolantHot);
    TEST_ASSERT_EQUAL(0U, configPage15.protCoolantHotEnable);
    TEST_ASSERT_EQUAL(0U, configPage15.protOilHotEnable);
    TEST_ASSERT_EQUAL(0U, configPage15.protFuelPressEnable);
    TEST_ASSERT_EQUAL(0U, configPage15.protSensorFaultEnable);
    TEST_ASSERT_EQUAL(0U, configPage15.protWarnInv);
    TEST_ASSERT_EQUAL(0U, configPage15.protUnused2);
    TEST_ASSERT_EQUAL(0U, configPage15.protLatch);
    // Upstream's protections keep limiting the RPM
    TEST_ASSERT_EQUAL((uint8_t)ProtectAction::LimitRpm, configPage15.protActionOilPress);
    TEST_ASSERT_EQUAL((uint8_t)ProtectAction::LimitRpm, configPage15.protActionBoost);
    TEST_ASSERT_EQUAL((uint8_t)ProtectAction::LimitRpm, configPage15.protActionAfr);
    TEST_ASSERT_EQUAL((uint8_t)ProtectAction::WarnOnly, configPage15.protActionCoolantHot);
    TEST_ASSERT_EQUAL((uint8_t)ProtectAction::WarnOnly, configPage15.protActionOilHot);
    TEST_ASSERT_EQUAL((uint8_t)ProtectAction::LimitRpm, configPage15.protActionFuelPress);
    TEST_ASSERT_EQUAL((uint8_t)ProtectAction::LimitRpm, configPage15.protActionSensorFault);
    TEST_ASSERT_EQUAL(0U, configPage15.protUnused3);
}

static void test_defaults_on_fresh_eeprom(void)
{
    resetFakeForkStorage();
    memset(getProtectBytes(), 0xFF, PROTECT_BYTES);

    doForkUpdates(configPage15);

    assert_protect_defaults();
    TEST_ASSERT_EQUAL(FORK_CONFIG_LATEST, fakeForkStorage.version);
}

// A board on config version 4 gets the protection defaults with the sensor fault ones
static void test_v4_upgrade_adds_protect_defaults(void)
{
    resetFakeForkStorage();
    doForkUpdates(configPage15);
    configPage15.pidTuneIdleAmplitude = 20U;
    fakeForkStorage.version = FORK_CONFIG_V4;
    memset(getProtectBytes(), 0xFF, PROTECT_BYTES);

    doForkUpdates(configPage15);

    assert_protect_defaults();
    TEST_ASSERT_EQUAL(FORK_CONFIG_V5, fakeForkStorage.version);
    TEST_ASSERT_EQUAL(20U, configPage15.pidTuneIdleAmplitude);
}

static void test_v5_unchanged(void)
{
    resetFakeForkStorage();
    doForkUpdates(configPage15);
    configPage15.protActionBoost = (uint8_t)ProtectAction::StopEngine;
    configPage15.protLatch = 0x7FU;
    configPage15.protWarnPin = 23U;

    doForkUpdates(configPage15);

    TEST_ASSERT_EQUAL((uint8_t)ProtectAction::StopEngine, configPage15.protActionBoost);
    TEST_ASSERT_EQUAL(0x7FU, configPage15.protLatch);
    TEST_ASSERT_EQUAL(23U, configPage15.protWarnPin);
}

// The INI file addresses these fields by page offset
static void test_protect_ini_offsets(void)
{
    const uint8_t *pBase = (const uint8_t*)&configPage15;
    uint8_t *pBytes = getProtectBytes();

    TEST_ASSERT_EQUAL(256U - CONFIG15_PAGE_OFFSET, sizeof(config15));
    TEST_ASSERT_EQUAL(106U - CONFIG15_PAGE_OFFSET, (const uint8_t*)&configPage15.Unused15_106_162 - pBase);
    TEST_ASSERT_EQUAL(57U, sizeof(configPage15.Unused15_106_162));
    TEST_ASSERT_EQUAL(164U - CONFIG15_PAGE_OFFSET, (const uint8_t*)&configPage15.protFuelPressRpm - pBase);
    TEST_ASSERT_EQUAL(165U - CONFIG15_PAGE_OFFSET, (const uint8_t*)&configPage15.protFuelPressDelay - pBase);
    TEST_ASSERT_EQUAL(166U - CONFIG15_PAGE_OFFSET, (const uint8_t*)&configPage15.protFuelPressMin - pBase);
    TEST_ASSERT_EQUAL(167U - CONFIG15_PAGE_OFFSET, (const uint8_t*)&configPage15.protCoolantHot - pBase);
    TEST_ASSERT_EQUAL(169U - CONFIG15_PAGE_OFFSET, (const uint8_t*)&configPage15.protLatch - pBase);
    // The sensor fault fields still start at 172
    TEST_ASSERT_EQUAL(172U - CONFIG15_PAGE_OFFSET, (const uint8_t*)&configPage15.tpsFailsafe - pBase);

    // Byte 163: the pin in the low 6 bits
    memset(pBytes, 0, PROTECT_BYTES);
    configPage15.protWarnPin = 23U;
    TEST_ASSERT_EQUAL_HEX8(23U, pBytes[0]);

    // Byte 168: the enable bits, in INI bit order
    configPage15.protCoolantHotEnable = 1U;
    TEST_ASSERT_EQUAL_HEX8(0x01U, pBytes[5]);
    configPage15.protCoolantHotEnable = 0U; configPage15.protOilHotEnable = 1U;
    TEST_ASSERT_EQUAL_HEX8(0x02U, pBytes[5]);
    configPage15.protOilHotEnable = 0U; configPage15.protFuelPressEnable = 1U;
    TEST_ASSERT_EQUAL_HEX8(0x04U, pBytes[5]);
    configPage15.protFuelPressEnable = 0U; configPage15.protSensorFaultEnable = 1U;
    TEST_ASSERT_EQUAL_HEX8(0x08U, pBytes[5]);
    configPage15.protSensorFaultEnable = 0U; configPage15.protWarnInv = 1U;
    TEST_ASSERT_EQUAL_HEX8(0x10U, pBytes[5]);
    configPage15.protWarnInv = 0U;

    // Byte 170: oil hot [0:1], fuel pressure [2:3], sensor fault [4:5]
    configPage15.protActionOilHot = 1U;
    TEST_ASSERT_EQUAL_HEX8(0x01U, pBytes[7]);
    configPage15.protActionOilHot = 0U; configPage15.protActionFuelPress = 2U;
    TEST_ASSERT_EQUAL_HEX8(0x08U, pBytes[7]);
    configPage15.protActionFuelPress = 0U; configPage15.protActionSensorFault = 3U;
    TEST_ASSERT_EQUAL_HEX8(0x30U, pBytes[7]);
    configPage15.protActionSensorFault = 0U;

    // Byte 171: oil pressure [0:1], boost [2:3], AFR [4:5], hot coolant [6:7]
    configPage15.protActionOilPress = 1U;
    TEST_ASSERT_EQUAL_HEX8(0x01U, pBytes[8]);
    configPage15.protActionOilPress = 0U; configPage15.protActionBoost = 2U;
    TEST_ASSERT_EQUAL_HEX8(0x08U, pBytes[8]);
    configPage15.protActionBoost = 0U; configPage15.protActionAfr = 1U;
    TEST_ASSERT_EQUAL_HEX8(0x10U, pBytes[8]);
    configPage15.protActionAfr = 0U; configPage15.protActionCoolantHot = 2U;
    TEST_ASSERT_EQUAL_HEX8(0x80U, pBytes[8]);
    configPage15.protActionCoolantHot = 0U;
}

// The memory bits follow the condition numbers
static void test_latch_bits_match_conditions(void)
{
    config15 page15 = {};
    setDefaults(page15);
    page15.protLatch = 1U << PROTECT_SENSOR_FAULT;

    TEST_ASSERT_EQUAL_HEX8(0x40U, page15.protLatch);
    TEST_ASSERT_TRUE(BIT_CHECK(page15.protLatch, PROTECT_SENSOR_FAULT));
    TEST_ASSERT_FALSE(BIT_CHECK(page15.protLatch, PROTECT_OIL_PRESSURE));
}

static void test_get_action(void)
{
    config15 page15 = {};
    setDefaults(page15);

    TEST_ASSERT_EQUAL((uint8_t)ProtectAction::LimitRpm, (uint8_t)getAction(page15, PROTECT_OIL_PRESSURE));
    TEST_ASSERT_EQUAL((uint8_t)ProtectAction::WarnOnly, (uint8_t)getAction(page15, PROTECT_COOLANT_HOT));
    page15.protActionSensorFault = (uint8_t)ProtectAction::StopEngine;
    TEST_ASSERT_EQUAL((uint8_t)ProtectAction::StopEngine, (uint8_t)getAction(page15, PROTECT_SENSOR_FAULT));
    // Unused values and conditions are warn only
    page15.protActionAfr = 3U;
    TEST_ASSERT_EQUAL((uint8_t)ProtectAction::WarnOnly, (uint8_t)getAction(page15, PROTECT_AFR));
    TEST_ASSERT_EQUAL((uint8_t)ProtectAction::WarnOnly, (uint8_t)getAction(page15, 7U));
}

void testProtectionConfig(void)
{
    SET_UNITY_FILENAME() {
        RUN_TEST_P(test_defaults_on_fresh_eeprom);
        RUN_TEST_P(test_v4_upgrade_adds_protect_defaults);
        RUN_TEST_P(test_v5_unchanged);
        RUN_TEST_P(test_protect_ini_offsets);
        RUN_TEST_P(test_latch_bits_match_conditions);
        RUN_TEST_P(test_get_action);
    }
}
