#include <string.h>
#include "../test_utils.h"
#include "oiltemp_fixture.h"
#include "fork_updates.h"
#include "storage.h"
#include "units.h"

// Oil calibration layout in the fake EEPROM: CRC (4), bins (64), values (32)
constexpr uint8_t OIL_VALUES_OFFSET = 4U + 64U;

// A board on config version 2 (cold start only): keep its settings, add the oil temperature defaults
static void test_v2_upgrade_adds_oil_defaults(void)
{
    resetFakeForkStorage();
    fakeForkStorage.version = FORK_CONFIG_V2;
    fakeForkStorage.attempts = 2U;
    configPage15.coldStartEnable = 1U;
    configPage15.coldStartKeyOnReset = 5U;
    // Unused bytes before version 3
    configPage15.oilTempEnable = 1U;
    configPage15.oilTempAlertOn = 0xFFU;
    configPage15.oilTempFilter = 0xFFU;
    memset(oilTempCalibrationTable.values, 0, sizeof(oilTempCalibrationTable.values));

    doForkUpdates(configPage15);

    TEST_ASSERT_EQUAL(FORK_CONFIG_LATEST, fakeForkStorage.version);
    // Cold start untouched
    TEST_ASSERT_EQUAL(1U, configPage15.coldStartEnable);
    TEST_ASSERT_EQUAL(5U, configPage15.coldStartKeyOnReset);
    TEST_ASSERT_EQUAL(2U, fakeForkStorage.attempts);
    // Oil temperature defaults
    TEST_ASSERT_EQUAL(0U, configPage15.oilTempEnable);
    TEST_ASSERT_EQUAL(temperatureAddOffset(130), configPage15.oilTempAlertOn);
    TEST_ASSERT_EQUAL(5U, configPage15.oilTempAlertHyst);
    TEST_ASSERT_EQUAL(180U, configPage15.oilTempFilter);
    // Default calibration, in RAM and in EEPROM
    TEST_ASSERT_EQUAL(255U, oilTempCalibrationTable.values[0]);
    TEST_ASSERT_EQUAL(58U, oilTempCalibrationTable.values[16]);
    TEST_ASSERT_EQUAL(528U, oilTempCalibrationTable.axis[16]);
    TEST_ASSERT_EQUAL(58U, fakeForkStorage.oilCalibration[OIL_VALUES_OFFSET + 16U]);
}

static void test_v3_unchanged(void)
{
    resetFakeForkStorage();
    fakeForkStorage.version = FORK_CONFIG_V3;
    configPage15.oilTempEnable = 1U;
    configPage15.oilTempAlertOn = temperatureAddOffset(120);

    doForkUpdates(configPage15);

    TEST_ASSERT_EQUAL(1U, configPage15.oilTempEnable);
    TEST_ASSERT_EQUAL(temperatureAddOffset(120), configPage15.oilTempAlertOn);
}

// The calibration written by TunerStudio is saved and reloaded on the next boot
static void test_calibration_survives_reboot(void)
{
    resetFakeForkStorage();
    setFlatCalibration(77);
    saveCalibrationTable(SensorCalibrationTable::OilTempSensor);

    setFlatCalibration(0);
    loadAllCalibrationTables();

    TEST_ASSERT_EQUAL(temperatureAddOffset(77), oilTempCalibrationTable.values[0]);
    TEST_ASSERT_EQUAL(temperatureAddOffset(77), oilTempCalibrationTable.values[31]);
    TEST_ASSERT_EQUAL(1023U, oilTempCalibrationTable.axis[31]);
}

void testOilTempUpgrade(void)
{
    SET_UNITY_FILENAME() {
        RUN_TEST_P(test_v2_upgrade_adds_oil_defaults);
        RUN_TEST_P(test_v3_unchanged);
        RUN_TEST_P(test_calibration_survives_reboot);
    }
}
