#include "../test_utils.h"
#include "src/controllers/valet/valetController.h"
#include "src/controllers/valet/valetController_details.h"
#include "../fake_fork_storage.h"
#include "fork_updates.h"
#include "globals.h"
#include "logger.h"
#include "units.h"

// Fork: Valet mode ('E' 0x43). Raising the level always works; lowering it or switching off only right after the
// unlock. The level survives power cycles (EEPROM 3665, a marked byte), and a new RPM limit below the current RPM
// waits for the engine to drop (no jolt).
using namespace valet::details;
extern state_t valetState;
extern bool valetCommandCore(uint16_t command, uint32_t nowMs);
extern void valetUpdateCore(uint16_t rpm, const config15 &page15);

static void setup_valet(void)
{
    resetFakeForkStorage();
    doForkUpdates(configPage15);
    fakeForkStorage.valetWrites = 0U;
    initialiseValet();
}

static void level(uint8_t l, uint32_t now = 1000U)
{
    (void)valetCommandCore((uint16_t)(TS_CMD_VALET_OFF + l), now);
    valetUpdateCore(0U, configPage15);
}

static void unlockTo(uint8_t l, uint32_t now = 1000U)
{
    (void)valetCommandCore(TS_CMD_VALET_UNLOCK, now);
    level(l, now + 10U);
}

static void test_defaults(void)
{
    setup_valet();
    TEST_ASSERT_EQUAL(FORK_CONFIG_LATEST, fakeForkStorage.version);
    TEST_ASSERT_EQUAL_UINT8(50U, configPage15.valetRevLimit[0]);
    TEST_ASSERT_EQUAL_UINT8(40U, configPage15.valetRevLimit[1]);
    TEST_ASSERT_EQUAL_UINT8(30U, configPage15.valetRevLimit[2]);
    TEST_ASSERT_EQUAL_UINT8(150U, configPage15.valetBoostLimit[0]);
    TEST_ASSERT_EQUAL_UINT8(100U, configPage15.valetBoostLimit[2]);
    TEST_ASSERT_EQUAL_UINT8(0U, configPage15.valetIgnRetard[0]);
    TEST_ASSERT_EQUAL_UINT8(6U, configPage15.valetIgnRetard[2]);
    TEST_ASSERT_EQUAL_UINT8(0U, configPage15.valetFlags[1]);
    TEST_ASSERT_EQUAL_UINT8(50U, configPage15.coldStartManualMaxTime);
    TEST_ASSERT_EQUAL_UINT8(temperatureAddOffset(40), configPage15.coldStartManualMaxClt);
    TEST_ASSERT_EQUAL_UINT8(0U, valetLevel());
    TEST_ASSERT_EQUAL_UINT16(0U, fakeForkStorage.valetWrites);
}

static void test_upgrade_from_v5(void)
{
    resetFakeForkStorage();
    doForkUpdates(configPage15);
    configPage15.protFuelPressRpm = 33U; // a V5 field the upgrade must keep
    configPage15.valetRevLimit[1] = 0xEEU;
    fakeForkStorage.version = FORK_CONFIG_V5;
    doForkUpdates(configPage15);
    TEST_ASSERT_EQUAL(FORK_CONFIG_V6, fakeForkStorage.version);
    TEST_ASSERT_EQUAL_UINT8(40U, configPage15.valetRevLimit[1]);
    TEST_ASSERT_EQUAL_UINT8(33U, configPage15.protFuelPressRpm);
}

static void test_command_group(void)
{
    setup_valet();
    TEST_ASSERT_FALSE(valetCommandCore(0x4201U, 1000U)); // the cold start pump
    TEST_ASSERT_FALSE(valetCommandCore(0x4101U, 1000U)); // the remote clutch
    TEST_ASSERT_FALSE(valetCommandCore(0x0100U, 1000U));
    TEST_ASSERT_TRUE(valetCommandCore(0x4301U, 1000U));
    TEST_ASSERT_EQUAL_UINT8(1U, valetLevel());
    TEST_ASSERT_TRUE(valetCommandCore(0x43FFU, 1100U)); // unknown in the group: handled, no effect
    TEST_ASSERT_EQUAL_UINT8(1U, valetLevel());
}

static void test_raise_always_lower_needs_unlock(void)
{
    setup_valet();
    level(2);
    TEST_ASSERT_EQUAL_UINT8(2U, valetLevel());
    level(3);
    TEST_ASSERT_EQUAL_UINT8(3U, valetLevel());
    level(1);
    TEST_ASSERT_EQUAL_UINT8(3U, valetLevel()); // lower without the unlock: refused
    level(0);
    TEST_ASSERT_EQUAL_UINT8(3U, valetLevel()); // off without the unlock: refused
    unlockTo(1);
    TEST_ASSERT_EQUAL_UINT8(1U, valetLevel());
    unlockTo(0);
    TEST_ASSERT_EQUAL_UINT8(0U, valetLevel());
}

static void test_unlock_window(void)
{
    setup_valet();
    level(3);
    (void)valetCommandCore(TS_CMD_VALET_UNLOCK, 5000U);
    (void)valetCommandCore(TS_CMD_VALET_OFF, 5000U + 1900U);
    TEST_ASSERT_EQUAL_UINT8(0U, valetLevel()); // 1.9 s: still open

    level(3);
    (void)valetCommandCore(TS_CMD_VALET_UNLOCK, 9000U);
    (void)valetCommandCore(TS_CMD_VALET_OFF, 9000U + 2100U);
    TEST_ASSERT_EQUAL_UINT8(3U, valetLevel()); // 2.1 s: closed

    // the window serves one selection: a raise right after the unlock uses it up
    level(2, 20000U);
    unlockTo(0); // (back to 0 first)
    level(2, 20000U);
    (void)valetCommandCore(TS_CMD_VALET_UNLOCK, 21000U);
    (void)valetCommandCore((uint16_t)(TS_CMD_VALET_OFF + 3U), 21100U);
    (void)valetCommandCore(TS_CMD_VALET_OFF, 21200U);
    TEST_ASSERT_EQUAL_UINT8(3U, valetLevel());
}

static void test_eeprom(void)
{
    setup_valet();
    level(1);
    TEST_ASSERT_EQUAL_UINT16(1U, fakeForkStorage.valetWrites);
    TEST_ASSERT_EQUAL_HEX8(0xA1U, fakeForkStorage.valetLevel);
    level(1);
    TEST_ASSERT_EQUAL_UINT16(1U, fakeForkStorage.valetWrites); // the same again: no write
    unlockTo(0);
    TEST_ASSERT_EQUAL_UINT16(2U, fakeForkStorage.valetWrites);
    TEST_ASSERT_EQUAL_HEX8(0x00U, fakeForkStorage.valetLevel);

    fakeForkStorage.valetLevel = 0xA2U;
    initialiseValet();
    TEST_ASSERT_EQUAL_UINT8(2U, valetLevel()); // survives power cycles
    TEST_ASSERT_FALSE(valetPending());
    fakeForkStorage.valetLevel = 0x37U;
    initialiseValet();
    TEST_ASSERT_EQUAL_UINT8(0U, valetLevel()); // a random byte is off
    fakeForkStorage.valetLevel = 0xA7U;
    initialiseValet();
    TEST_ASSERT_EQUAL_UINT8(0U, valetLevel()); // a marked byte with no such level is off
}

static void test_pending(void)
{
    setup_valet();
    (void)valetCommandCore((uint16_t)(TS_CMD_VALET_OFF + 3U), 1000U); // 3000 RPM, at 5000 RPM
    valetUpdateCore(5000U, configPage15);
    TEST_ASSERT_EQUAL_UINT8(3U, valetLevel());
    TEST_ASSERT_TRUE(valetPending());
    TEST_ASSERT_EQUAL_UINT8(0U, valetRevLimit(configPage15)); // the old one (none) still in force
    TEST_ASSERT_EQUAL_UINT8(0U, valetBoostLimit(configPage15));
    valetUpdateCore(2900U, configPage15);
    TEST_ASSERT_TRUE(valetPending()); // 2900 + 200 is above 3000
    valetUpdateCore(2800U, configPage15);
    TEST_ASSERT_FALSE(valetPending());
    TEST_ASSERT_EQUAL_UINT8(30U, valetRevLimit(configPage15));

    unlockTo(1); // 5000 RPM: higher than now, in force at once
    valetUpdateCore(2800U, configPage15);
    TEST_ASSERT_FALSE(valetPending());
    TEST_ASSERT_EQUAL_UINT8(50U, valetRevLimit(configPage15));

    (void)valetCommandCore((uint16_t)(TS_CMD_VALET_OFF + 2U), 2000U);
    valetUpdateCore(0U, configPage15); // stopped: at once
    TEST_ASSERT_FALSE(valetPending());

    (void)valetCommandCore(TS_CMD_VALET_UNLOCK, 3000U);
    (void)valetCommandCore(TS_CMD_VALET_OFF, 3010U);
    valetUpdateCore(6000U, configPage15); // off: no limit, at once even at high RPM
    TEST_ASSERT_FALSE(valetPending());
    TEST_ASSERT_EQUAL_UINT8(0U, valetRevLimit(configPage15));
}

static void test_limits(void)
{
    setup_valet();
    TEST_ASSERT_TRUE(valetLaunchAllowed(configPage15));
    TEST_ASSERT_TRUE(valetFlatShiftAllowed(configPage15));
    TEST_ASSERT_EQUAL_UINT8(0U, valetIgnRetard(configPage15));
    level(2);
    TEST_ASSERT_EQUAL_UINT8(40U, valetRevLimit(configPage15));
    TEST_ASSERT_EQUAL_UINT8(120U, valetBoostLimit(configPage15));
    TEST_ASSERT_EQUAL_UINT8(3U, valetIgnRetard(configPage15));
    TEST_ASSERT_FALSE(valetLaunchAllowed(configPage15));
    TEST_ASSERT_FALSE(valetFlatShiftAllowed(configPage15));
    configPage15.valetFlags[1] = 0x02U;
    TEST_ASSERT_FALSE(valetLaunchAllowed(configPage15));
    TEST_ASSERT_TRUE(valetFlatShiftAllowed(configPage15));
    configPage15.valetFlags[1] = 0x01U;
    TEST_ASSERT_TRUE(valetLaunchAllowed(configPage15));
    TEST_ASSERT_FALSE(valetFlatShiftAllowed(configPage15));
    configPage15.valetRevLimit[1] = 0U;
    TEST_ASSERT_EQUAL_UINT8(0U, valetRevLimit(configPage15)); // 0 = no limit
}

static void test_soft_limit(void)
{
    setup_valet();
    TEST_ASSERT_EQUAL_UINT8(60U, valetSoftRevLimit(60U, 65U, configPage15)); // off: the tune's
    level(3);                                                                 // 3000 RPM
    TEST_ASSERT_EQUAL_UINT8(25U, valetSoftRevLimit(60U, 65U, configPage15)); // the same 500 RPM below
    TEST_ASSERT_EQUAL_UINT8(30U, valetSoftRevLimit(60U, 55U, configPage15)); // no gap in the tune
    configPage15.valetRevLimit[2] = 70U;
    TEST_ASSERT_EQUAL_UINT8(60U, valetSoftRevLimit(60U, 65U, configPage15)); // above the tune's: the tune's
}

static void test_boost(void)
{
    setup_valet();
    TEST_ASSERT_EQUAL_UINT16(200U, valetBoostTarget(200U, configPage15));
    TEST_ASSERT_FALSE(valetBoostOverLimit(300U, configPage15));
    level(1); // 150 kPa
    TEST_ASSERT_EQUAL_UINT16(150U, valetBoostTarget(200U, configPage15));
    TEST_ASSERT_EQUAL_UINT16(100U, valetBoostTarget(100U, configPage15));
    TEST_ASSERT_FALSE(valetBoostOverLimit(149U, configPage15));
    TEST_ASSERT_TRUE(valetBoostOverLimit(150U, configPage15));
    TEST_ASSERT_TRUE(valetBoostOverLimit(147U, configPage15)); // hysteresis
    TEST_ASSERT_FALSE(valetBoostOverLimit(144U, configPage15));
}

static void test_realtime_byte(void)
{
    setup_valet();
    TEST_ASSERT_EQUAL_HEX8(0x00U, getTSLogEntry(160U));
    (void)valetCommandCore((uint16_t)(TS_CMD_VALET_OFF + 2U), 1000U);
    valetUpdateCore(6000U, configPage15); // pending
    TEST_ASSERT_EQUAL_HEX8(0x06U, getTSLogEntry(160U)); // level 2 + pending
    valetUpdateCore(0U, configPage15);
    TEST_ASSERT_EQUAL_HEX8(0x02U, getTSLogEntry(160U));
    unlockTo(0);

    // Fork bug fix: bit 0 of the status 5 byte is the hard flat shift (the .ini already read it so)
    currentStatus.launchStatus.flatShiftingHard = true;
    TEST_ASSERT_BITS_HIGH(0x01U, getTSLogEntry(127U));
    currentStatus.launchStatus.flatShiftingHard = false;
    TEST_ASSERT_BITS_LOW(0x01U, getTSLogEntry(127U));
}

void testValet(void)
{
    SET_UNITY_FILENAME() {
        RUN_TEST_P(test_defaults);
        RUN_TEST_P(test_upgrade_from_v5);
        RUN_TEST_P(test_command_group);
        RUN_TEST_P(test_raise_always_lower_needs_unlock);
        RUN_TEST_P(test_unlock_window);
        RUN_TEST_P(test_eeprom);
        RUN_TEST_P(test_pending);
        RUN_TEST_P(test_limits);
        RUN_TEST_P(test_soft_limit);
        RUN_TEST_P(test_boost);
        RUN_TEST_P(test_realtime_byte);
    }
    valetState = state_t(); // (the other suites run with the Valet off)
}
