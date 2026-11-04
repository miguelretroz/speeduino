#include "../test_utils.h"
#include "launch_fixture.h"
#include "units.h"

// Fork: the remote clutch ('E' 0x41). It is a second clutch input: OR-ed with the pin, held for
// REMOTE_CLUTCH_HOLD_MS per HOLD, freed by RELEASE or silence.
extern bool remoteClutchCommandCore(uint16_t command, uint32_t nowMs);
extern bool remoteClutchHeld;

static void releaseRemote(void)
{
    (void)remoteClutchCommandCore(TS_CMD_REMOTE_CLUTCH_RELEASE, 0U);
}

static void test_commandGroup(void)
{
    releaseRemote();
    TEST_ASSERT_TRUE(remoteClutchCommandCore(TS_CMD_REMOTE_CLUTCH_HOLD, 1000U));
    TEST_ASSERT_TRUE(remoteClutchHeld);
    TEST_ASSERT_TRUE(remoteClutchCommandCore(TS_CMD_REMOTE_CLUTCH_RELEASE, 1100U));
    TEST_ASSERT_FALSE(remoteClutchHeld);
    // other groups are not ours (the PID auto tune, the TS test commands)
    TEST_ASSERT_FALSE(remoteClutchCommandCore(0x4000U, 1200U));
    TEST_ASSERT_FALSE(remoteClutchCommandCore(256U, 1200U));
    // an unknown command in the group frees the clutch (the safe side)
    (void)remoteClutchCommandCore(TS_CMD_REMOTE_CLUTCH_HOLD, 1300U);
    TEST_ASSERT_TRUE(remoteClutchCommandCore(0x41FFU, 1310U));
    TEST_ASSERT_FALSE(remoteClutchHeld);
}

static void test_engagesClutchLikeThePin(void)
{
    launch_fixture fixture;
    fixture.init();
    fixture.setClutch(false);
    fixture.page6.flatSEnable = false;
    fixture.current.setRpm(RPM_COARSE.toUser(fixture.page6.flatSArm) - 3);
    fixture.current.TPS = fixture.page10.lnchCtrlTPS + 3;
    fixture.update();
    TEST_ASSERT_FALSE(fixture.current.launchStatus.clutchTrigger);

    (void)remoteClutchCommandCore(TS_CMD_REMOTE_CLUTCH_HOLD, fixture.now);
    fixture.current.setRpm(RPM_COARSE.toUser(fixture.page6.lnchHardLim) + 1);
    fixture.update();
    TEST_ASSERT_TRUE(fixture.current.launchStatus.clutchTrigger);
    TEST_ASSERT_EQUAL_UINT16(fixture.current.RPM, fixture.current.launchStatus.clutchEngagedRPM);
    TEST_ASSERT_TRUE(fixture.current.launchStatus.launchingHard);
    releaseRemote();
}

static void test_expiresWithoutRenewal(void)
{
    launch_fixture fixture;
    fixture.init();
    fixture.setClutch(false);
    (void)remoteClutchCommandCore(TS_CMD_REMOTE_CLUTCH_HOLD, fixture.now);

    fixture.now += REMOTE_CLUTCH_HOLD_MS - 1U;
    fixture.update();
    TEST_ASSERT_TRUE(fixture.current.launchStatus.clutchTrigger);

    fixture.now += 1U;
    fixture.update();
    TEST_ASSERT_FALSE(fixture.current.launchStatus.clutchTrigger);
}

static void test_renewalKeepsItEngaged(void)
{
    launch_fixture fixture;
    fixture.init();
    fixture.setClutch(false);
    for (int i = 0; i < 10; i++)
    {
        (void)remoteClutchCommandCore(TS_CMD_REMOTE_CLUTCH_HOLD, fixture.now);
        fixture.now += 150U;
        fixture.update();
        TEST_ASSERT_TRUE(fixture.current.launchStatus.clutchTrigger);
    }
    (void)remoteClutchCommandCore(TS_CMD_REMOTE_CLUTCH_RELEASE, fixture.now);
    fixture.update();
    TEST_ASSERT_FALSE(fixture.current.launchStatus.clutchTrigger);
}

static void test_pinOrRemote(void)
{
    launch_fixture fixture;
    fixture.init();

    fixture.setClutch(true);
    fixture.update();
    TEST_ASSERT_TRUE(fixture.current.launchStatus.clutchTrigger);

    (void)remoteClutchCommandCore(TS_CMD_REMOTE_CLUTCH_HOLD, fixture.now);
    fixture.update();
    TEST_ASSERT_TRUE(fixture.current.launchStatus.clutchTrigger);

    fixture.setClutch(false);
    fixture.update();
    TEST_ASSERT_TRUE(fixture.current.launchStatus.clutchTrigger);

    releaseRemote();
    fixture.update();
    TEST_ASSERT_FALSE(fixture.current.launchStatus.clutchTrigger);
}

static void test_flatShiftAboveTheSwitchRpm(void)
{
    launch_fixture fixture;
    fixture.init();
    fixture.setClutch(false);
    fixture.current.setRpm(RPM_COARSE.toUser(fixture.page6.flatSArm) + 300);
    (void)remoteClutchCommandCore(TS_CMD_REMOTE_CLUTCH_HOLD, fixture.now);
    fixture.update();
    fixture.current.setRpm(fixture.current.launchStatus.clutchEngagedRPM + 50);
    fixture.update();
    TEST_ASSERT_TRUE(fixture.current.launchStatus.flatShiftingHard);
    TEST_ASSERT_FALSE(fixture.current.launchStatus.launchingHard);
    releaseRemote();
}

static void test_noEffectWithLaunchAndFlatShiftOff(void)
{
    launch_fixture fixture;
    fixture.page6.launchEnabled = false;
    fixture.page6.flatSEnable = false;
    fixture.init();
    fixture.setClutch(false);
    (void)remoteClutchCommandCore(TS_CMD_REMOTE_CLUTCH_HOLD, fixture.now);
    fixture.update();
    TEST_ASSERT_FALSE(fixture.current.launchStatus.clutchTrigger);
    releaseRemote();
}

static void test_initFreesIt(void)
{
    launch_fixture fixture;
    (void)remoteClutchCommandCore(TS_CMD_REMOTE_CLUTCH_HOLD, fixture.now);
    fixture.init();
    TEST_ASSERT_FALSE(remoteClutchHeld);
}

static void test_millisWrap(void)
{
    launch_fixture fixture;
    fixture.init();
    fixture.setClutch(false);
    fixture.now = 0xFFFFFF00U;
    (void)remoteClutchCommandCore(TS_CMD_REMOTE_CLUTCH_HOLD, fixture.now);
    fixture.now += 300U; // wraps past 0
    fixture.update();
    TEST_ASSERT_TRUE(fixture.current.launchStatus.clutchTrigger);
    fixture.now += 100U;
    fixture.update();
    TEST_ASSERT_FALSE(fixture.current.launchStatus.clutchTrigger);
}

void testRemoteClutch(void)
{
    SET_UNITY_FILENAME() {
        RUN_TEST_P(test_commandGroup);
        RUN_TEST_P(test_engagesClutchLikeThePin);
        RUN_TEST_P(test_expiresWithoutRenewal);
        RUN_TEST_P(test_renewalKeepsItEngaged);
        RUN_TEST_P(test_pinOrRemote);
        RUN_TEST_P(test_flatShiftAboveTheSwitchRpm);
        RUN_TEST_P(test_noEffectWithLaunchAndFlatShiftOff);
        RUN_TEST_P(test_initFreesIt);
        RUN_TEST_P(test_millisWrap);
    }
}

// Fork: a Valet level can block launch and flat shift (valetFlags), and the clutch pin alone is exposed for the
// transmitter's start interlock (the remote clutch doesn't count there).
#include "src/controllers/valet/valetController_details.h"
extern valet::details::state_t valetState;

static void test_valetBlocksLaunchAndFlatShift(void)
{
    launch_fixture fixture;
    fixture.init();
    valetState.level = valetState.inForce = 2U;
    fixture.page15.valetFlags[1] = 0x00U;
    fixture.armHardLaunch();
    fixture.update();
    TEST_ASSERT_FALSE(fixture.current.launchStatus.launchingHard);
    fixture.page15.valetFlags[1] = 0x01U; // launch allowed in this level
    fixture.update();
    TEST_ASSERT_TRUE(fixture.current.launchStatus.launchingHard);

    fixture.setClutch(false);
    fixture.update();
    fixture.page15.valetFlags[1] = 0x00U;
    fixture.armFlatShift();
    fixture.current.setRpm(RPM_COARSE.toUser(fixture.page6.flatSArm) + 3);
    fixture.update();
    fixture.current.setRpm(fixture.current.launchStatus.clutchEngagedRPM + 200);
    fixture.update();
    TEST_ASSERT_FALSE(fixture.current.launchStatus.flatShiftingHard);
    fixture.page15.valetFlags[1] = 0x02U; // flat shift allowed
    fixture.update();
    TEST_ASSERT_TRUE(fixture.current.launchStatus.flatShiftingHard);
    valetState = valet::details::state_t();
}

static void test_clutchPinAlone(void)
{
    launch_fixture fixture;
    fixture.init();
    fixture.setClutch(true);
    fixture.update();
    TEST_ASSERT_TRUE(isClutchPinPressed());
    fixture.setClutch(false);
    (void)remoteClutchCommandCore(TS_CMD_REMOTE_CLUTCH_HOLD, fixture.now);
    fixture.update();
    TEST_ASSERT_TRUE(fixture.current.launchStatus.clutchTrigger); // the remote clutch engages launch / flat shift...
    TEST_ASSERT_FALSE(isClutchPinPressed());                       // ...but isn't the pin
    releaseRemote();
}

void testValetLaunch(void)
{
    SET_UNITY_FILENAME() {
        RUN_TEST_P(test_valetBlocksLaunchAndFlatShift);
        RUN_TEST_P(test_clutchPinAlone);
    }
}
