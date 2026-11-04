#include "../test_harness_device.h"
#include "../test_harness_native.h"

void runAllTests(void)
{
    extern void testLaunchControl(void);
    extern void testInit(void);
    extern void testRemoteClutch(void);
    extern void testValetLaunch(void);

    testLaunchControl();
    testInit();
    testRemoteClutch();
    testValetLaunch();
}

TEST_HARNESS(runAllTests)
