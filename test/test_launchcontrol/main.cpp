#include "../test_harness_device.h"
#include "../test_harness_native.h"

void runAllTests(void)
{
    extern void testLaunchControl(void);
    extern void testInit(void);
    extern void testRemoteClutch(void);

    testLaunchControl();
    testInit();
    testRemoteClutch();
}

TEST_HARNESS(runAllTests)
