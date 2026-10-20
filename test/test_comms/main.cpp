#include "../test_harness_device.h"
#include "../test_harness_native.h"

void runAllTests(void)
{
    extern void testSecondaryTunerStudioLockout(void);

    testSecondaryTunerStudioLockout();
}

TEST_HARNESS(runAllTests)
