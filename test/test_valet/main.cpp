#include "../test_harness_device.h"
#include "../test_harness_native.h"

void runAllTests(void)
{
    extern void testValet(void);

    testValet();
}

TEST_HARNESS(runAllTests)
