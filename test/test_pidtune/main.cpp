#include "../test_harness_device.h"
#include "../test_harness_native.h"

void runAllTests(void)
{
    extern void testRelayTuner(void);
    extern void testPidTuneRules(void);

    testRelayTuner();
    testPidTuneRules();
}

TEST_HARNESS(runAllTests)
