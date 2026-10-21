#include "../test_harness_device.h"
#include "../test_harness_native.h"

void runAllTests(void)
{
    extern void testRelayTuner(void);
    extern void testPidTuneRules(void);
    extern void testPidTuneConfig(void);
    extern void testPidTuneController(void);

    testRelayTuner();
    testPidTuneRules();
    testPidTuneConfig();
    testPidTuneController();
}

TEST_HARNESS(runAllTests)
