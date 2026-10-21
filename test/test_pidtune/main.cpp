#include "../test_harness_device.h"
#include "../test_harness_native.h"

void runAllTests(void)
{
    extern void testRelayTuner(void);
    extern void testPidTuneRules(void);
    extern void testPidTuneConfig(void);
    extern void testPidTuneController(void);
    extern void testPidTuneLog(void);
    extern void testGainAdapter(void);
    extern void testPidTuneAdaptation(void);

    testRelayTuner();
    testPidTuneRules();
    testPidTuneConfig();
    testPidTuneController();
    testPidTuneLog();
    testGainAdapter();
    testPidTuneAdaptation();
}

TEST_HARNESS(runAllTests)
