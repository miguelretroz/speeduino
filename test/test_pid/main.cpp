#include "../test_harness_device.h"
#include "../test_harness_native.h"

void runAllPIDTests(void)
{
    extern void testPID(void);
    extern void testIntegerPID(void);
    extern void testIntegerPID_ideal(void);
    extern void testPidTuningParameters(void);
    extern void testPidCorePreset(void);

    testPidTuningParameters();
    testPID();
    testIntegerPID();
    testIntegerPID_ideal();
    testPidCorePreset();
}

TEST_HARNESS(runAllPIDTests)
