#include "../test_harness_device.h"
#include "../test_harness_native.h"

void runAllTests(void)
{
    extern void testColdStartConfig(void);
    extern void testColdStartPrime(void);
    extern void testColdStartCranking(void);
    extern void testColdStartAttempts(void);
    extern void testColdStartLog(void);
    extern void testColdStartBattery(void);

    testColdStartConfig();
    testColdStartPrime();
    testColdStartCranking();
    testColdStartAttempts();
    testColdStartLog();
    testColdStartBattery();
}

TEST_HARNESS(runAllTests)
