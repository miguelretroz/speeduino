#include "../test_harness_device.h"
#include "../test_harness_native.h"

void runAllTests(void)
{
    extern void testProtectionResolve(void);
    extern void testProtectionConditions(void);
    extern void testProtectionLog(void);
    extern void testProtectionOutput(void);
    extern void testProtectionConfig(void);

    testProtectionResolve();
    testProtectionConditions();
    testProtectionLog();
    testProtectionOutput();
    testProtectionConfig();
}

TEST_HARNESS(runAllTests)
