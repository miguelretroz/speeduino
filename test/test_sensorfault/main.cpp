#include "../test_harness_device.h"
#include "../test_harness_native.h"

void runAllTests(void)
{
    extern void testSensorFaultDetect(void);
    extern void testSensorFaultFailsafe(void);
    extern void testSensorFaultConfig(void);

    testSensorFaultDetect();
    testSensorFaultFailsafe();
    testSensorFaultConfig();
}

TEST_HARNESS(runAllTests)
