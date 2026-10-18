#include "../test_harness_device.h"
#include "../test_harness_native.h"

void runAllTests(void)
{
    extern void testOilTempReading(void);
    extern void testOilTempAlert(void);
    extern void testOilTempUpgrade(void);

    testOilTempReading();
    testOilTempAlert();
    testOilTempUpgrade();
}

TEST_HARNESS(runAllTests)
