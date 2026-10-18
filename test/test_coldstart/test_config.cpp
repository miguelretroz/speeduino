#include "../test_utils.h"
#include "coldstart_fixture.h"
#include "units.h"

static void test_defaults_applied_to_fresh_eeprom(void)
{
    resetFakeStorage();
    // Simulate never initialised EEPROM content
    configPage15.coldStartEnable = 1U;
    configPage15.coldStartPin = 63U;
    fill_curve(configPage15.coldStartPrimeValues, 0xFFU);
    configPage15.coldStartDosePct[3] = 0xFFU;

    initialiseColdStartConfig(configPage15);

    TEST_ASSERT_EQUAL(0U, configPage15.coldStartEnable);
    TEST_ASSERT_EQUAL(0U, configPage15.coldStartPin);
    TEST_ASSERT_EQUAL(0U, configPage15.coldStartPrimeValues[0]); // Prime is off by default
    TEST_ASSERT_EQUAL(0U, configPage15.coldStartAfterValues[0]); // After start is off by default
    TEST_ASSERT_EQUAL(temperatureAddOffset(40), configPage15.coldStartCrankBins[5]);
    TEST_ASSERT_EQUAL(0U, configPage15.coldStartCrankValues[5]); // Nothing above 40C
    TEST_ASSERT_EQUAL(100U, configPage15.coldStartDosePct[0]);
    TEST_ASSERT_EQUAL(60U, configPage15.coldStartDosePct[1]);
    TEST_ASSERT_EQUAL(30U, configPage15.coldStartDosePct[2]);
    TEST_ASSERT_EQUAL(0U, configPage15.coldStartDosePct[3]);
    TEST_ASSERT_EQUAL(100U, configPage15.coldStartMinBattery);
    TEST_ASSERT_EQUAL(0U, fakeStorage.attempts);
    TEST_ASSERT_NOT_EQUAL(0xFFU, fakeStorage.marker);
}

static void test_defaults_not_reapplied(void)
{
    resetFakeStorage();
    initialiseColdStartConfig(configPage15);

    configPage15.coldStartEnable = 1U;
    configPage15.coldStartDosePct[0] = 80U;
    fakeStorage.attempts = 2U;
    initialiseColdStartConfig(configPage15);

    TEST_ASSERT_EQUAL(1U, configPage15.coldStartEnable);
    TEST_ASSERT_EQUAL(80U, configPage15.coldStartDosePct[0]);
    TEST_ASSERT_EQUAL(2U, fakeStorage.attempts);
}

// The INI file addresses these fields by page offset. Page 15 starts with the 80 byte boost duty table.
static void test_ini_offsets(void)
{
    constexpr uint16_t CONFIG15_PAGE_OFFSET = 80U;
    const uint8_t *pBase = (const uint8_t*)&configPage15;

    TEST_ASSERT_EQUAL(256U - CONFIG15_PAGE_OFFSET, sizeof(config15));
    TEST_ASSERT_EQUAL(208U - CONFIG15_PAGE_OFFSET, (const uint8_t*)&configPage15.coldStartPrimeBins - pBase);
    TEST_ASSERT_EQUAL(226U - CONFIG15_PAGE_OFFSET, (const uint8_t*)&configPage15.coldStartCrankValues - pBase);
    TEST_ASSERT_EQUAL(244U - CONFIG15_PAGE_OFFSET, (const uint8_t*)&configPage15.coldStartPulsePeriod - pBase);
    TEST_ASSERT_EQUAL(249U - CONFIG15_PAGE_OFFSET, (const uint8_t*)&configPage15.coldStartDosePct - pBase);
    TEST_ASSERT_EQUAL(255U - CONFIG15_PAGE_OFFSET, (const uint8_t*)&configPage15.coldStartResetTime - pBase);
}

static void test_disabled_does_nothing(void)
{
    auto context = setup_coldstart();
    context.page15.coldStartEnable = 0U;
    fill_curve(context.page15.coldStartPrimeValues, 20U);
    context.initialise();

    context.keyOn();
    TEST_ASSERT_EQUAL(0U, context.runFor(3000U));
    context.crank();
    TEST_ASSERT_EQUAL(0U, context.runFor(3000U));
    TEST_ASSERT_FALSE(coldStartState.pin.isValid());
    TEST_ASSERT_EQUAL(0U, fakeStorage.attemptWrites);
}

static void test_output_polarity(void)
{
    auto context = setup_coldstart();
    TEST_ASSERT_FALSE(context.isPinHigh());

    context.page15.coldStartInv = 1U;
    context.initialise();
    TEST_ASSERT_TRUE(context.isPinHigh()); // Off

    context.keyOn();
    context.crank();
    context.run();
    TEST_ASSERT_TRUE(context.isOutputOn());
    TEST_ASSERT_FALSE(context.isPinHigh()); // On
}

void testColdStartConfig(void)
{
    SET_UNITY_FILENAME() {
        RUN_TEST_P(test_defaults_applied_to_fresh_eeprom);
        RUN_TEST_P(test_defaults_not_reapplied);
        RUN_TEST_P(test_ini_offsets);
        RUN_TEST_P(test_disabled_does_nothing);
        RUN_TEST_P(test_output_polarity);
    }
}
