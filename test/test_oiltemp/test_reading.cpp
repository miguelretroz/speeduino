#include "../test_utils.h"
#include "oiltemp_fixture.h"
#include "logger.h"

// Default calibration: Bosch preset, 2490 ohm pull-up. 20C = 2500 ohm = ADC 513. 110C = 144 ohm = ADC 56.

static void test_converts_with_default_calibration(void)
{
    auto context = setup_oiltemp();
    context.read(513U);
    TEST_ASSERT_INT_WITHIN(1, 20, oilTempState.temperature);

    auto hotContext = setup_oiltemp();
    hotContext.read(56U);
    // The 32 point table is coarse at the hot end (33 ADC counts = ~30C there)
    TEST_ASSERT_INT_WITHIN(3, 110, oilTempState.temperature);
}

static void test_first_reading_not_filtered(void)
{
    auto context = setup_oiltemp();
    context.read(513U);
    TEST_ASSERT_EQUAL(513U, oilTempState.adc);
}

static void test_readings_filtered(void)
{
    auto context = setup_oiltemp();
    context.read(500U);
    context.read(900U);
    TEST_ASSERT_GREATER_THAN(500U, oilTempState.adc);
    TEST_ASSERT_LESS_THAN(900U, oilTempState.adc);
}

static void test_open_circuit_is_fault(void)
{
    auto context = setup_oiltemp();
    context.read(1023U);
    TEST_ASSERT_TRUE(oilTempState.isFault);
    TEST_ASSERT_EQUAL_HEX8(0x02U, getOilTempLogStatus());
}

static void test_short_circuit_is_fault(void)
{
    auto context = setup_oiltemp();
    context.read(0U);
    TEST_ASSERT_TRUE(oilTempState.isFault);
}

static void test_normal_reading_no_fault(void)
{
    auto context = setup_oiltemp();
    context.read(ADC_MID);
    TEST_ASSERT_FALSE(oilTempState.isFault);
    TEST_ASSERT_EQUAL_HEX8(0x00U, getOilTempLogStatus());
}

static void test_disabled_does_nothing(void)
{
    auto context = setup_oiltemp();
    context.page15.oilTempEnable = 0U;
    context.initialise();
    context.read(ADC_MID);
    TEST_ASSERT_FALSE(oilTempState.hasReading);
    TEST_ASSERT_EQUAL(0U, getOilTempLogValue());
    TEST_ASSERT_EQUAL(0U, getOilTempLogStatus());
}

static void test_no_input_pin_does_nothing(void)
{
    auto context = setup_oiltemp();
    context.pins.pinOilTemp = NOT_A_PIN;
    context.initialise();
    context.read(ADC_MID);
    TEST_ASSERT_FALSE(oilTempState.hasReading);
}

static void test_live_data(void)
{
    auto context = setup_oiltemp();
    setFlatCalibration(90);
    context.read(ADC_MID);
    TEST_ASSERT_EQUAL(130U, getOilTempLogValue()); // +40 offset
    TEST_ASSERT_EQUAL(130U, getTSLogEntry(140U));
    TEST_ASSERT_EQUAL(0U, getTSLogEntry(141U));

    setFlatCalibration(140);
    context.read(ADC_MID);
    TEST_ASSERT_EQUAL(180U, getTSLogEntry(140U));
    TEST_ASSERT_EQUAL(1U, getTSLogEntry(141U)); // Hot
}

static void test_input_pin_from_tune(void)
{
#if defined(CORE_AVR)
    TEST_ASSERT_EQUAL(A0, getOilTempInputPin(0U));
    TEST_ASSERT_EQUAL(A9, getOilTempInputPin(9U));
    // A15 is outside the upstream ANALOG_PINS table, but it's on the proto area of older v0.4 boards
    TEST_ASSERT_EQUAL(A15, getOilTempInputPin(15U));
    TEST_ASSERT_EQUAL(NOT_A_PIN, getOilTempInputPin(16U));
#else
    TEST_IGNORE_MESSAGE("AVR only");
#endif
}

// A9 is on J5 of the v0.4.3d
static void test_default_input_is_a9(void)
{
    config15 page15 = {};
    setDefaults(page15);
    TEST_ASSERT_EQUAL(9U, page15.oilTempPin);
}

void testOilTempReading(void)
{
    SET_UNITY_FILENAME() {
        RUN_TEST_P(test_input_pin_from_tune);
        RUN_TEST_P(test_default_input_is_a9);
        RUN_TEST_P(test_converts_with_default_calibration);
        RUN_TEST_P(test_first_reading_not_filtered);
        RUN_TEST_P(test_readings_filtered);
        RUN_TEST_P(test_open_circuit_is_fault);
        RUN_TEST_P(test_short_circuit_is_fault);
        RUN_TEST_P(test_normal_reading_no_fault);
        RUN_TEST_P(test_disabled_does_nothing);
        RUN_TEST_P(test_no_input_pin_does_nothing);
        RUN_TEST_P(test_live_data);
    }
}
