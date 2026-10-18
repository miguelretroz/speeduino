#include "../test_utils.h"
#include "oiltemp_fixture.h"

// Defaults: alert at 130C, clears at 125C

static bool isHotAt(oiltemp_context_t &context, int16_t temperature)
{
    setFlatCalibration(temperature);
    context.read(ADC_MID);
    return oilTempState.isHot;
}

static void test_alert_threshold_and_hysteresis(void)
{
    auto context = setup_oiltemp();
    TEST_ASSERT_FALSE(isHotAt(context, 129));
    TEST_ASSERT_TRUE(isHotAt(context, 130));
    TEST_ASSERT_TRUE(isHotAt(context, 126)); // Inside the hysteresis band
    TEST_ASSERT_FALSE(isHotAt(context, 125));
    TEST_ASSERT_FALSE(isHotAt(context, 128)); // Has to reach 130 again
}

static void test_sensor_fault_never_alerts(void)
{
    auto context = setup_oiltemp();
    setFlatCalibration(150);
    context.read(1023U);
    TEST_ASSERT_TRUE(oilTempState.isFault);
    TEST_ASSERT_FALSE(oilTempState.isHot);
}

static void test_alert_output(void)
{
    auto context = setup_oiltemp();
    context.page15.oilTempAlertOutput = 1U;
    context.initialise();
    TEST_ASSERT_TRUE(oilTempState.alertPin.isValid());
    TEST_ASSERT_FALSE(context.isAlertPinHigh());

    (void)isHotAt(context, 140);
    TEST_ASSERT_TRUE(context.isAlertPinHigh());
    (void)isHotAt(context, 100);
    TEST_ASSERT_FALSE(context.isAlertPinHigh());
}

static void test_alert_output_inverted(void)
{
    auto context = setup_oiltemp();
    context.page15.oilTempAlertOutput = 1U;
    context.page15.oilTempAlertInv = 1U;
    context.initialise();
    TEST_ASSERT_TRUE(context.isAlertPinHigh()); // Off

    (void)isHotAt(context, 140);
    TEST_ASSERT_FALSE(context.isAlertPinHigh()); // On
}

static void test_indicator_without_alert_output(void)
{
    auto context = setup_oiltemp();
    TEST_ASSERT_FALSE(oilTempState.alertPin.isValid());
    TEST_ASSERT_TRUE(isHotAt(context, 140));
    TEST_ASSERT_EQUAL_HEX8(0x01U, getOilTempLogStatus());
}

void testOilTempAlert(void)
{
    SET_UNITY_FILENAME() {
        RUN_TEST_P(test_alert_threshold_and_hysteresis);
        RUN_TEST_P(test_sensor_fault_never_alerts);
        RUN_TEST_P(test_alert_output);
        RUN_TEST_P(test_alert_output_inverted);
        RUN_TEST_P(test_indicator_without_alert_output);
    }
}
