#include <unity.h>
#include "../test_utils.h"
#include "sensorfault_fixture.h"

static void test_out_of_range_limits(void)
{
    // Limits in ADC/4 units: 2 = readings 0-7, 253 = readings 1012-1015 are still fine
    TEST_ASSERT_TRUE(isOutOfRange(0U, 2U, 253U));
    TEST_ASSERT_TRUE(isOutOfRange(7U, 2U, 253U));
    TEST_ASSERT_FALSE(isOutOfRange(8U, 2U, 253U));
    TEST_ASSERT_FALSE(isOutOfRange(1015U, 2U, 253U));
    TEST_ASSERT_TRUE(isOutOfRange(1016U, 2U, 253U));
    TEST_ASSERT_TRUE(isOutOfRange(1023U, 2U, 253U));
    // Wider MAP margins
    TEST_ASSERT_TRUE(isOutOfRange(47U, 12U, 250U));
    TEST_ASSERT_FALSE(isOutOfRange(48U, 12U, 250U));
    TEST_ASSERT_FALSE(isOutOfRange(1003U, 12U, 250U));
    TEST_ASSERT_TRUE(isOutOfRange(1004U, 12U, 250U));
}

static void test_no_fault_in_range(void)
{
    sensorfault_context_t context;

    context.ticks(50U);

    TEST_ASSERT_EQUAL_HEX8(0U, context.current.sensorFaults);
}

// The reading must stay at the limit for the delay (1S = 10 checks)
static void test_coolant_short_faults_after_delay(void)
{
    sensorfault_context_t context;
    context.current.cltADC = ADC_SHORT;

    context.ticks(9U);
    TEST_ASSERT_FALSE(context.isFaulted(SENSOR_FAULT_CLT));
    context.tick();
    TEST_ASSERT_TRUE(context.isFaulted(SENSOR_FAULT_CLT));
    // Only the coolant
    TEST_ASSERT_EQUAL_HEX8(1U << SENSOR_FAULT_CLT, context.current.sensorFaults);
}

static void test_coolant_open_faults_after_delay(void)
{
    sensorfault_context_t context;
    context.current.cltADC = ADC_OPEN;

    context.ticks(10U);

    TEST_ASSERT_TRUE(context.isFaulted(SENSOR_FAULT_CLT));
}

// A short glitch (shorter than the delay) is ignored
static void test_glitch_restarts_the_count(void)
{
    sensorfault_context_t context;

    context.current.cltADC = ADC_SHORT;
    context.ticks(9U);
    context.current.cltADC = ADC_MID;
    context.tick();
    context.current.cltADC = ADC_SHORT;
    context.ticks(9U);

    TEST_ASSERT_FALSE(context.isFaulted(SENSOR_FAULT_CLT));
    context.tick();
    TEST_ASSERT_TRUE(context.isFaulted(SENSOR_FAULT_CLT));
}

// The fault clears after the same time back in range
static void test_fault_clears_after_delay_in_range(void)
{
    sensorfault_context_t context;
    context.current.cltADC = ADC_SHORT;
    context.ticks(10U);
    TEST_ASSERT_TRUE(context.isFaulted(SENSOR_FAULT_CLT));

    context.current.cltADC = ADC_MID;
    context.ticks(9U);
    TEST_ASSERT_TRUE(context.isFaulted(SENSOR_FAULT_CLT));
    context.tick();
    TEST_ASSERT_FALSE(context.isFaulted(SENSOR_FAULT_CLT));
}

static void test_zero_delay_faults_on_first_check(void)
{
    sensorfault_context_t context;
    context.page15.sensorFaultDelay = 0U;
    context.current.iatADC = ADC_OPEN;

    context.tick();
    TEST_ASSERT_TRUE(context.isFaulted(SENSOR_FAULT_IAT));

    context.current.iatADC = ADC_MID;
    context.tick();
    TEST_ASSERT_FALSE(context.isFaulted(SENSOR_FAULT_IAT));
}

// The counters saturate instead of wrapping around, so a long fault stays a fault
static void test_long_fault_stays_faulted(void)
{
    sensorfault_context_t context;
    context.current.iatADC = ADC_SHORT;

    for (uint16_t i = 0U; i < 600U; ++i) { context.tick(); }

    TEST_ASSERT_TRUE(context.isFaulted(SENSOR_FAULT_IAT));
    TEST_ASSERT_EQUAL_UINT8(UINT8_MAX, sensorFaultState.outOfRange[SENSOR_FAULT_IAT]);
}

static void test_map_uses_the_raw_sample(void)
{
    sensorfault_context_t context;

    // Below 0.23V: a MAP sensor never gets there
    sensorFaultMapSample(40U);
    context.ticks(10U);
    TEST_ASSERT_TRUE(context.isFaulted(SENSOR_FAULT_MAP));

    sensorFaultMapSample(300U);
    context.ticks(10U);
    TEST_ASSERT_FALSE(context.isFaulted(SENSOR_FAULT_MAP));

    sensorFaultMapSample(1010U);
    context.ticks(10U);
    TEST_ASSERT_TRUE(context.isFaulted(SENSOR_FAULT_MAP));
}

// Without a MAP sample there is nothing to judge
static void test_map_needs_a_sample(void)
{
    sensorfault_context_t context;
    sensorFaultState.hasMapSample = false;
    sensorFaultState.mapAdc = ADC_SHORT;

    context.ticks(20U);

    TEST_ASSERT_FALSE(context.isFaulted(SENSOR_FAULT_MAP));
}

static void test_tps_uses_the_8_bit_reading(void)
{
    sensorfault_context_t context;

    context.current.tpsADC = 1U; // Below 2
    context.ticks(10U);
    TEST_ASSERT_TRUE(context.isFaulted(SENSOR_FAULT_TPS));

    context.current.tpsADC = 2U;
    context.ticks(10U);
    TEST_ASSERT_FALSE(context.isFaulted(SENSOR_FAULT_TPS));

    context.current.tpsADC = 254U; // Above 253
    context.ticks(10U);
    TEST_ASSERT_TRUE(context.isFaulted(SENSOR_FAULT_TPS));
}

static void test_each_sensor_has_its_own_fault(void)
{
    sensorfault_context_t context;
    context.current.cltADC = ADC_SHORT;
    context.current.iatADC = ADC_OPEN;
    sensorFaultMapSample(ADC_OPEN);
    context.current.tpsADC = 0U;

    context.ticks(10U);

    TEST_ASSERT_EQUAL_HEX8(0x0FU, context.current.sensorFaults);
}

static void test_disabled_sensor_never_faults(void)
{
    sensorfault_context_t context;
    context.page15.sensorFaultClt = 0U;
    context.current.cltADC = ADC_SHORT;

    context.ticks(20U);

    TEST_ASSERT_FALSE(context.isFaulted(SENSOR_FAULT_CLT));
}

// Turning the detection off clears a fault
static void test_disabling_clears_the_fault(void)
{
    sensorfault_context_t context;
    context.current.cltADC = ADC_SHORT;
    context.ticks(10U);
    TEST_ASSERT_TRUE(context.isFaulted(SENSOR_FAULT_CLT));

    context.page15.sensorFaultClt = 0U;
    context.tick();

    TEST_ASSERT_FALSE(context.isFaulted(SENSOR_FAULT_CLT));
}

// A sensor broken at power up is faulted right away, before the engine cranks
static void test_initialise_faults_at_once(void)
{
    sensorfault_context_t context;
    context.current.cltADC = ADC_OPEN;
    context.current.coolant = -40;
    context.current.tpsADC = 0U;
    context.current.TPS = 0U;
    context.current.TPSlast = 60U;

    initialiseSensorFaults(context.current, context.page15);

    TEST_ASSERT_TRUE(context.isFaulted(SENSOR_FAULT_CLT));
    TEST_ASSERT_TRUE(context.isFaulted(SENSOR_FAULT_TPS));
    TEST_ASSERT_FALSE(context.isFaulted(SENSOR_FAULT_IAT));
    TEST_ASSERT_FALSE(context.isFaulted(SENSOR_FAULT_MAP));
    // The fallback values replace the readings taken before the check
    TEST_ASSERT_EQUAL_INT16(60, context.current.coolant);
    TEST_ASSERT_EQUAL_UINT8(40U, context.current.TPS);
    TEST_ASSERT_EQUAL_UINT8(40U, context.current.TPSlast);
    TEST_ASSERT_EQUAL_INT16(25, context.current.IAT);
    TEST_ASSERT_EQUAL_UINT16(40U, context.current.MAP);
}

// The MAP sample taken while the baro was read survives the initialisation
static void test_initialise_keeps_the_map_sample(void)
{
    sensorfault_context_t context;
    sensorFaultMapSample(ADC_SHORT);

    initialiseSensorFaults(context.current, context.page15);

    TEST_ASSERT_TRUE(context.isFaulted(SENSOR_FAULT_MAP));
    TEST_ASSERT_EQUAL_UINT16(100U, context.current.MAP);
}

static void test_initialise_clears_old_faults(void)
{
    sensorfault_context_t context;
    context.current.cltADC = ADC_SHORT;
    context.ticks(10U);
    TEST_ASSERT_TRUE(context.isFaulted(SENSOR_FAULT_CLT));

    context.current.cltADC = ADC_MID;
    initialiseSensorFaults(context.current, context.page15);

    TEST_ASSERT_EQUAL_HEX8(0U, context.current.sensorFaults);
    TEST_ASSERT_EQUAL_UINT8(0U, sensorFaultState.outOfRange[SENSOR_FAULT_CLT]);
}

void testSensorFaultDetect(void)
{
    SET_UNITY_FILENAME() {
        RUN_TEST_P(test_out_of_range_limits);
        RUN_TEST_P(test_no_fault_in_range);
        RUN_TEST_P(test_coolant_short_faults_after_delay);
        RUN_TEST_P(test_coolant_open_faults_after_delay);
        RUN_TEST_P(test_glitch_restarts_the_count);
        RUN_TEST_P(test_fault_clears_after_delay_in_range);
        RUN_TEST_P(test_zero_delay_faults_on_first_check);
        RUN_TEST_P(test_long_fault_stays_faulted);
        RUN_TEST_P(test_map_uses_the_raw_sample);
        RUN_TEST_P(test_map_needs_a_sample);
        RUN_TEST_P(test_tps_uses_the_8_bit_reading);
        RUN_TEST_P(test_each_sensor_has_its_own_fault);
        RUN_TEST_P(test_disabled_sensor_never_faults);
        RUN_TEST_P(test_disabling_clears_the_fault);
        RUN_TEST_P(test_initialise_faults_at_once);
        RUN_TEST_P(test_initialise_keeps_the_map_sample);
        RUN_TEST_P(test_initialise_clears_old_faults);
    }
}
