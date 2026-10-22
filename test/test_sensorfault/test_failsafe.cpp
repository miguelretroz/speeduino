#include <unity.h>
#include "../test_utils.h"
#include "sensorfault_fixture.h"
#include "logger.h"

static void test_fallbacks_pass_the_reading_without_a_fault(void)
{
    sensorfault_context_t context;

    TEST_ASSERT_EQUAL_INT16(-10, sensorFaultCoolant(-10, context.current, context.page15));
    TEST_ASSERT_EQUAL_INT16(55, sensorFaultIat(55, context.current, context.page15));
    TEST_ASSERT_EQUAL_UINT16(180U, sensorFaultMap(180U, context.current, context.page15));
    TEST_ASSERT_EQUAL_UINT8(150U, sensorFaultTps(150U, context.current, context.page15));
}

static void test_fallbacks_replace_a_faulted_reading(void)
{
    sensorfault_context_t context;
    context.current.sensorFaults = 0x0FU;
    context.page15.cltFailsafe = temperatureAddOffset(70);
    context.page15.iatFailsafe = temperatureAddOffset(35);
    context.page15.mapFailsafe = 90U;
    context.page15.tpsFailsafe = 25U;

    TEST_ASSERT_EQUAL_INT16(70, sensorFaultCoolant(-40, context.current, context.page15));
    TEST_ASSERT_EQUAL_INT16(35, sensorFaultIat(-40, context.current, context.page15));
    TEST_ASSERT_EQUAL_UINT16(90U, sensorFaultMap(0U, context.current, context.page15));
    // Whole percent in the tune, 0.5% units in the status
    TEST_ASSERT_EQUAL_UINT8(50U, sensorFaultTps(0U, context.current, context.page15));
}

static void test_tps_fallback_is_clamped_to_100_percent(void)
{
    sensorfault_context_t context;
    context.current.sensorFaults = 1U << SENSOR_FAULT_TPS;
    context.page15.tpsFailsafe = 255U;

    TEST_ASSERT_EQUAL_UINT8(200U, sensorFaultTps(0U, context.current, context.page15));
}

// Each fault only replaces its own sensor
static void test_fallbacks_are_per_sensor(void)
{
    sensorfault_context_t context;
    context.current.sensorFaults = 1U << SENSOR_FAULT_IAT;

    TEST_ASSERT_EQUAL_INT16(-10, sensorFaultCoolant(-10, context.current, context.page15));
    TEST_ASSERT_EQUAL_INT16(30, sensorFaultIat(-40, context.current, context.page15));
    TEST_ASSERT_EQUAL_UINT16(180U, sensorFaultMap(180U, context.current, context.page15));
    TEST_ASSERT_EQUAL_UINT8(150U, sensorFaultTps(150U, context.current, context.page15));
}

static void test_fan_forced_by_a_coolant_fault(void)
{
    sensorfault_context_t context;
    TEST_ASSERT_FALSE(sensorFaultForcesFan(context.current, context.page15));

    context.current.sensorFaults = 1U << SENSOR_FAULT_CLT;
    TEST_ASSERT_TRUE(sensorFaultForcesFan(context.current, context.page15));

    // Not by the other sensors
    context.current.sensorFaults = 0x0EU;
    TEST_ASSERT_FALSE(sensorFaultForcesFan(context.current, context.page15));

    // Not when the option is off
    context.current.sensorFaults = 1U << SENSOR_FAULT_CLT;
    context.page15.sensorFaultFan = 0U;
    TEST_ASSERT_FALSE(sensorFaultForcesFan(context.current, context.page15));
}

static void test_log_status_byte(void)
{
    sensorfault_context_t context;
    TEST_ASSERT_EQUAL_HEX8(0x00U, getSensorFaultLogStatus(context.current, context.page15));

    context.current.sensorFaults = (1U << SENSOR_FAULT_IAT) | (1U << SENSOR_FAULT_TPS);
    TEST_ASSERT_EQUAL_HEX8(0x0AU, getSensorFaultLogStatus(context.current, context.page15));

    // The coolant fault also shows the fan being forced on
    context.current.sensorFaults = 1U << SENSOR_FAULT_CLT;
    TEST_ASSERT_EQUAL_HEX8(0x11U, getSensorFaultLogStatus(context.current, context.page15));
    context.page15.sensorFaultFan = 0U;
    TEST_ASSERT_EQUAL_HEX8(0x01U, getSensorFaultLogStatus(context.current, context.page15));
}

// Live data byte 157 reads the global status and tune
static void test_live_data_byte(void)
{
    sensorFault::details::setDefaults(configPage15);
    currentStatus.sensorFaults = 0U;
    TEST_ASSERT_EQUAL_HEX8(0x00U, getTSLogEntry(157U));

    currentStatus.sensorFaults = (1U << SENSOR_FAULT_CLT) | (1U << SENSOR_FAULT_MAP);
    TEST_ASSERT_EQUAL_HEX8(0x15U, getTSLogEntry(157U));
    currentStatus.sensorFaults = 0U;
}

void testSensorFaultFailsafe(void)
{
    SET_UNITY_FILENAME() {
        RUN_TEST_P(test_fallbacks_pass_the_reading_without_a_fault);
        RUN_TEST_P(test_fallbacks_replace_a_faulted_reading);
        RUN_TEST_P(test_tps_fallback_is_clamped_to_100_percent);
        RUN_TEST_P(test_fallbacks_are_per_sensor);
        RUN_TEST_P(test_fan_forced_by_a_coolant_fault);
        RUN_TEST_P(test_log_status_byte);
        RUN_TEST_P(test_live_data_byte);
    }
}
