#include <unity.h>
#include "../test_utils.h"
#include "protection_fixture.h"

static bool hasCondition(uint8_t condition)
{
    return BIT_CHECK(protectionState.forkConditions, condition);
}

static void test_coolant_hot_with_hysteresis(void)
{
    protection_context_t context;
    context.page15.protCoolantHotEnable = 1U;
    context.page15.protCoolantHot = temperatureAddOffset(110);

    context.current.coolant = 109;
    context.tick();
    TEST_ASSERT_FALSE(hasCondition(PROTECT_COOLANT_HOT));

    context.current.coolant = 110;
    context.tick();
    TEST_ASSERT_TRUE(hasCondition(PROTECT_COOLANT_HOT));

    // Stays hot in the band
    context.current.coolant = 106;
    context.tick();
    TEST_ASSERT_TRUE(hasCondition(PROTECT_COOLANT_HOT));

    // Clears 5C below
    context.current.coolant = 105;
    context.tick();
    TEST_ASSERT_FALSE(hasCondition(PROTECT_COOLANT_HOT));

    // Stays clear in the band
    context.current.coolant = 109;
    context.tick();
    TEST_ASSERT_FALSE(hasCondition(PROTECT_COOLANT_HOT));
}

static void test_coolant_hot_disabled(void)
{
    protection_context_t context;
    context.current.coolant = 150;

    context.tick();

    TEST_ASSERT_FALSE(hasCondition(PROTECT_COOLANT_HOT));
}

static void test_oil_hot(void)
{
    protection_context_t context;
    context.oilHot = true;

    context.tick();
    TEST_ASSERT_FALSE(hasCondition(PROTECT_OIL_HOT)); // Off by default

    context.page15.protOilHotEnable = 1U;
    context.tick();
    TEST_ASSERT_TRUE(hasCondition(PROTECT_OIL_HOT));

    context.oilHot = false;
    context.tick();
    TEST_ASSERT_FALSE(hasCondition(PROTECT_OIL_HOT));
}

static void test_fuel_pressure_differential(void)
{
    protection_context_t context;

    // No boost: the rail pressure
    TEST_ASSERT_EQUAL_INT16(43, fuelPressureDifferential(context.current));

    // 100kPa of boost is 14.5 PSI less across the injectors
    context.current.MAP = 200U;
    TEST_ASSERT_EQUAL_INT16(29, fuelPressureDifferential(context.current));

    // Vacuum adds to it (60kPa = 8.7 PSI)
    context.current.MAP = 40U;
    TEST_ASSERT_EQUAL_INT16(51, fuelPressureDifferential(context.current));

    // No fuel at all under boost: negative
    context.current.MAP = 200U;
    context.current.fuelPressure = 0U;
    TEST_ASSERT_EQUAL_INT16(-14, fuelPressureDifferential(context.current));
}

// The pressure must stay low for the delay (1S = 10 checks)
static void test_fuel_pressure_low_after_delay(void)
{
    protection_context_t context;
    context.page15.protFuelPressEnable = 1U;
    context.current.fuelPressure = 25U; // Below 30

    context.ticks(9U);
    TEST_ASSERT_FALSE(hasCondition(PROTECT_FUEL_PRESSURE));
    context.tick();
    TEST_ASSERT_TRUE(hasCondition(PROTECT_FUEL_PRESSURE));

    // Back to normal at once
    context.current.fuelPressure = 43U;
    context.tick();
    TEST_ASSERT_FALSE(hasCondition(PROTECT_FUEL_PRESSURE));
    TEST_ASSERT_EQUAL_UINT8(0U, protectionState.fuelPressureLowTicks);
}

// Boost brings the pressure across the injectors below the minimum even with the rail pressure fine
static void test_fuel_pressure_low_under_boost(void)
{
    protection_context_t context;
    context.page15.protFuelPressEnable = 1U;
    context.current.fuelPressure = 43U;
    context.current.MAP = 200U; // 43 - 14 = 29 < 30

    context.ticks(10U);

    TEST_ASSERT_TRUE(hasCondition(PROTECT_FUEL_PRESSURE));
}

static void test_fuel_pressure_only_above_rpm(void)
{
    protection_context_t context;
    context.page15.protFuelPressEnable = 1U;
    context.current.fuelPressure = 0U;

    // Cranking: below 500 RPM
    context.current.setRpm(400U);
    context.ticks(20U);
    TEST_ASSERT_FALSE(hasCondition(PROTECT_FUEL_PRESSURE));
    TEST_ASSERT_EQUAL_UINT8(0U, protectionState.fuelPressureLowTicks);

    // Stopped, with the RPM gate at 0: still nothing
    context.page15.protFuelPressRpm = 0U;
    context.current.setRpm(0U);
    context.ticks(20U);
    TEST_ASSERT_FALSE(hasCondition(PROTECT_FUEL_PRESSURE));

    context.current.setRpm(500U);
    context.page15.protFuelPressRpm = 5U;
    context.ticks(10U);
    TEST_ASSERT_TRUE(hasCondition(PROTECT_FUEL_PRESSURE));
}

static void test_fuel_pressure_needs_the_sensor(void)
{
    protection_context_t context;
    context.page15.protFuelPressEnable = 1U;
    context.page10.fuelPressureEnable = 0U;
    context.current.fuelPressure = 0U;

    context.ticks(20U);

    TEST_ASSERT_FALSE(hasCondition(PROTECT_FUEL_PRESSURE));
}

static void test_fuel_pressure_disabled(void)
{
    protection_context_t context;
    context.current.fuelPressure = 0U;

    context.ticks(20U);

    TEST_ASSERT_FALSE(hasCondition(PROTECT_FUEL_PRESSURE));
}

static void test_fuel_pressure_zero_delay(void)
{
    protection_context_t context;
    context.page15.protFuelPressEnable = 1U;
    context.page15.protFuelPressDelay = 0U;
    context.current.fuelPressure = 10U;

    context.tick();

    TEST_ASSERT_TRUE(hasCondition(PROTECT_FUEL_PRESSURE));
}

static void test_sensor_fault(void)
{
    protection_context_t context;
    context.current.sensorFaults = 1U << SENSOR_FAULT_CLT;

    context.tick();
    TEST_ASSERT_FALSE(hasCondition(PROTECT_SENSOR_FAULT)); // Off by default

    context.page15.protSensorFaultEnable = 1U;
    context.tick();
    TEST_ASSERT_TRUE(hasCondition(PROTECT_SENSOR_FAULT));

    context.current.sensorFaults = 0U;
    context.tick();
    TEST_ASSERT_FALSE(hasCondition(PROTECT_SENSOR_FAULT));
}

static void test_all_fork_conditions_together(void)
{
    protection_context_t context;
    context.page15.protCoolantHotEnable = 1U;
    context.page15.protOilHotEnable = 1U;
    context.page15.protFuelPressEnable = 1U;
    context.page15.protFuelPressDelay = 0U;
    context.page15.protSensorFaultEnable = 1U;
    context.current.coolant = 120;
    context.oilHot = true;
    context.current.fuelPressure = 0U;
    context.current.sensorFaults = 1U << SENSOR_FAULT_TPS;

    context.tickAndResolve();

    TEST_ASSERT_EQUAL_HEX8(0x78U, protectionState.forkConditions);
    TEST_ASSERT_EQUAL_HEX8(0x78U, getProtectionLogReason());
}

void testProtectionConditions(void)
{
    SET_UNITY_FILENAME() {
        RUN_TEST_P(test_coolant_hot_with_hysteresis);
        RUN_TEST_P(test_coolant_hot_disabled);
        RUN_TEST_P(test_oil_hot);
        RUN_TEST_P(test_fuel_pressure_differential);
        RUN_TEST_P(test_fuel_pressure_low_after_delay);
        RUN_TEST_P(test_fuel_pressure_low_under_boost);
        RUN_TEST_P(test_fuel_pressure_only_above_rpm);
        RUN_TEST_P(test_fuel_pressure_needs_the_sensor);
        RUN_TEST_P(test_fuel_pressure_disabled);
        RUN_TEST_P(test_fuel_pressure_zero_delay);
        RUN_TEST_P(test_sensor_fault);
        RUN_TEST_P(test_all_fork_conditions_together);
    }
}
