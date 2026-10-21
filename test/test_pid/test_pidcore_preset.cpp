#include <unity.h>
#include "src/PID/integerPID.h"
#include "src/PID/PID.h"
#include "../test_utils.h"

// Fork specific: presetting the output and changing the gains without a jump, used by the PID auto tune

static constexpr uint32_t NOW = 10000UL;
static constexpr uint16_t SAMPLE_TIME = 250U;

static void setup_integerPID(integerPID &pid, const PidTuningParameters &params, int32_t input)
{
    pid.setOutputLimits(0, 2000);
    pid.setTunings(params, NOW, SAMPLE_TIME);
    pid.setSetPoint(1000);
    pid.activate(input);
}

static void test_integerPID_presetOutput_next_compute_matches(void)
{
    integerPID pid;
    setup_integerPID(pid, PidTuningParameters(3, 2, 1), 900);

    pid.presetOutput(500, 900);

    int32_t output = 0;
    TEST_ASSERT_TRUE(pid.compute(NOW, 900, &output));
    TEST_ASSERT_EQUAL(500, output);
}

static void test_integerPID_presetOutput_with_feedforward(void)
{
    integerPID pid;
    setup_integerPID(pid, PidTuningParameters(3, 2, 1), 900);
    pid.setFeedForwardTerm(300);

    pid.presetOutput(500, 900);

    int32_t output = 0;
    TEST_ASSERT_TRUE(pid.compute(NOW, 900, &output));
    TEST_ASSERT_EQUAL(500, output);
}

static void test_integerPID_presetOutput_ki_zero_resets_integral(void)
{
    integerPID pid;
    setup_integerPID(pid, PidTuningParameters(3, 0, 0), 900);

    pid.presetOutput(500, 900);

    // Without an integral term only the P term is left: (3*32)*100 >> 10
    int32_t output = 0;
    TEST_ASSERT_TRUE(pid.compute(NOW, 900, &output));
    TEST_ASSERT_EQUAL(9, output);
}

static void test_integerPID_presetOutput_holds_while_on_target(void)
{
    integerPID pid;
    setup_integerPID(pid, PidTuningParameters(3, 2, 1), 1000);

    pid.presetOutput(744, 1000);

    // No error: the output stays where it was preset
    int32_t output = 0;
    for (uint8_t step = 0U; step < 5U; ++step)
    {
        TEST_ASSERT_TRUE(pid.compute(NOW + (step * SAMPLE_TIME), 1000, &output));
        TEST_ASSERT_EQUAL(744, output);
    }
}

static void test_PID_presetOutput(void)
{
    // Same setup as the AFR closed loop: negative gains, output limited to +/- the authority
    PID pid;
    pid.setOutputLimits(-10, 10);
    pid.setTunings(PidTuningParameters(50, 20, 5) * -1);
    pid.setSetPoint(147);
    pid.resetIntegral(150);

    pid.presetOutput(4, 150);
    TEST_ASSERT_EQUAL(4, pid.compute(150));

    pid.presetOutput(-6, 150);
    TEST_ASSERT_EQUAL(-6, pid.compute(150));

    pid.presetOutput(0, 150);
    TEST_ASSERT_EQUAL(0, pid.compute(150));
}

// Build up the integral with a constant error, then return to the setpoint
static int32_t build_integral(integerPID &pid, uint32_t &now)
{
    int32_t output = 0;
    for (uint8_t step = 0U; step < 10U; ++step)
    {
        TEST_ASSERT_TRUE(pid.compute(now, 900, &output));
        now += SAMPLE_TIME;
    }
    TEST_ASSERT_TRUE(pid.compute(now, 1000, &output));
    now += SAMPLE_TIME;
    return output;
}

static void test_integerPID_bumpless_retune_keeps_output(void)
{
    integerPID pid;
    setup_integerPID(pid, PidTuningParameters(3, 2, 0), 900);
    pid.setOutputLimits(0, 100000);
    uint32_t now = NOW;
    int32_t before = build_integral(pid, now);
    TEST_ASSERT_EQUAL(15, before); // 16 * 1000 >> 10

    pid.setTunings(PidTuningParameters(3, 4, 0), now, SAMPLE_TIME, true);

    int32_t after = 0;
    TEST_ASSERT_TRUE(pid.compute(now, 1000, &after));
    TEST_ASSERT_EQUAL(before, after);
}

static void test_integerPID_retune_without_bumpless_jumps(void)
{
    integerPID pid;
    setup_integerPID(pid, PidTuningParameters(3, 2, 0), 900);
    pid.setOutputLimits(0, 100000);
    uint32_t now = NOW;
    int32_t before = build_integral(pid, now);

    pid.setTunings(PidTuningParameters(3, 4, 0), now, SAMPLE_TIME);

    int32_t after = 0;
    TEST_ASSERT_TRUE(pid.compute(now, 1000, &after));
    TEST_ASSERT_EQUAL(31, after); // Doubling Ki doubles the integral term
    TEST_ASSERT_GREATER_THAN(before, after);
}

static void test_pidcore_rescale_integral_zero_ki(void)
{
    PidCore core;
    core.setOutputLimits(-100000, 100000);
    core.setTunings(PidTuningParameters(1, 5, 0));
    (void)core.compute(0, 10, 0);
    (void)core.compute(0, 10, 0);
    TEST_ASSERT_EQUAL(20, core._integralTerm);

    // The integral will have no effect: drop it
    core.setTunings(PidTuningParameters(1, 0, 0), true);
    TEST_ASSERT_EQUAL(0, core._integralTerm);

    // The integral had no effect: start from zero
    core._integralTerm = 20;
    core.setTunings(PidTuningParameters(1, 5, 0), true);
    TEST_ASSERT_EQUAL(0, core._integralTerm);
}

static void test_pidcore_rescale_integral_sign_and_rounding(void)
{
    PidCore core;
    core.setTunings(PidTuningParameters(1, 3, 0));
    core._integralTerm = -10;

    core.setTunings(PidTuningParameters(1, 4, 0), true);
    TEST_ASSERT_EQUAL(-8, core._integralTerm); // -30/4 = -7.5, rounded away from zero

    // A direction change keeps the integral term (Ki * integral)
    core.setTunings(PidTuningParameters(1, 4, 0) * -1, true);
    TEST_ASSERT_EQUAL(8, core._integralTerm);
}

static void test_pidcore_rescale_integral_no_overflow(void)
{
    PidCore core;
    core.setTunings(PidTuningParameters(1, 100, 0));
    core._integralTerm = INT32_MAX / 2;

    core.setTunings(PidTuningParameters(1, 200, 0), true);

    // Limited to (INT32_MAX/2)/100 before scaling, so no overflow
    TEST_ASSERT_EQUAL(5368709L, core._integralTerm);

    // Same on the negative side: limited to -(INT32_MAX/2)/200, then scaled by 200/100
    core._integralTerm = INT32_MIN / 2;
    core.setTunings(PidTuningParameters(1, 100, 0), true);
    TEST_ASSERT_EQUAL(-10737418L, core._integralTerm);
}

void testPidCorePreset(void)
{
  SET_UNITY_FILENAME() {
    RUN_TEST_P(test_integerPID_presetOutput_next_compute_matches);
    RUN_TEST_P(test_integerPID_presetOutput_with_feedforward);
    RUN_TEST_P(test_integerPID_presetOutput_ki_zero_resets_integral);
    RUN_TEST_P(test_integerPID_presetOutput_holds_while_on_target);
    RUN_TEST_P(test_PID_presetOutput);
    RUN_TEST_P(test_integerPID_bumpless_retune_keeps_output);
    RUN_TEST_P(test_integerPID_retune_without_bumpless_jumps);
    RUN_TEST_P(test_pidcore_rescale_integral_zero_ki);
    RUN_TEST_P(test_pidcore_rescale_integral_sign_and_rounding);
    RUN_TEST_P(test_pidcore_rescale_integral_no_overflow);
  }
}
