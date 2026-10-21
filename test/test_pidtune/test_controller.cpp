#include <unity.h>
#include "../test_utils.h"
#include "pidtune_fixture.h"
#include "src/controllers/tsCommand/tsCommandController.h"

using namespace pidTune;
using namespace pidTuneController::details;

static constexpr uint32_t STEP_MS = IDLE_SAMPLE_MS;

static void assert_failed(TuneFail reason)
{
    TEST_ASSERT_EQUAL(TunePhase::Failed, pidTuneState.tuner.getPhase());
    TEST_ASSERT_EQUAL(reason, pidTuneState.tuner.getFailure());
}

// ---------------------------------------------------------------- Idle

static void test_idle_start_refused_when_stopped(void)
{
    setup_idle_tune();
    currentStatus.rotationStatus = EngineRotationStatus::Stopped;

    TEST_ASSERT_TRUE(send_command(TS_CMD_PIDTUNE_IDLE_START, TEST_START_MS));

    assert_failed(TuneFail::NotReady);
    TEST_ASSERT_EQUAL(TuneLoop::Idle, pidTuneState.loop);
    TEST_ASSERT_FALSE(isPidTuneIdleRunning());
}

static void test_idle_start_refused_in_open_loop(void)
{
    setup_idle_tune();
    configPage6.iacAlgorithm = IAC_ALGORITHM_STEP_OL;

    (void)send_command(TS_CMD_PIDTUNE_IDLE_START, TEST_START_MS);

    assert_failed(TuneFail::WrongMode);
}

static void test_idle_start_refused_while_tapering(void)
{
    setup_idle_tune();
    configPage2.idleTaperTime = 10U;

    (void)send_command(TS_CMD_PIDTUNE_IDLE_START, TEST_START_MS);

    assert_failed(TuneFail::NotReady);
}

static void test_idle_start_refused_with_throttle_open(void)
{
    setup_idle_tune();
    currentStatus.TPS = configPage2.iacTPSlimit + 1U;

    (void)send_command(TS_CMD_PIDTUNE_IDLE_START, TEST_START_MS);

    assert_failed(TuneFail::NotReady);
}

static void test_idle_throttle_closed_even_with_a_high_reset_tps(void)
{
    // The idle integral reset TPS can be set as high as 100%: the tune still needs the throttle closed
    setup_idle_tune();
    configPage2.iacTPSlimit = 200U;
    currentStatus.TPS = IDLE_MAX_TPS + 1U;
    (void)send_command(TS_CMD_PIDTUNE_IDLE_START, TEST_START_MS);
    assert_failed(TuneFail::NotReady);

    setup_idle_tune();
    configPage2.iacTPSlimit = 200U;
    (void)send_command(TS_CMD_PIDTUNE_IDLE_START, TEST_START_MS);
    TEST_ASSERT_TRUE(isPidTuneIdleRunning());
    currentStatus.TPS = IDLE_MAX_TPS + 1U;
    run_control(TEST_START_MS + 100U);
    assert_failed(TuneFail::ConditionLost);
}

static void test_idle_start_refused_below_step_hysteresis(void)
{
    setup_idle_tune();
    configPage6.iacStepHyster = 4U;
    configPage15.pidTuneIdleAmplitude = 8U;

    (void)send_command(TS_CMD_PIDTUNE_IDLE_START, TEST_START_MS);

    assert_failed(TuneFail::AmplitudeBelowHysteresis);
}

static void test_idle_start_refused_without_authority(void)
{
    setup_idle_tune();
    // 8 steps above the valve position is past the closed loop maximum
    _idleState.idle_pid_target_value = (300 * 4) - 20;

    (void)send_command(TS_CMD_PIDTUNE_IDLE_START, TEST_START_MS);

    assert_failed(TuneFail::NoAuthority);
}

static void test_idle_start(void)
{
    setup_idle_tune();

    TEST_ASSERT_TRUE(send_command(TS_CMD_PIDTUNE_IDLE_START, TEST_START_MS));

    TEST_ASSERT_EQUAL(TunePhase::Settle, pidTuneState.tuner.getPhase());
    TEST_ASSERT_TRUE(isPidTuneIdleRunning());
    TEST_ASSERT_FALSE(isPidTuneEgoRunning());
    TEST_ASSERT_EQUAL(IDLE_BASE, pidTuneState.tuner.getBase());

    // A second start is ignored while running
    (void)send_command(TS_CMD_PIDTUNE_EGO_START, TEST_START_MS);
    TEST_ASSERT_TRUE(isPidTuneIdleRunning());
}

static void test_idle_relay_runs_at_the_pid_rate(void)
{
    setup_idle_tune();
    (void)send_command(TS_CMD_PIDTUNE_IDLE_START, TEST_START_MS);

    int32_t output = 0;
    TEST_ASSERT_TRUE(pidTuneIdleRelayStep(TEST_START_MS, IDLE_RPM, IDLE_BASE, output));
    TEST_ASSERT_EQUAL(IDLE_BASE, output);

    output = 0;
    TEST_ASSERT_FALSE(pidTuneIdleRelayStep(TEST_START_MS + STEP_MS - 1U, IDLE_RPM, IDLE_BASE, output));
    TEST_ASSERT_EQUAL(0, output);
    TEST_ASSERT_TRUE(pidTuneIdleRelayStep(TEST_START_MS + STEP_MS, IDLE_RPM, IDLE_BASE, output));
    TEST_ASSERT_EQUAL(IDLE_BASE, output);
}

static void test_idle_tune_done(void)
{
    setup_idle_tune();
    (void)send_command(TS_CMD_PIDTUNE_IDLE_START, TEST_START_MS);
    uint32_t nowMs = settle_idle(TEST_START_MS);
    TEST_ASSERT_EQUAL(TunePhase::Relay, pidTuneState.tuner.getPhase());

    // +/-8 steps around the valve position
    int32_t output = pidTuneState.tuner.getOutput();
    TEST_ASSERT_EQUAL(IDLE_BASE + (8 * 4), output);
    for (uint8_t index = 1U; isPidTuneIdleRunning() && (index < 100U); ++index)
    {
        nowMs += STEP_MS;
        TEST_ASSERT_TRUE(pidTuneIdleRelayStep(nowMs, IDLE_RPM + IDLE_TRIANGLE[index % IDLE_TRIANGLE_SAMPLES], output, output));
    }

    TEST_ASSERT_EQUAL(TunePhase::Done, pidTuneState.tuner.getPhase());
    TEST_ASSERT_EQUAL(3000U, pidTuneState.tuner.getPeriodMs());
    TEST_ASSERT_EQUAL(40U, pidTuneState.tuner.getAmplitude());
    // Ku = 4 * 32 / (pi * 40) = 1.02, balanced PI: Kc = 0.3 Ku = 0.305, Ti = Tu = 3S
    TEST_ASSERT_EQUAL_UINT16(260U, pidTuneState.ultimateGainQ8);
    TEST_ASSERT_EQUAL_UINT8(10U, pidTuneState.result.kp);
    TEST_ASSERT_EQUAL_UINT8(3U, pidTuneState.result.ki);
    TEST_ASSERT_EQUAL_UINT8(0U, pidTuneState.result.kd);

    // Back to the PID, at the position the test started from
    TEST_ASSERT_EQUAL(IDLE_BASE, output);
    TEST_ASSERT_EQUAL(IDLE_BASE, _idleState.idle_pid_target_value);
    int32_t pidOutput = 0;
    TEST_ASSERT_TRUE(_idleState.idlePID.compute(nowMs + 10000UL, IDLE_RPM, &pidOutput));
    TEST_ASSERT_EQUAL(IDLE_BASE, pidOutput);
}

static void test_idle_aborts_on_throttle(void)
{
    setup_idle_tune();
    (void)send_command(TS_CMD_PIDTUNE_IDLE_START, TEST_START_MS);
    uint32_t nowMs = settle_idle(TEST_START_MS);
    _idleState.idle_pid_target_value = pidTuneState.tuner.getOutput();

    currentStatus.TPS = configPage2.iacTPSlimit + 1U;
    run_control(nowMs);

    assert_failed(TuneFail::ConditionLost);
    TEST_ASSERT_FALSE(isPidTuneIdleRunning());
    TEST_ASSERT_EQUAL(IDLE_BASE, _idleState.idle_pid_target_value);
}

static void test_idle_aborts_on_load_change(void)
{
    setup_idle_tune();
    (void)send_command(TS_CMD_PIDTUNE_IDLE_START, TEST_START_MS);
    run_control(TEST_START_MS + 100U);
    TEST_ASSERT_TRUE(isPidTuneIdleRunning());

    currentStatus.acStatus.turningOn = true;
    run_control(TEST_START_MS + 200U);
    assert_failed(TuneFail::ConditionLost);

    setup_idle_tune();
    (void)send_command(TS_CMD_PIDTUNE_IDLE_START, TEST_START_MS);
    currentStatus.fanDuty = 100U;
    run_control(TEST_START_MS + 100U);
    assert_failed(TuneFail::ConditionLost);
}

static void test_idle_aborts_on_stall_and_mode_change(void)
{
    setup_idle_tune();
    (void)send_command(TS_CMD_PIDTUNE_IDLE_START, TEST_START_MS);
    currentStatus.RPM = IDLE_MIN_RPM - 1U;
    run_control(TEST_START_MS + 100U);
    assert_failed(TuneFail::ConditionLost);

    setup_idle_tune();
    (void)send_command(TS_CMD_PIDTUNE_IDLE_START, TEST_START_MS);
    configPage6.iacAlgorithm = IAC_ALGORITHM_STEP_CL;
    run_control(TEST_START_MS + 100U);
    assert_failed(TuneFail::WrongMode);
}

static void test_idle_timeout_without_samples(void)
{
    setup_idle_tune();
    (void)send_command(TS_CMD_PIDTUNE_IDLE_START, TEST_START_MS);

    run_control(TEST_START_MS + ((uint32_t)configPage15.pidTuneTimeout * 1000UL) - 1U);
    TEST_ASSERT_TRUE(isPidTuneIdleRunning());

    run_control(TEST_START_MS + ((uint32_t)configPage15.pidTuneTimeout * 1000UL));
    assert_failed(TuneFail::NoConvergence);
}

static void test_abort_command(void)
{
    setup_idle_tune();
    (void)send_command(TS_CMD_PIDTUNE_IDLE_START, TEST_START_MS);
    uint32_t nowMs = settle_idle(TEST_START_MS);
    _idleState.idle_pid_target_value = pidTuneState.tuner.getOutput();

    TEST_ASSERT_TRUE(send_command(TS_CMD_PIDTUNE_ABORT, nowMs));

    assert_failed(TuneFail::Aborted);
    TEST_ASSERT_EQUAL(IDLE_BASE, _idleState.idle_pid_target_value);
}

static void test_apply_idle_result(void)
{
    run_idle_tune_to_done();
    TEST_ASSERT_FALSE(pidTuneIdleTakeBumpless());

    TEST_ASSERT_TRUE(send_command(TS_CMD_PIDTUNE_APPLY, TEST_START_MS));

    TEST_ASSERT_EQUAL_UINT8(10U, configPage6.idleKP);
    TEST_ASSERT_EQUAL_UINT8(3U, configPage6.idleKI);
    TEST_ASSERT_EQUAL_UINT8(0U, configPage6.idleKD);
    TEST_ASSERT_TRUE(pidTuneState.isApplied);
    TEST_ASSERT_TRUE(pidTuneState.isUiRefresh);
    // The new gains are applied without a jump, once
    TEST_ASSERT_TRUE(pidTuneIdleTakeBumpless());
    TEST_ASSERT_FALSE(pidTuneIdleTakeBumpless());
    TEST_ASSERT_FALSE(pidTuneEgoTakeBumpless());

    // Applied once only
    configPage6.idleKP = 50U;
    (void)send_command(TS_CMD_PIDTUNE_APPLY, TEST_START_MS);
    TEST_ASSERT_EQUAL_UINT8(50U, configPage6.idleKP);
}

// The bytes mean something else in stepper closed loop (velocity form): a result for the open + closed loop mode can't go there
static void test_apply_refused_after_a_mode_change(void)
{
    run_idle_tune_to_done();
    configPage6.idleKP = 50U;
    configPage6.iacAlgorithm = IAC_ALGORITHM_STEP_CL;

    (void)send_command(TS_CMD_PIDTUNE_APPLY, TEST_START_MS);

    TEST_ASSERT_EQUAL_UINT8(50U, configPage6.idleKP);
    assert_failed(TuneFail::WrongMode);
    TEST_ASSERT_FALSE(pidTuneState.isApplied);
    TEST_ASSERT_FALSE(pidTuneState.isUiRefresh);
}

static void test_apply_needs_a_result(void)
{
    setup_idle_tune();
    configPage6.idleKP = 50U;

    (void)send_command(TS_CMD_PIDTUNE_APPLY, TEST_START_MS);
    TEST_ASSERT_EQUAL_UINT8(50U, configPage6.idleKP);

    // Failed test: nothing to apply
    (void)send_command(TS_CMD_PIDTUNE_IDLE_START, TEST_START_MS);
    (void)send_command(TS_CMD_PIDTUNE_ABORT, TEST_START_MS);
    (void)send_command(TS_CMD_PIDTUNE_APPLY, TEST_START_MS);
    TEST_ASSERT_EQUAL_UINT8(50U, configPage6.idleKP);
    TEST_ASSERT_FALSE(pidTuneState.isUiRefresh);
}

static void test_idle_gains_are_the_config_bytes(void)
{
    setup_idle_tune();
    PidTuningParameters gains = pidTuneIdleGains(configPage6);
    TEST_ASSERT_EQUAL(32, gains.Kp);
    TEST_ASSERT_EQUAL(8, gains.Ki);
    TEST_ASSERT_EQUAL(0, gains.Kd);
}

static void test_commands_of_other_groups_are_ignored(void)
{
    setup_idle_tune();
    TEST_ASSERT_FALSE(send_command(TS_CMD_VSS_60KMH, TEST_START_MS));
    TEST_ASSERT_FALSE(send_command(TS_CMD_TEST_ENBL, TEST_START_MS));
    TEST_ASSERT_TRUE(send_command(0x40FFU, TEST_START_MS));
    TEST_ASSERT_EQUAL(TunePhase::Idle, pidTuneState.tuner.getPhase());
}

// ---------------------------------------------------------------- AFR

static void test_ego_start_refused_wrong_mode(void)
{
    setup_ego_tune();
    configPage6.egoAlgorithm = EGO_ALGORITHM_SIMPLE;
    (void)send_command(TS_CMD_PIDTUNE_EGO_START, TEST_START_MS);
    assert_failed(TuneFail::WrongMode);
    TEST_ASSERT_EQUAL(TuneLoop::Ego, pidTuneState.loop);

    setup_ego_tune();
    configPage6.egoType = EGO_TYPE_NARROW;
    (void)send_command(TS_CMD_PIDTUNE_EGO_START, TEST_START_MS);
    assert_failed(TuneFail::WrongMode);
}

static void test_ego_start_refused_outside_closed_loop(void)
{
    setup_ego_tune();
    currentStatus.TPS = configPage6.egoTPSMax + 1U;
    (void)send_command(TS_CMD_PIDTUNE_EGO_START, TEST_START_MS);
    assert_failed(TuneFail::NotReady);
}

static void test_ego_start_refused_without_authority(void)
{
    // 110% leaves 2% of the 12% authority
    setup_ego_tune();
    configPage6.egoLimit = 12U;
    currentStatus.egoCorrection = 110U;
    (void)send_command(TS_CMD_PIDTUNE_EGO_START, TEST_START_MS);
    assert_failed(TuneFail::NoAuthority);
}

static void test_ego_tune_done(void)
{
    setup_ego_tune();
    (void)send_command(TS_CMD_PIDTUNE_EGO_START, TEST_START_MS);
    TEST_ASSERT_TRUE(isPidTuneEgoRunning());
    TEST_ASSERT_FALSE(isPidTuneIdleRunning());

    // One closed loop step every 125mS. AFR: a triangle wave of +/-0.4, period 1.5S
    constexpr int8_t AFR_TRIANGLE[] = { 0, 1, 3, 4, 3, 1, 0, -1, -3, -4, -3, -1 };
    uint32_t nowMs = TEST_START_MS;
    uint8_t index = 0U;
    while (isPidTuneEgoRunning() && (index < 200U))
    {
        const bool isSettling = (pidTuneState.tuner.getPhase() == TunePhase::Settle);
        currentStatus.O2 = isSettling ? 147U : (uint8_t)(147 + AFR_TRIANGLE[index % _countof(AFR_TRIANGLE)]);
        currentStatus.egoCorrection = pidTuneEgoRelayStep(currentStatus, nowMs);
        if (!isSettling) { ++index; }
        nowMs += 125U;
    }

    TEST_ASSERT_EQUAL(TunePhase::Done, pidTuneState.tuner.getPhase());
    TEST_ASSERT_EQUAL(1500U, pidTuneState.tuner.getPeriodMs());
    TEST_ASSERT_EQUAL(4U, pidTuneState.tuner.getAmplitude());
    TEST_ASSERT_EQUAL(5U, pidTuneState.tuner.getRelayAmplitude());
    TEST_ASSERT_EQUAL(125U, pidTuneState.tuner.getSamplePeriodMs());
    // Ku = 4 * 5 / (pi * 4) = 1.59: the P term saturates at 200
    TEST_ASSERT_EQUAL_UINT8(200U, pidTuneState.result.kp);
    TEST_ASSERT_EQUAL_UINT8(17U, pidTuneState.result.ki);
    TEST_ASSERT_EQUAL_UINT8(0U, pidTuneState.result.kd);
    TEST_ASSERT_TRUE(pidTuneState.result.saturated);

    // Back to the PID, at the correction the test started from
    TEST_ASSERT_EQUAL_UINT8(100U, currentStatus.egoCorrection);
    TEST_ASSERT_EQUAL(0, egoPID.compute(currentStatus.O2));

    // Apply writes the AFR bytes
    (void)send_command(TS_CMD_PIDTUNE_APPLY, nowMs);
    TEST_ASSERT_EQUAL_UINT8(200U, configPage6.egoKP);
    TEST_ASSERT_EQUAL_UINT8(17U, configPage6.egoKI);
    TEST_ASSERT_EQUAL_UINT8(0U, configPage6.egoKD);
    TEST_ASSERT_TRUE(pidTuneEgoTakeBumpless());
    TEST_ASSERT_FALSE(pidTuneIdleTakeBumpless());
}

static void test_ego_aborts_when_the_operating_point_moves(void)
{
    setup_ego_tune();
    (void)send_command(TS_CMD_PIDTUNE_EGO_START, TEST_START_MS);
    run_control(TEST_START_MS + 100U);
    TEST_ASSERT_TRUE(isPidTuneEgoRunning());

    // More than 15% RPM
    currentStatus.RPM = IDLE_RPM + 140U;
    run_control(TEST_START_MS + 200U);
    assert_failed(TuneFail::ConditionLost);
    TEST_ASSERT_EQUAL_UINT8(100U, currentStatus.egoCorrection);

    setup_ego_tune();
    (void)send_command(TS_CMD_PIDTUNE_EGO_START, TEST_START_MS);
    currentStatus.MAP = 51U;
    run_control(TEST_START_MS + 100U);
    assert_failed(TuneFail::ConditionLost);

    setup_ego_tune();
    (void)send_command(TS_CMD_PIDTUNE_EGO_START, TEST_START_MS);
    currentStatus.isDFCOActive = true;
    run_control(TEST_START_MS + 100U);
    assert_failed(TuneFail::ConditionLost);
}

static void test_ego_fails_without_closed_loop_steps(void)
{
    setup_ego_tune();
    (void)send_command(TS_CMD_PIDTUNE_EGO_START, TEST_START_MS);

    run_control(TEST_START_MS + EGO_STEP_TIMEOUT_MS - STEP_MS - 1U);
    TEST_ASSERT_TRUE(isPidTuneEgoRunning());

    run_control(TEST_START_MS + EGO_STEP_TIMEOUT_MS - STEP_MS);
    assert_failed(TuneFail::StepPeriodTooLong);
}

void testPidTuneController(void)
{
  SET_UNITY_FILENAME() {
    RUN_TEST_P(test_idle_start_refused_when_stopped);
    RUN_TEST_P(test_idle_start_refused_in_open_loop);
    RUN_TEST_P(test_idle_start_refused_while_tapering);
    RUN_TEST_P(test_idle_start_refused_with_throttle_open);
    RUN_TEST_P(test_idle_throttle_closed_even_with_a_high_reset_tps);
    RUN_TEST_P(test_idle_start_refused_below_step_hysteresis);
    RUN_TEST_P(test_idle_start_refused_without_authority);
    RUN_TEST_P(test_idle_start);
    RUN_TEST_P(test_idle_relay_runs_at_the_pid_rate);
    RUN_TEST_P(test_idle_tune_done);
    RUN_TEST_P(test_idle_aborts_on_throttle);
    RUN_TEST_P(test_idle_aborts_on_load_change);
    RUN_TEST_P(test_idle_aborts_on_stall_and_mode_change);
    RUN_TEST_P(test_idle_timeout_without_samples);
    RUN_TEST_P(test_abort_command);
    RUN_TEST_P(test_apply_idle_result);
    RUN_TEST_P(test_apply_refused_after_a_mode_change);
    RUN_TEST_P(test_apply_needs_a_result);
    RUN_TEST_P(test_idle_gains_are_the_config_bytes);
    RUN_TEST_P(test_commands_of_other_groups_are_ignored);
    RUN_TEST_P(test_ego_start_refused_wrong_mode);
    RUN_TEST_P(test_ego_start_refused_outside_closed_loop);
    RUN_TEST_P(test_ego_start_refused_without_authority);
    RUN_TEST_P(test_ego_tune_done);
    RUN_TEST_P(test_ego_aborts_when_the_operating_point_moves);
    RUN_TEST_P(test_ego_fails_without_closed_loop_steps);
  }
}
