#include <unity.h>
#include "pidtune_fixture.h"
#include "corrections.h"
#include "units.h"

// Free pins in the simulator, so the idle outputs don't touch the serial port
static constexpr uint8_t TEST_IDLE1_PIN = 18U;
static constexpr uint8_t TEST_IDLE2_PIN = 19U;

static void reset_globals(void)
{
    resetFakeForkStorage();
    currentStatus = statuses();
    configPage2 = config2();
    configPage6 = config6();
    configPage9 = config9();
    configPage15 = config15();
    pidTuneController::details::setDefaults(configPage15);
    initialisePidTune(configPage6, configPage15);
}

void setup_idle_tune(void)
{
    reset_globals();
    pinNumbers.pinIdle1 = TEST_IDLE1_PIN;
    pinNumbers.pinIdle2 = TEST_IDLE2_PIN;

    configPage6.iacAlgorithm = IAC_ALGORITHM_STEP_OLCL;
    configPage6.iacStepHome = 0U;
    configPage6.iacStepHyster = 1U;
    configPage6.idleKP = 32U;
    configPage6.idleKI = 8U;
    configPage6.idleKD = 0U;
    configPage2.iacCLminValue = 0U;
    configPage2.iacCLmaxValue = 100U; // 300 steps: 1200 quarter steps
    configPage2.iacTPSlimit = 10U;
    configPage2.idleTaperTime = 0U;
    configPage9.iacMaxSteps = 100U;

    currentStatus.rotationStatus = EngineRotationStatus::Running;
    currentStatus.RPM = IDLE_RPM;
    currentStatus.TPS = 0U;
    currentStatus.CLIdleTarget = IDLE_RPM / 10U;

    initialiseIdle(false);
    _idleState.completedHomeSteps = 0U;
    _idleState.idleTaper = 0U;
    _idleState.idle_cl_target_rpm = IDLE_RPM;
    _idleState.idle_pid_target_value = IDLE_BASE;
}

void setup_ego_tune(void)
{
    reset_globals();
    configPage6.egoType = EGO_TYPE_WIDE;
    configPage6.egoAlgorithm = EGO_ALGORITHM_PID;
    configPage6.egoLimit = 15U;
    configPage6.egoKP = 35U;
    configPage6.egoKI = 15U;
    configPage6.egoKD = 0U;
    configPage6.ego_sdelay = 10U;
    configPage6.egoTemp = temperatureAddOffset(60);
    configPage6.egoRPM = 5U;
    configPage6.egoTPSMax = 40U;
    configPage6.ego_min = 70U;
    configPage6.ego_max = 200U;
    configPage9.egoMAPMax = 100U;
    configPage9.egoMAPMin = 0U;

    currentStatus.rotationStatus = EngineRotationStatus::Running;
    currentStatus.runSecs = 20U;
    currentStatus.coolant = 85;
    currentStatus.RPM = IDLE_RPM;
    currentStatus.TPS = 0U;
    currentStatus.MAP = 40U;
    currentStatus.O2 = 147U;
    currentStatus.afrTarget = 147U;
    currentStatus.egoCorrection = 100U;
    currentStatus.isDFCOActive = false;

    egoPID.setOutputLimits(-15, 15);
    egoPID.setTunings(PidTuningParameters(35U, 15U, 0U) * -1);
    egoPID.setSetPoint(147U);
    egoPID.resetIntegral(147);
}

bool send_command(uint16_t command, uint32_t nowMs)
{
    return pidTuneCommandCore(command, currentStatus, configPage2, configPage6, configPage15, nowMs);
}

void run_control(uint32_t nowMs)
{
    pidTuneControlCore(currentStatus, configPage2, configPage6, configPage15, nowMs);
}

const int8_t IDLE_TRIANGLE[IDLE_TRIANGLE_SAMPLES] = { 0, 13, 27, 40, 27, 13, 0, -13, -27, -40, -27, -13 };

uint32_t settle_idle(uint32_t nowMs)
{
    int32_t output = IDLE_BASE;
    while (pidTuneState.tuner.getPhase() == pidTune::TunePhase::Settle)
    {
        TEST_ASSERT_TRUE(pidTuneIdleRelayStep(nowMs, IDLE_RPM, output, output));
        nowMs += pidTuneController::details::IDLE_SAMPLE_MS;
    }
    return nowMs - pidTuneController::details::IDLE_SAMPLE_MS;
}

void run_idle_tune_to_done(void)
{
    setup_idle_tune();
    (void)send_command(TS_CMD_PIDTUNE_IDLE_START, TEST_START_MS);
    uint32_t nowMs = settle_idle(TEST_START_MS);
    int32_t output = pidTuneState.tuner.getOutput();
    for (uint8_t index = 1U; isPidTuneIdleRunning() && (index < 100U); ++index)
    {
        nowMs += pidTuneController::details::IDLE_SAMPLE_MS;
        (void)pidTuneIdleRelayStep(nowMs, IDLE_RPM + IDLE_TRIANGLE[index % IDLE_TRIANGLE_SAMPLES], output, output);
    }
    TEST_ASSERT_EQUAL(pidTune::TunePhase::Done, pidTuneState.tuner.getPhase());
}
