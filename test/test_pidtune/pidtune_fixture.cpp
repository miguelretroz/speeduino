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
    initialisePidTune();
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
