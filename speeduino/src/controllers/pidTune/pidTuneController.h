#pragma once

/**
 * @file
 * @brief PID auto tune for the idle and AFR closed loops. Fork specific.
 *
 * Started from TunerStudio, with the engine in a steady condition:
 * - Idle: closed loop idle running (PWM or stepper), throttle closed. The valve is switched around its position.
 * - AFR: AFR closed loop running with the PID algorithm and a wideband sensor. The fuel correction is switched around its value.
 *
 * While the test runs, a relay replaces the PID (see relayTuner.h). When it stops (done, failed or aborted) the
 * PID takes over again from where the test started, without a jump. The suggested P, I and D are shown live, and
 * Apply writes them to the tune (page 6) and burns them.
 */

#include "statuses.h"
#include "config_pages.h"
#include "src/PID/PidTuningParameters.h"

/** @name TunerStudio commands ('E' command, group 0x40) */
///@{
constexpr uint16_t TS_CMD_PIDTUNE_IDLE_START       = 0x4000U;
constexpr uint16_t TS_CMD_PIDTUNE_EGO_START        = 0x4001U;
constexpr uint16_t TS_CMD_PIDTUNE_ABORT            = 0x4002U;
constexpr uint16_t TS_CMD_PIDTUNE_APPLY            = 0x4003U;
constexpr uint16_t TS_CMD_PIDTUNE_ADAPT_RESET_IDLE = 0x4004U;
constexpr uint16_t TS_CMD_PIDTUNE_ADAPT_RESET_EGO  = 0x4005U;
///@}

void initialisePidTune(void);

/** @brief Checks the operating conditions and the timeouts of a running test. Call it at 10Hz. */
void pidTuneControl(statuses &current, const config2 &page2, const config6 &page6, const config15 &page15);

/**
 * @brief Process a TunerStudio command
 *
 * @param page6 Must be the global page, as Apply burns it to EEPROM
 * @return false if it isn't a PID auto tune command
 */
bool handlePidTuneCommand(uint16_t command, statuses &current, config6 &page6, const config15 &page15);

/** @name Idle hooks (idle.cpp) */
///@{
/** @brief The auto tune drives the idle valve instead of the PID */
bool isPidTuneIdleRunning(void);

/**
 * @brief Run the relay instead of the idle PID
 *
 * @param realizedOutput Where the valve really is (the stepper position stops short of the target by the step hysteresis)
 * @param output The PID output to use. Only set when true is returned.
 * @return true if the output was updated (at the rate of the idle PID)
 */
bool pidTuneIdleRelayStep(uint32_t nowMs, uint16_t rpm, int32_t realizedOutput, int32_t &output);

/** @brief The idle PID gains */
PidTuningParameters pidTuneIdleGains(const config6 &page6);

/** @brief True (once) when the next idle gain change must not make the output jump */
bool pidTuneIdleTakeBumpless(void);
///@}

/** @name AFR closed loop hooks (corrections.cpp) */
///@{
/** @brief The auto tune drives the AFR correction instead of the PID */
bool isPidTuneEgoRunning(void);

/** @brief Run the relay instead of the AFR PID, for one closed loop step. Returns the correction (%). */
uint8_t pidTuneEgoRelayStep(const statuses &current, uint32_t nowMs);

/** @brief The AFR closed loop PID gains */
PidTuningParameters pidTuneEgoGains(const config6 &page6);

/** @brief True (once) when the next AFR gain change must not make the output jump */
bool pidTuneEgoTakeBumpless(void);
///@}
