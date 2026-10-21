#pragma once

/**
 * @file
 * @brief PID auto tune and gain adaptation for the idle and AFR closed loops. Fork specific.
 *
 * Auto tune, started from TunerStudio with the engine in a steady condition:
 * - Idle: closed loop idle running (PWM or stepper), throttle closed. The valve is switched around its position.
 * - AFR: AFR closed loop running with the PID algorithm and a wideband sensor. The fuel correction is switched around its value.
 *
 * While the test runs, a relay replaces the PID (see relayTuner.h). When it stops (done, failed or aborted) the
 * PID takes over again from where the test started, without a jump. The suggested P, I and D are shown live, and
 * Apply writes them to the tune (page 6) and burns them.
 *
 * Gain adaptation (each loop can be turned on separately): a multiplier of P, I and D, slowly adjusted while the
 * loop runs (see gainAdapt.h). It's kept in EEPROM and goes back to 100% when P, I or D change.
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

/** @brief Loads the gain adaptation multipliers. Call it after the config has been loaded. */
void initialisePidTune(const config6 &page6, const config15 &page15);

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

/** @brief The idle PID gains: the config bytes, scaled by the gain adaptation when it's on */
PidTuningParameters pidTuneIdleGains(const config6 &page6);

/**
 * @brief Feed the gain adaptation, after each idle PID computation
 *
 * @param error Target RPM - RPM
 * @param output The PID output
 */
void pidTuneIdleAdaptSample(const statuses &current, int16_t error, int32_t output, uint32_t nowMs);

/** @brief True (once) when the next idle gain change must not make the output jump */
bool pidTuneIdleTakeBumpless(void);
///@}

/** @name AFR closed loop hooks (corrections.cpp) */
///@{
/** @brief The auto tune drives the AFR correction instead of the PID */
bool isPidTuneEgoRunning(void);

/** @brief Run the relay instead of the AFR PID, for one closed loop step. Returns the correction (%). */
uint8_t pidTuneEgoRelayStep(const statuses &current, uint32_t nowMs);

/** @brief The AFR closed loop PID gains: the config bytes, scaled by the gain adaptation when it's on */
PidTuningParameters pidTuneEgoGains(const config6 &page6);

/**
 * @brief Feed the gain adaptation, after each AFR closed loop PID step
 *
 * @param correction The correction the PID worked out (%)
 */
void pidTuneEgoAdaptSample(const statuses &current, const config6 &page6, uint8_t correction, uint32_t nowMs);

/** @brief True (once) when the next AFR gain change must not make the output jump */
bool pidTuneEgoTakeBumpless(void);
///@}

/** @name Live data (bytes 142 to 156 of the realtime data) */
///@{
/** @brief First live data byte */
constexpr uint16_t PIDTUNE_LOG_FIRST_BYTE = 142U;
/** @brief Number of live data bytes */
constexpr uint8_t PIDTUNE_LOG_BYTES = 15U;

/**
 * @brief One live data byte, index 0 being byte 142
 *
 * - 0: status. Bits 0-2: phase (0 idle, 1 settle, 2 relay, 3 done, 4 failed). Bit 3: loop (0 idle, 1 AFR).
 *   Bit 4: TunerStudio must reload page 6. Bit 5: gain limited. Bit 6: applied. Bit 7: a byte was rounded up to 1.
 * - 1: failure (pidTune::TuneFail)
 * - 2, 3, 4: suggested P, I and D bytes
 * - 5-6: ultimate period (mS). 7-8: oscillation amplitude (RPM or 0.1 AFR). 9-10: ultimate gain (Q8)
 * - 11: cycles measured
 * - 12, 13: gain adaptation multipliers, idle and AFR (%)
 * - 14: gain adaptation status. Bits 0, 1: idle, AFR loop steady. Bits 2-4, 5-7: last decision, idle and AFR
 */
uint8_t getPidTuneLogByte(uint8_t index);

/**
 * @brief Call once live data (from getTSLogEntry) has been sent: the page reload request is cleared once the bytes
 * that carry it have gone out. TunerStudio may read the live data in several parts.
 */
void pidTuneAckUiRefresh(uint16_t offset, uint16_t length);
///@}
