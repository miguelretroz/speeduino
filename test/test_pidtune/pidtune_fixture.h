#pragma once

#include "globals.h"
#include "src/controllers/pidTune/pidTuneController.h"
#include "src/controllers/pidTune/pidTuneController_details.h"
#include "src/controllers/idle/idle.h"
#include "src/controllers/idle/idleController_state.h"
#include "src/PID/PID.h"
#include "../fake_fork_storage.h"

extern pidTuneController::details::state_t pidTuneState;
extern idleController::detail::state_t _idleState;
extern PID egoPID;

void pidTuneControlCore(statuses &current, const config2 &page2, const config6 &page6, const config15 &page15, uint32_t nowMs);
bool pidTuneCommandCore(uint16_t command, statuses &current, const config2 &page2, config6 &page6, const config15 &page15, uint32_t nowMs);

/** @brief Test start time: well past 0, so time differences never wrap */
constexpr uint32_t TEST_START_MS = 100000UL;

/** @brief Idle PID output (quarter steps) at the start of the tests: 100 steps */
constexpr int32_t IDLE_BASE = 400;
constexpr uint16_t IDLE_RPM = 900U;

/** @brief Closed loop stepper idle (open + closed loop) at 900 RPM, ready to be tuned */
void setup_idle_tune(void);

/** @brief AFR closed loop (PID, wideband) in its operating window, ready to be tuned */
void setup_ego_tune(void);

/** @brief Send a command, at the given time */
bool send_command(uint16_t command, uint32_t nowMs);

/** @brief Run the 10Hz checks */
void run_control(uint32_t nowMs);

/** @brief RPM triangle wave for the idle relay: 12 samples per cycle (3S), +/-40 RPM */
constexpr uint8_t IDLE_TRIANGLE_SAMPLES = 12U;
extern const int8_t IDLE_TRIANGLE[IDLE_TRIANGLE_SAMPLES];

/** @brief Hold the RPM while the idle relay settles, at its own rate. Returns the time of the last sample. */
uint32_t settle_idle(uint32_t nowMs);

/** @brief Run an idle auto tune on the triangle wave until it's done: Ku 1.02 (Q8 260), Tu 3S, bytes 10/2/0 */
void run_idle_tune_to_done(void);
