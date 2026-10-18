#pragma once

#include "src/controllers/coldStart/coldStartController.h"
#include "src/controllers/coldStart/coldStartController_details.h"
#include "globals.h"

using namespace coldStartController::details;

extern state_t coldStartState;

/** @brief Step size used by the fixture to advance time */
constexpr uint32_t TEST_STEP_MS = 10U;

constexpr uint8_t TEST_COLDSTART_PIN = 22U;

/** @brief Battery levels (Unit: 0.1V) */
constexpr uint8_t BATTERY_KEY_ON = 125U;
constexpr uint8_t BATTERY_USB_ONLY = 0U;

/** @brief Fake EEPROM: only the 2 cold start bytes are stored, everything else reads as erased */
struct fake_coldstart_storage_t
{
    uint8_t marker = 0xFFU;
    uint8_t attempts = 0xFFU;
    uint16_t attemptWrites = 0U;
};
extern fake_coldstart_storage_t fakeStorage;

/** @brief Install the fake EEPROM, with both cold start bytes erased */
void resetFakeStorage(void);

struct coldstart_context_t
{
    statuses current = {};
    config4 page4 = {};
    // The curves are bound to the global page, so the tests have to use it too
    config15 &page15 = configPage15;
    pinNumbers_t pins = {};
    uint32_t nowMs = 0U;

    void initialise(void)
    {
        initialiseColdStart(current, page15, pins);
    }

    void run(void)
    {
        extern void coldStartControlCore(statuses &current, const config4 &page4, const config15 &page15, uint32_t nowMs);
        coldStartControlCore(current, page4, page15, nowMs);
    }

    /** @brief Run the controller for a period of time, returning how long (mS) the output was on */
    uint32_t runFor(uint32_t durationMs)
    {
        uint32_t onTimeMs = 0U;
        for (uint32_t elapsed = 0U; elapsed < durationMs; elapsed += TEST_STEP_MS)
        {
            run();
            if (coldStartState.isOutputOn) { onTimeMs += TEST_STEP_MS; }
            nowMs += TEST_STEP_MS;
        }
        return onTimeMs;
    }

    bool isOutputOn(void) const { return coldStartState.isOutputOn; }
    bool isPinHigh(void) const { return coldStartState.pin._pin.isPinHigh(); }

    void keyOn(void) { current.battery10 = BATTERY_KEY_ON; }
    void keyOff(void) { current.battery10 = BATTERY_USB_ONLY; }
    void crank(void) { current.rotationStatus = EngineRotationStatus::Cranking; }
    void engineRunning(void) { current.rotationStatus = EngineRotationStatus::Running; }
    void engineStopped(void) { current.rotationStatus = EngineRotationStatus::Stopped; }

    /** @brief One failed attempt: crank for a while, then stop */
    void failedCrankAttempt(void)
    {
        crank();
        (void)runFor(2000U);
        engineStopped();
        (void)runFor(100U);
    }
};

/** @brief Default cold start config, enabled on the test pin, cold engine, key off */
coldstart_context_t setup_coldstart(void);

/** @brief Set every value of a cold start curve */
void fill_curve(uint8_t (&values)[6], uint8_t value);
