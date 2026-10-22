#pragma once

#include "src/controllers/protection/protectionController.h"
#include "src/controllers/protection/protectionController_details.h"
#include "src/controllers/sensorFault/sensorFaultController.h"
#include "globals.h"
#include "units.h"

using namespace protection::details;

extern state_t protectionState;
extern void protectionControlCore(statuses &current, const config6 &page6, const config10 &page10, const config15 &page15, bool oilHot);

constexpr uint8_t TEST_WARN_PIN = 23U;
/** @brief The limp home RPM of the tests (Unit: 100 RPM) */
constexpr uint8_t TEST_LIMP_RPM = 30U;

struct protection_context_t
{
    statuses current = {};
    config4 page4 = {};
    config6 page6 = {};
    config10 page10 = {};
    config15 page15 = {};
    pinNumbers_t pins = {};
    bool oilHot = false;

    /** @brief Engine running at 2000 RPM, warm, no boost, 43 PSI of fuel, protection cut on, every fork condition off */
    protection_context_t(void)
    {
        setDefaults(page15);
        page4.engineProtectMaxRPM = TEST_LIMP_RPM;
        page6.engineProtectType = PROTECT_CUT_BOTH;
        page10.fuelPressureEnable = 1U;
        current.rotationStatus = EngineRotationStatus::Running;
        current.setRpm(2000U);
        current.coolant = 85;
        current.MAP = 100U;
        current.baro = 100U;
        current.fuelPressure = 43U;
        initialiseProtection(page15, pins);
    }

    /** @brief What the main loop does right after checkEngineProtection() */
    void resolve(void) { protectionResolve(current, page6, page15); }

    /** @brief One 10Hz check */
    void tick(void) { protectionControlCore(current, page6, page10, page15, oilHot); }

    void ticks(uint8_t count)
    {
        for (uint8_t i = 0U; i < count; ++i) { tick(); }
    }

    /** @brief A 10Hz check followed by the resolve of the next loop */
    void tickAndResolve(void)
    {
        tick();
        resolve();
    }

    /** @brief The conditions found by upstream's checkEngineProtection() */
    void setUpstream(bool oilPressure, bool boost, bool afr)
    {
        current.engineProtect.oil = oilPressure;
        current.engineProtect.boostCut = boost;
        current.engineProtect.afr = afr;
    }

    bool isWarnPinHigh(void) const { return protectionState.warnPin._pin.isPinHigh(); }
};
