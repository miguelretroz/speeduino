#pragma once

#include "src/controllers/sensorFault/sensorFaultController.h"
#include "src/controllers/sensorFault/sensorFaultController_details.h"
#include "globals.h"
#include "units.h"

using namespace sensorFault::details;

extern state_t sensorFaultState;
extern void sensorFaultCheck(statuses &current, const config15 &page15, bool immediate);

/** @brief A mid range reading (0-1023): never a fault */
constexpr uint16_t ADC_MID = 512U;
/** @brief Readings at the electrical limits */
constexpr uint16_t ADC_SHORT = 0U;
constexpr uint16_t ADC_OPEN = 1023U;

struct sensorfault_context_t
{
    statuses current = {};
    config15 page15 = {};

    /** @brief Detection on for every sensor, defaults otherwise, every reading mid range */
    sensorfault_context_t(void)
    {
        setDefaults(page15);
        page15.sensorFaultClt = 1U;
        page15.sensorFaultIat = 1U;
        page15.sensorFaultMap = 1U;
        page15.sensorFaultTps = 1U;
        current.cltADC = ADC_MID;
        current.iatADC = ADC_MID;
        current.tpsADC = (uint8_t)(ADC_MID >> 2U);
        current.coolant = 85;
        current.IAT = 25;
        current.MAP = 40U;
        current.TPS = 0U;
        sensorFaultMapSample(ADC_MID);
        initialiseSensorFaults(current, page15);
    }

    /** @brief One 10Hz check */
    void tick(void) { sensorFaultControl(current, page15); }

    /** @brief A number of 10Hz checks */
    void ticks(uint8_t count)
    {
        for (uint8_t i = 0U; i < count; ++i) { tick(); }
    }

    bool isFaulted(uint8_t sensor) const { return BIT_CHECK(current.sensorFaults, sensor); }
};
