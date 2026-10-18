#pragma once

#include "src/controllers/oilTemp/oilTempController.h"
#include "src/controllers/oilTemp/oilTempController_details.h"
#include "globals.h"
#include "sensors.h"
#include "../fake_fork_storage.h"

using namespace oilTempController::details;

extern state_t oilTempState;

constexpr uint8_t TEST_ALERT_PIN = 23U;

/** @brief A mid range reading: never a sensor fault */
constexpr uint16_t ADC_MID = 500U;

struct oiltemp_context_t
{
    // The calibration and the sensor read path use the global page, so the tests use it too
    config15 &page15 = configPage15;
    pinNumbers_t pins = {};

    void initialise(void) { initialiseOilTemp(page15, pins); }
    void read(uint16_t adc) { oilTempControl(page15, adc); }
    bool isAlertPinHigh(void) const { return oilTempState.alertPin._pin.isPinHigh(); }
};

/** @brief Enabled with the default calibration, alert at 130C (clears at 125C), no alert output */
oiltemp_context_t setup_oiltemp(void);

/** @brief A calibration where every reading converts to the same temperature */
void setFlatCalibration(int16_t temperature);
