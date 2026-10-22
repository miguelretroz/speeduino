/** @file
 * Oil temperature input and hot oil alert. See oilTempController.h for an overview.
 * The config is found at the end of page 15 (of type @ref config15).
 */
#include "oilTempController.h"
#include "oilTempController_details.h"
#include "../../../globals.h"
#include "../../../sensors.h"
#include "../../../maths.h"
#include "../../../units.h"
#include "../../../unit_testing.h"
#include "../../../preprocessor.h"
#include "../../pins/pinMapping.h"

using namespace oilTempController::details;

TESTABLE_STATIC state_t oilTempState;

/**
 * @brief Default calibration (+40 offset) at ADC 0, 33, 66 ... 1023.
 *
 * Generated with the Steinhart-Hart equation from TunerStudio's "Bosch CLT/IAT" preset
 * (-30C 26144 ohm, 20C 2500 ohm, 110C 144 ohm) and a 2490 ohm pull-up, as TunerStudio would.
 */
static constexpr uint8_t DEFAULT_CALIBRATION[32] PROGMEM = {
  255, 173, 143, 127, 115, 107, 100, 94, 89, 84, 80, 76, 72, 68, 65, 62,
  58, 55, 52, 49, 46, 43, 39, 36, 32, 28, 23, 18, 12, 4, 0, 0,
};

namespace oilTempController {

namespace details {

void __attribute__((optimize("Os"))) setDefaults(config15 &page15)
{
  page15.oilTempEnable = 0U;
  page15.oilTempAlertOutput = 0U;
  page15.oilTempAlertInv = 0U;
  page15.oilTempUnused1 = 0U;
  page15.oilTempPin = 9U; // A9: on J5 of the v0.4.3d, which already has a 470R/0.1uF input filter
  page15.oilTempUnused2 = 0U;
  page15.oilTempAlertPin = 0U;
  page15.oilTempUnused3 = 0U;
  page15.oilTempAlertOn = temperatureAddOffset(130);
  page15.oilTempAlertHyst = 5U;
  page15.oilTempFilter = ADCFILTER_CLT_DEFAULT;
}

void __attribute__((optimize("Os"))) setDefaultCalibration(void)
{
  static_assert(_countof(DEFAULT_CALIBRATION)==decltype(oilTempCalibrationTable)::size(), "Calibration size mismatch");
  for (uint8_t x = 0U; x < _countof(DEFAULT_CALIBRATION); ++x)
  {
    oilTempCalibrationTable.values[x] = pgm_read_byte(&DEFAULT_CALIBRATION[x]);
    oilTempCalibrationTable.axis[x] = x * 33U; // Same bins as the TunerStudio calibration
  }
}

} // namespace details

} // namespace oilTempController

uint8_t getOilTempInputPin(uint8_t analogIndex)
{
#if defined(CORE_AVR)
  // The Mega's analog pins are contiguous. ANALOG_PINS stops at A14, but A15 is broken out to the
  // proto area of older v0.4 boards (E.g. v0.4.3), so allow it too.
  return (analogIndex <= 15U) ? (uint8_t)(A0 + analogIndex) : NOT_A_PIN;
#else
  return (analogIndex < _countof(ANALOG_PINS)) ? pinTranslateAnalog(analogIndex) : NOT_A_PIN;
#endif
}

static void writeAlertOutput(bool isOn, const config15 &page15)
{
  if (oilTempState.alertPin.isValid())
  {
    if (isOn != (page15.oilTempAlertInv != 0U)) { oilTempState.alertPin.setPinHigh(); }
    else { oilTempState.alertPin.setPinLow(); }
  }
}

void __attribute__((optimize("Os"))) initialiseOilTemp(const config15 &page15, const pinNumbers_t &pins)
{
  oilTempState = state_t();
  oilTempState.isEnabled = (page15.oilTempEnable != 0U) && (pins.pinOilTemp != NOT_A_PIN);

  if (oilTempState.isEnabled && (page15.oilTempAlertOutput != 0U) && (pins.pinOilTempAlert != NOT_A_PIN) && !pinIsReserved(pins.pinOilTempAlert))
  {
    oilTempState.alertPin.setPin(pins.pinOilTempAlert, OUTPUT);
    writeAlertOutput(false, page15);
  }
}

static bool isOilHot(const config15 &page15)
{
  // A broken sensor must not raise a false alarm. It is reported as a fault instead.
  if (oilTempState.isFault) { return false; }

  int16_t alertTemp = temperatureRemoveOffset(page15.oilTempAlertOn);
  bool isHot = oilTempState.isHot;
  if (oilTempState.temperature >= alertTemp) { isHot = true; }
  else if (oilTempState.temperature <= (alertTemp - (int16_t)page15.oilTempAlertHyst)) { isHot = false; }
  else { /* Hysteresis band: keep the last state */ }
  return isHot;
}

void oilTempControl(const config15 &page15, uint16_t adc)
{
  if (oilTempState.isEnabled)
  {
    // Seed the filter with the first reading, otherwise it ramps up from 0 (which looks like a short)
    oilTempState.adc = oilTempState.hasReading ? LOW_PASS_FILTER(adc, page15.oilTempFilter, oilTempState.adc) : adc;
    oilTempState.hasReading = true;
    oilTempState.isFault = (oilTempState.adc < FAULT_ADC_LOW) || (oilTempState.adc > FAULT_ADC_HIGH);
    oilTempState.temperature = temperatureRemoveOffset(table2D_getValue(&oilTempCalibrationTable, oilTempState.adc));

    bool isHot = isOilHot(page15);
    if (isHot != oilTempState.isHot) { writeAlertOutput(isHot, page15); }
    oilTempState.isHot = isHot;
  }
}

uint8_t getOilTempLogValue(void)
{
  return oilTempState.hasReading ? temperatureAddOffset(oilTempState.temperature) : 0U;
}

bool isOilTempHot(void)
{
  return oilTempState.isHot;
}

uint8_t getOilTempLogStatus(void)
{
  return (uint8_t)((oilTempState.isHot ? 1U : 0U) | ((oilTempState.isFault ? 1U : 0U) << 1U));
}
