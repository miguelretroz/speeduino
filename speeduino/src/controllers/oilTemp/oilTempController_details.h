#pragma once

#include <stdint.h>
#include "config_pages.h"
#include "../../pins/outputPin.h"

namespace oilTempController {

namespace details {

  /** @brief ADC readings below this are a shorted sensor */
  constexpr uint16_t FAULT_ADC_LOW = 10U;
  /** @brief ADC readings above this are an open circuit (disconnected sensor) */
  constexpr uint16_t FAULT_ADC_HIGH = 1013U;

  struct state_t
  {
    outputPin_t alertPin;
    bool isEnabled = false;   ///< Enabled with a valid input pin
    bool hasReading = false;  ///< At least one reading has been processed
    bool isFault = false;
    bool isHot = false;
    uint16_t adc = 0U;        ///< Filtered ADC reading
    int16_t temperature = 0;  ///< Degrees C
  };

  /** @brief Set the oil temperature fields of page 15 to their defaults. Leaves the input disabled. */
  void setDefaults(config15 &page15);

  /** @brief Load the default sensor calibration (TunerStudio's "Bosch CLT/IAT" preset with a 2490 ohm pull-up) */
  void setDefaultCalibration(void);

} // namespace details

} // namespace oilTempController
