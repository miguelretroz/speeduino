#pragma once

#include <stdint.h>
#include "config_pages.h"

namespace sensorFault {

namespace details {

  /** @brief Sensors watched, in the order of the fault bits */
  constexpr uint8_t SENSOR_COUNT = 4U;

  struct state_t
  {
    uint16_t mapAdc = 0U;                    ///< Latest raw MAP reading (0-1023)
    bool hasMapSample = false;
    uint8_t outOfRange[SENSOR_COUNT] = {};   ///< Consecutive 10Hz checks out of range
    uint8_t inRange[SENSOR_COUNT] = {};      ///< Consecutive 10Hz checks in range
  };

  /** @brief Set the sensor fault fields of page 15 to their defaults. Leaves the detection off. */
  void setDefaults(config15 &page15);

  /**
   * @brief Is a reading at an electrical limit?
   *
   * @param adc The reading (0-1023)
   * @param low Readings below this are a fault (Unit: ADC/4)
   * @param high Readings above this are a fault (Unit: ADC/4)
   */
  bool isOutOfRange(uint16_t adc, uint8_t low, uint8_t high);

} // namespace details

} // namespace sensorFault
