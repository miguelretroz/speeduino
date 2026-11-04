#pragma once

#include <stdint.h>
#include "config_pages.h"

namespace valet {

namespace details {

  struct state_t
  {
    uint8_t level = 0U;        ///< Selected (0 = off)
    uint8_t inForce = 0U;      ///< The level whose limits apply (differs from level while pending)
    bool unlockArmed = false;  ///< 0x4310 seen: one lower level / off is accepted until unlockUntilMs
    uint32_t unlockUntilMs = 0U;
    bool boostOver = false;    ///< Open loop boost cut, with hysteresis
  };

  /** @brief Level encoding in EEPROM: a marked value, so a blank or random byte reads as off */
  constexpr uint8_t EEPROM_MARK = 0xA0U;

  /** @brief Default limits of the 3 levels (Light, Medium, Strong): launch and flat shift blocked in all of them */
  void setDefaults(config15 &page15);

} // namespace details

} // namespace valet
