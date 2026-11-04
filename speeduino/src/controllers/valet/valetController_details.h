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
    bool boostOver = false;    ///< Open loop: the duty at 0, with hysteresis
    bool boostCutting = false; ///< The Valet boost cut is on (with hysteresis)
    uint8_t boostCutPct = 0U;  ///< And how much of it (valetBoostCutPercent)
  };

  /** @brief Level encoding in EEPROM: a marked value, so a blank or random byte reads as off */
  constexpr uint8_t EEPROM_MARK = 0xA0U;

  /** @brief Default limits of the 3 levels (Light, Medium, Strong): launch and flat shift blocked in all of them */
  void setDefaults(config15 &page15);

  /** @brief Config version 7: the boost limits change from absolute kPa to kPa above the ambient pressure (100 = 0,
   * the old 0 = no limit). Only for a board that was on version 6. */
  void upgradeBoostToV7(config15 &page15);

  /** @brief The cut for a pressure and a limit (absolute kPa; 0 = none), with the hysteresis of the last state */
  uint8_t boostCutPercentFor(uint16_t mapKpa, uint16_t limitAbs, bool &cutting);

} // namespace details

} // namespace valet
