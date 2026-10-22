#pragma once

#include <stdint.h>
#include "protectionController.h"
#include "../../pins/outputPin.h"

namespace protection {

namespace details {

  /** @brief The hot coolant condition clears this far below its temperature (C) */
  constexpr int16_t COOLANT_HOT_HYSTERESIS = 5;
  /** @brief kPa to PSI: x 37/256 = 0.1445 (exact: 0.1450) */
  constexpr int32_t KPA_TO_PSI_MUL = 37;
  constexpr int32_t KPA_TO_PSI_DIV = 256;

  /** @name The action in effect, for the live data */
  ///@{
  constexpr uint8_t ACTION_NONE = 0U;
  constexpr uint8_t ACTION_WARN = 1U;
  constexpr uint8_t ACTION_LIMIT = 2U;
  constexpr uint8_t ACTION_STOP = 3U;
  ///@}

  struct state_t
  {
    outputPin_t warnPin;
    uint8_t forkConditions = 0U;      ///< The fork's conditions met now (hot coolant, hot oil, fuel pressure, sensor fault)
    uint8_t upstream = 0U;            ///< Upstream's conditions met at the last resolve (oil pressure, boost, AFR)
    uint8_t latched = 0U;             ///< Conditions remembered until the engine stops
    uint8_t active = 0U;              ///< Conditions met or remembered at the last resolve
    uint8_t action = ACTION_NONE;     ///< The action in effect
    uint8_t fuelPressureLowTicks = 0U; ///< Consecutive 10Hz checks with the fuel pressure below the minimum
    bool coolantHot = false;
    bool warnOutputOn = false;
  };

  /** @brief Set the protection fields of page 15 to their defaults: upstream's behaviour, the new conditions off */
  void setDefaults(config15 &page15);

  /** @brief The action of a condition. An unused value in the tune is "warn only": it can't cut anything. */
  ProtectAction getAction(const config15 &page15, uint8_t condition);

  /**
   * @brief Fuel pressure across the injectors (PSI): the rail pressure minus the boost
   *
   * Vacuum adds to it, boost takes from it, so the check works for manifold referenced and fixed regulators alike.
   */
  int16_t fuelPressureDifferential(const statuses &current);

} // namespace details

} // namespace protection
