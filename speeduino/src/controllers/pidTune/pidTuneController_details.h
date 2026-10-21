#pragma once

#include <stdint.h>
#include "config_pages.h"

namespace pidTuneController {

namespace details {

  /** @brief Set the PID auto tune and gain adaptation fields of page 15 to their defaults. Adaptation off. */
  void setDefaults(config15 &page15);

} // namespace details

} // namespace pidTuneController
