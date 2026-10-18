#pragma once

/**
 * @file
 * @brief Initialise and upgrade the fork specific config.
 *
 * The fork's fields (end of page 15, extra calibration tables) can't use the upstream data version
 * (@ref CURRENT_DATA_VERSION): that would clash with upstream's own migrations. They have their own
 * version byte in EEPROM instead.
 */

#include "config_pages.h"

/** @brief Apply defaults to new fork fields, after the upstream updates have run */
void doForkUpdates(config15 &page15);
