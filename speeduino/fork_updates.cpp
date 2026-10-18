/** @file
 * Fork specific config initialisation and upgrades. See fork_updates.h
 */
#include "fork_updates.h"
#include "storage.h"
#include "pages.h"
#include "src/controllers/coldStart/coldStartController_details.h"
#include "src/controllers/oilTemp/oilTempController_details.h"

// Fork config versions. Not small numbers, so a blank or random EEPROM byte is unlikely to match one.
static constexpr uint8_t FORK_CONFIG_V1 = 0xC5U; // Cold start (page 15 bytes 206-255)
static constexpr uint8_t FORK_CONFIG_V2 = 0xC6U; // Cold start low battery cut and key on reset (bytes 204-205)
static constexpr uint8_t FORK_CONFIG_V3 = 0xC7U; // Oil temperature (bytes 198-203 and the oil calibration table)

static void __attribute__((optimize("Os"))) upgradeToV2(config15 &page15)
{
  coldStartController::details::setV2Defaults(page15);
}

static void __attribute__((optimize("Os"))) upgradeToV3(config15 &page15)
{
  oilTempController::details::setDefaults(page15);
  oilTempController::details::setDefaultCalibration();
  saveCalibrationTable(SensorCalibrationTable::OilTempSensor);
  saveCalibrationCrc(SensorCalibrationTable::OilTempSensor, 0U);
}

void __attribute__((optimize("Os"))) doForkUpdates(config15 &page15)
{
  uint8_t version = loadForkConfigVersion();
  if (version == FORK_CONFIG_V3) { return; }

  if (version == FORK_CONFIG_V1)
  {
    upgradeToV2(page15);
    version = FORK_CONFIG_V2;
  }
  if (version == FORK_CONFIG_V2)
  {
    upgradeToV3(page15);
  }
  else
  {
    // Never initialised: all defaults
    coldStartController::details::setDefaults(page15);
    saveColdStartAttempts(0U);
    upgradeToV3(page15);
  }

  savePage(boostvvtPage2);
  saveForkConfigVersion(FORK_CONFIG_V3);
}
