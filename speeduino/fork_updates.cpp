/** @file
 * Fork specific config initialisation and upgrades. See fork_updates.h
 */
#include "fork_updates.h"
#include "storage.h"
#include "pages.h"
#include "src/controllers/coldStart/coldStartController_details.h"
#include "src/controllers/oilTemp/oilTempController_details.h"
#include "src/controllers/pidTune/pidTuneController_details.h"
#include "src/controllers/sensorFault/sensorFaultController_details.h"
#include "src/controllers/protection/protectionController_details.h"

// Fork config versions. Not small numbers, so a blank or random EEPROM byte is unlikely to match one.
static constexpr uint8_t FORK_CONFIG_V1 = 0xC5U; // Cold start (page 15 bytes 206-255)
static constexpr uint8_t FORK_CONFIG_V2 = 0xC6U; // Cold start low battery cut and key on reset (bytes 204-205)
static constexpr uint8_t FORK_CONFIG_V3 = 0xC7U; // Oil temperature (bytes 198-203 and the oil calibration table)
static constexpr uint8_t FORK_CONFIG_V4 = 0xC8U; // PID auto tune and gain adaptation (bytes 186-196)
static constexpr uint8_t FORK_CONFIG_V5 = 0xC9U; // Sensor fault detection and engine protection actions (bytes 163-185)

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

static void __attribute__((optimize("Os"))) upgradeToV4(config15 &page15)
{
  pidTuneController::details::setDefaults(page15);
  // The gain adaptation multipliers start at 100%, whatever the EEPROM had
  savePidAdaptIdle(pidTune::GainAdapter::DEFAULT_MULTIPLIER);
  savePidAdaptEgo(pidTune::GainAdapter::DEFAULT_MULTIPLIER);
}

static void __attribute__((optimize("Os"))) upgradeToV5(config15 &page15)
{
  sensorFault::details::setDefaults(page15);
  protection::details::setDefaults(page15);
}

void __attribute__((optimize("Os"))) doForkUpdates(config15 &page15)
{
  uint8_t version = loadForkConfigVersion();
  if (version == FORK_CONFIG_V5) { return; }

  if (version == FORK_CONFIG_V1)
  {
    upgradeToV2(page15);
    version = FORK_CONFIG_V2;
  }
  if (version == FORK_CONFIG_V2)
  {
    upgradeToV3(page15);
    version = FORK_CONFIG_V3;
  }
  if (version == FORK_CONFIG_V3)
  {
    upgradeToV4(page15);
    version = FORK_CONFIG_V4;
  }
  if (version == FORK_CONFIG_V4)
  {
    upgradeToV5(page15);
  }
  else
  {
    // Never initialised: all defaults
    coldStartController::details::setDefaults(page15);
    saveColdStartAttempts(0U);
    upgradeToV3(page15);
    upgradeToV4(page15);
    upgradeToV5(page15);
  }

  savePage(boostvvtPage2);
  saveForkConfigVersion(FORK_CONFIG_V5);
}
