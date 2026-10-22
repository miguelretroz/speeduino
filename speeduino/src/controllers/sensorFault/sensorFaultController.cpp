/** @file
 * Sensor fault detection with safe fallback values. See sensorFaultController.h. Fork specific.
 * The config is found at the end of page 15 (of type @ref config15).
 */
#include "sensorFaultController.h"
#include "sensorFaultController_details.h"
#include "../../../globals.h"
#include "../../../units.h"
#include "../../../unit_testing.h"
#include "../../../preprocessor.h"

// Runs a few times per second: optimize for size.
#pragma GCC optimize("Os")

using namespace sensorFault::details;

TESTABLE_STATIC state_t sensorFaultState;

namespace sensorFault {

namespace details {

void setDefaults(config15 &page15)
{
  // Electrical limits, in ADC/4 (0.0195V) units. The NTC sensors have a pull-up: a short reads 0V, an open
  // circuit 5V. MAP and TPS sensors output 0.5V to 4.5V, so leave a wider margin at the ends.
  page15.cltFaultLow = 2U;      // 0.04V
  page15.cltFaultHigh = 253U;   // 4.94V
  page15.iatFaultLow = 2U;
  page15.iatFaultHigh = 253U;
  page15.mapFaultLow = 12U;     // 0.23V
  page15.mapFaultHigh = 250U;   // 4.88V
  page15.tpsFaultLow = 2U;
  page15.tpsFaultHigh = 253U;
  // Fallbacks: a warm engine (little warm up enrichment), a mild day, atmospheric pressure (rich, never lean)
  // and a part throttle that keeps the idle loop, DFCO and flood clear out of the way.
  page15.cltFailsafe = temperatureAddOffset(60);
  page15.iatFailsafe = temperatureAddOffset(30);
  page15.mapFailsafe = 100U;    // 100kPa
  page15.tpsFailsafe = 20U;     // 20%
  page15.sensorFaultDelay = 10U; // 1S
  page15.sensorFaultClt = 0U;
  page15.sensorFaultIat = 0U;
  page15.sensorFaultMap = 0U;
  page15.sensorFaultTps = 0U;
  page15.sensorFaultFan = 1U;
  page15.sensorFaultUnused = 0U;
}

bool isOutOfRange(uint16_t adc, uint8_t low, uint8_t high)
{
  uint8_t adc8 = (uint8_t)(adc >> 2U);
  return (adc8 < low) || (adc8 > high);
}

} // namespace details

} // namespace sensorFault

/** @brief Is the detection of a sensor turned on? */
static bool isEnabled(uint8_t sensor, const config15 &page15)
{
  bool enabled;
  switch (sensor)
  {
    case SENSOR_FAULT_CLT: enabled = (page15.sensorFaultClt != 0U); break;
    case SENSOR_FAULT_IAT: enabled = (page15.sensorFaultIat != 0U); break;
    case SENSOR_FAULT_MAP: enabled = (page15.sensorFaultMap != 0U) && sensorFaultState.hasMapSample; break;
    case SENSOR_FAULT_TPS: enabled = (page15.sensorFaultTps != 0U); break;
    default: enabled = false; break;
  }
  return enabled;
}

/** @brief Is the latest reading of a sensor at an electrical limit? */
static bool isSensorOutOfRange(uint8_t sensor, const statuses &current, const config15 &page15)
{
  bool outOfRange;
  switch (sensor)
  {
    case SENSOR_FAULT_CLT: outOfRange = isOutOfRange(current.cltADC, page15.cltFaultLow, page15.cltFaultHigh); break;
    case SENSOR_FAULT_IAT: outOfRange = isOutOfRange(current.iatADC, page15.iatFaultLow, page15.iatFaultHigh); break;
    case SENSOR_FAULT_MAP: outOfRange = isOutOfRange(sensorFaultState.mapAdc, page15.mapFaultLow, page15.mapFaultHigh); break;
    // The TPS reading is kept as 8 bits already
    case SENSOR_FAULT_TPS: outOfRange = isOutOfRange((uint16_t)current.tpsADC << 2U, page15.tpsFaultLow, page15.tpsFaultHigh); break;
    default: outOfRange = false; break;
  }
  return outOfRange;
}

static void countUp(uint8_t &counter)
{
  if (counter < UINT8_MAX) { ++counter; }
}

/**
 * @brief One check of every sensor
 *
 * @param immediate Fault (or clear) on this check alone, without the debounce time
 */
TESTABLE_STATIC void sensorFaultCheck(statuses &current, const config15 &page15, bool immediate)
{
  for (uint8_t sensor = 0U; sensor < SENSOR_COUNT; ++sensor)
  {
    if (isEnabled(sensor, page15))
    {
      if (isSensorOutOfRange(sensor, current, page15))
      {
        countUp(sensorFaultState.outOfRange[sensor]);
        sensorFaultState.inRange[sensor] = 0U;
        if (immediate || (sensorFaultState.outOfRange[sensor] >= page15.sensorFaultDelay)) { BIT_SET(current.sensorFaults, sensor); }
      }
      else
      {
        countUp(sensorFaultState.inRange[sensor]);
        sensorFaultState.outOfRange[sensor] = 0U;
        if (immediate || (sensorFaultState.inRange[sensor] >= page15.sensorFaultDelay)) { BIT_CLEAR(current.sensorFaults, sensor); }
      }
    }
    else
    {
      BIT_CLEAR(current.sensorFaults, sensor);
      sensorFaultState.outOfRange[sensor] = 0U;
      sensorFaultState.inRange[sensor] = 0U;
    }
  }
}

void __attribute__((optimize("Os"))) initialiseSensorFaults(statuses &current, const config15 &page15)
{
  // Keep the MAP sample taken while the baro was initialised
  uint16_t mapAdc = sensorFaultState.mapAdc;
  bool hasMapSample = sensorFaultState.hasMapSample;
  sensorFaultState = state_t();
  sensorFaultState.mapAdc = mapAdc;
  sensorFaultState.hasMapSample = hasMapSample;
  current.sensorFaults = 0U;

  sensorFaultCheck(current, page15, true);

  // The sensors were read before the faults were known
  current.coolant = sensorFaultCoolant(current.coolant, current, page15);
  current.IAT = sensorFaultIat(current.IAT, current, page15);
  current.MAP = sensorFaultMap(current.MAP, current, page15);
  current.TPS = sensorFaultTps(current.TPS, current, page15);
  current.TPSlast = current.TPS;
}

void sensorFaultControl(statuses &current, const config15 &page15)
{
  sensorFaultCheck(current, page15, false);
}

void sensorFaultMapSample(uint16_t adc)
{
  sensorFaultState.mapAdc = adc;
  sensorFaultState.hasMapSample = true;
}

int16_t sensorFaultCoolant(int16_t measured, const statuses &current, const config15 &page15)
{
  return BIT_CHECK(current.sensorFaults, SENSOR_FAULT_CLT) ? temperatureRemoveOffset(page15.cltFailsafe) : measured;
}

int16_t sensorFaultIat(int16_t measured, const statuses &current, const config15 &page15)
{
  return BIT_CHECK(current.sensorFaults, SENSOR_FAULT_IAT) ? temperatureRemoveOffset(page15.iatFailsafe) : measured;
}

uint16_t sensorFaultMap(uint16_t measured, const statuses &current, const config15 &page15)
{
  return BIT_CHECK(current.sensorFaults, SENSOR_FAULT_MAP) ? (uint16_t)page15.mapFailsafe : measured;
}

uint8_t sensorFaultTps(uint8_t measured, const statuses &current, const config15 &page15)
{
  // The tune holds whole percent, the status 0.5% units
  return BIT_CHECK(current.sensorFaults, SENSOR_FAULT_TPS) ? (uint8_t)((std::min)(page15.tpsFailsafe, (uint8_t)100U) * 2U) : measured;
}

bool sensorFaultForcesFan(const statuses &current, const config15 &page15)
{
  return (page15.sensorFaultFan != 0U) && BIT_CHECK(current.sensorFaults, SENSOR_FAULT_CLT);
}

uint8_t getSensorFaultLogStatus(const statuses &current, const config15 &page15)
{
  return (uint8_t)((current.sensorFaults & 0x0FU) | (sensorFaultForcesFan(current, page15) ? 0x10U : 0U));
}
