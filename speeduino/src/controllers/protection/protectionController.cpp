/** @file
 * Engine protection actions, memory and warning output. See protectionController.h. Fork specific.
 * The config is found at the end of page 15 (of type @ref config15).
 */
#include "protectionController.h"
#include "protectionController_details.h"
#include "../../../globals.h"
#include "../../../units.h"
#include "../../../unit_testing.h"
#include "../../../preprocessor.h"
#include "../oilTemp/oilTempController.h"
#include "../../pins/pinMapping.h"

// A few bit tests per loop: optimize for size.
#pragma GCC optimize("Os")

using namespace protection::details;

TESTABLE_STATIC state_t protectionState;

/** @brief The conditions found by upstream's checkEngineProtection() */
static constexpr uint8_t UPSTREAM_CONDITIONS = (1U << PROTECT_OIL_PRESSURE) | (1U << PROTECT_BOOST) | (1U << PROTECT_AFR);

namespace protection {

namespace details {

void setDefaults(config15 &page15)
{
  page15.protWarnPin = 0U;
  page15.protUnused1 = 0U;
  page15.protFuelPressRpm = 5U;    // 500 RPM: past cranking
  page15.protFuelPressDelay = 10U; // 1S
  page15.protFuelPressMin = 30U;   // 30 PSI across the injectors
  page15.protCoolantHot = temperatureAddOffset(110);
  page15.protCoolantHotEnable = 0U;
  page15.protOilHotEnable = 0U;
  page15.protFuelPressEnable = 0U;
  page15.protSensorFaultEnable = 0U;
  page15.protWarnInv = 0U;
  page15.protUnused2 = 0U;
  page15.protLatch = 0U;
  // Upstream's protections keep doing what they did
  page15.protActionOilPress = (uint8_t)ProtectAction::LimitRpm;
  page15.protActionBoost = (uint8_t)ProtectAction::LimitRpm;
  page15.protActionAfr = (uint8_t)ProtectAction::LimitRpm;
  // Temperatures warn, the ones that can hurt the engine right away limit it
  page15.protActionCoolantHot = (uint8_t)ProtectAction::WarnOnly;
  page15.protActionOilHot = (uint8_t)ProtectAction::WarnOnly;
  page15.protActionFuelPress = (uint8_t)ProtectAction::LimitRpm;
  page15.protActionSensorFault = (uint8_t)ProtectAction::LimitRpm;
  page15.protUnused3 = 0U;
}

ProtectAction getAction(const config15 &page15, uint8_t condition)
{
  uint8_t action;
  switch (condition)
  {
    case PROTECT_OIL_PRESSURE:  action = page15.protActionOilPress; break;
    case PROTECT_BOOST:         action = page15.protActionBoost; break;
    case PROTECT_AFR:           action = page15.protActionAfr; break;
    case PROTECT_COOLANT_HOT:   action = page15.protActionCoolantHot; break;
    case PROTECT_OIL_HOT:       action = page15.protActionOilHot; break;
    case PROTECT_FUEL_PRESSURE: action = page15.protActionFuelPress; break;
    case PROTECT_SENSOR_FAULT:  action = page15.protActionSensorFault; break;
    default:                    action = (uint8_t)ProtectAction::WarnOnly; break;
  }
  return (action > (uint8_t)ProtectAction::StopEngine) ? ProtectAction::WarnOnly : (ProtectAction)action;
}

int16_t fuelPressureDifferential(const statuses &current)
{
  int32_t boostKpa = (int32_t)current.MAP - (int32_t)current.baro;
  int32_t boostPsi = (boostKpa * KPA_TO_PSI_MUL) / KPA_TO_PSI_DIV; // Towards zero either way: boost or vacuum
  return (int16_t)((int32_t)current.fuelPressure - boostPsi);
}

} // namespace details

} // namespace protection

static void writeWarnOutput(bool isOn, const config15 &page15)
{
  protectionState.warnOutputOn = isOn;
  if (protectionState.warnPin.isValid())
  {
    if (isOn != (page15.protWarnInv != 0U)) { protectionState.warnPin.setPinHigh(); }
    else { protectionState.warnPin.setPinLow(); }
  }
}

void __attribute__((optimize("Os"))) initialiseProtection(const config15 &page15, const pinNumbers_t &pins)
{
  protectionState = state_t();
  if ((pins.pinProtectWarn != NOT_A_PIN) && !pinIsReserved(pins.pinProtectWarn))
  {
    protectionState.warnPin.setPin(pins.pinProtectWarn, OUTPUT);
    writeWarnOutput(false, page15);
  }
}

static uint8_t upstreamConditions(const statuses::engine_protect_flags_t &flags)
{
  return (uint8_t)((flags.oil ? (1U << PROTECT_OIL_PRESSURE) : 0U)
                 | (flags.boostCut ? (1U << PROTECT_BOOST) : 0U)
                 | (flags.afr ? (1U << PROTECT_AFR) : 0U));
}

/** @brief Upstream applies the limit whenever one of its flags is on: "warn only" means clearing the flag */
static void clearUpstreamFlag(statuses::engine_protect_flags_t &flags, uint8_t condition)
{
  switch (condition)
  {
    case PROTECT_OIL_PRESSURE: flags.oil = false; break;
    case PROTECT_BOOST:        flags.boostCut = false; break;
    case PROTECT_AFR:          flags.afr = false; break;
    default: break;
  }
}

void protectionResolve(statuses &current, const config6 &page6, const config15 &page15)
{
  statuses::engine_protect_flags_t &flags = current.engineProtect;
  uint8_t present = upstreamConditions(flags) | protectionState.forkConditions;
  protectionState.upstream = present & UPSTREAM_CONDITIONS;
  // Memory: a condition with memory stays until the engine stops (cleared in protectionControl)
  protectionState.latched |= (present & page15.protLatch);
  uint8_t effective = present | protectionState.latched;

  bool limit = false;
  bool stop = false;
  uint8_t action = ACTION_NONE;
  for (uint8_t condition = 0U; condition < PROTECT_CONDITIONS; ++condition)
  {
    if (BIT_CHECK(effective, condition))
    {
      switch (getAction(page15, condition))
      {
        case ProtectAction::WarnOnly:
          clearUpstreamFlag(flags, condition);
          action = (std::max)(action, ACTION_WARN);
          break;
        case ProtectAction::StopEngine:
          stop = true;
          action = ACTION_STOP;
          break;
        case ProtectAction::LimitRpm:
        default:
          limit = true;
          action = (std::max)(action, ACTION_LIMIT);
          break;
      }
    }
  }

  // Nothing is cut with the protection cut off, as upstream. The warnings still show.
  bool canCut = (page6.engineProtectType != PROTECT_CUT_OFF);
  flags.forkLimit = limit && canCut;
  flags.forkStop = stop && canCut;
  if (!canCut && (action > ACTION_WARN)) { action = ACTION_WARN; }

  protectionState.active = effective;
  protectionState.action = action;
}

static void countUp(uint8_t &counter)
{
  if (counter < UINT8_MAX) { ++counter; }
}

static bool isCoolantHot(const statuses &current, const config15 &page15)
{
  bool isHot = false;
  if (page15.protCoolantHotEnable != 0U)
  {
    int16_t hotTemp = temperatureRemoveOffset(page15.protCoolantHot);
    isHot = protectionState.coolantHot;
    if (current.coolant >= hotTemp) { isHot = true; }
    else if (current.coolant <= (hotTemp - COOLANT_HOT_HYSTERESIS)) { isHot = false; }
    else { /* Hysteresis band: keep the last state */ }
  }
  return isHot;
}

static bool isFuelPressureLow(const statuses &current, const config10 &page10, const config15 &page15)
{
  bool isLow = false;
  if ((page15.protFuelPressEnable != 0U) && (page10.fuelPressureEnable != 0U)
   && (current.RPM > 0U) && (current.RPM >= RPM_COARSE.toUser(page15.protFuelPressRpm))
   && (fuelPressureDifferential(current) < (int16_t)page15.protFuelPressMin))
  {
    countUp(protectionState.fuelPressureLowTicks);
    isLow = (protectionState.fuelPressureLowTicks >= page15.protFuelPressDelay);
  }
  else
  {
    protectionState.fuelPressureLowTicks = 0U;
  }
  return isLow;
}

TESTABLE_STATIC void protectionControlCore(statuses &current, const config6 &page6, const config10 &page10, const config15 &page15, bool oilHot)
{
  bool coolantHot = isCoolantHot(current, page15);
  protectionState.coolantHot = coolantHot;
  bool fuelLow = isFuelPressureLow(current, page10, page15);
  bool oilIsHot = (page15.protOilHotEnable != 0U) && oilHot;
  bool sensorFault = (page15.protSensorFaultEnable != 0U) && (current.sensorFaults != 0U);

  protectionState.forkConditions = (uint8_t)((coolantHot ? (1U << PROTECT_COOLANT_HOT) : 0U)
                                           | (oilIsHot ? (1U << PROTECT_OIL_HOT) : 0U)
                                           | (fuelLow ? (1U << PROTECT_FUEL_PRESSURE) : 0U)
                                           | (sensorFault ? (1U << PROTECT_SENSOR_FAULT) : 0U));

  if (current.rotationStatus == EngineRotationStatus::Stopped)
  {
    // The memory lasts until the engine stops. Upstream only judges its conditions while the engine runs.
    protectionState.latched = 0U;
    current.engineProtect.oil = false;
    current.engineProtect.boostCut = false;
    current.engineProtect.afr = false;
    // Nothing else resolves while the engine is stopped
    protectionResolve(current, page6, page15);
  }

  // Warning output: a protection met or remembered, or a faulted sensor
  writeWarnOutput((protectionState.active != 0U) || (current.sensorFaults != 0U), page15);
}

// LCOV_EXCL_START
void protectionControl(statuses &current, const config6 &page6, const config10 &page10, const config15 &page15)
{
  protectionControlCore(current, page6, page10, page15, isOilTempHot());
}
// LCOV_EXCL_STOP

uint8_t getProtectionLogReason(void)
{
  return (uint8_t)((protectionState.active & 0x7FU) | ((protectionState.latched != 0U) ? 0x80U : 0U));
}

uint8_t getProtectionLogState(void)
{
  return (uint8_t)(protectionState.action | (protectionState.warnOutputOn ? 0x04U : 0U));
}
