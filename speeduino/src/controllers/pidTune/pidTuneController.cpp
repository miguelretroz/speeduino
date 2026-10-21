/** @file
 * PID auto tune and gain adaptation for the idle and AFR closed loops. Fork specific.
 * The config is found at the end of page 15 (of type @ref config15).
 */
#include "pidTuneController_details.h"
#include "pidTuneRules.h"

namespace pidTuneController {

namespace details {

void __attribute__((optimize("Os"))) setDefaults(config15 &page15)
{
  page15.pidAdaptEgoBand = 2U;        // 0.2 AFR
  page15.pidAdaptIdleBand = 25U;      // 25 RPM
  page15.pidAdaptWindow = 15U;        // 15S
  page15.pidAdaptStep = 5U;           // 5%
  page15.pidTuneTimeout = 90U;        // 90S
  page15.pidTuneCycles = 3U;
  page15.pidTuneEgoHyst = 1U;         // 0.1 AFR
  page15.pidTuneEgoAmplitude = 5U;    // 5%
  page15.pidTuneIdleHyst = 15U;       // 15 RPM
  page15.pidTuneIdleAmplitude = 8U;   // 8 steps or 8% duty
  page15.pidAdaptIdle = 0U;
  page15.pidAdaptEgo = 0U;
  page15.pidTuneRule = (uint8_t)pidTune::TuneRule::TyreusLuybenPI;
  page15.pidTuneUnused = 0U;
}

} // namespace details

} // namespace pidTuneController
