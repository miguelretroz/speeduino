#pragma once
#include "config_pages.h"
#include "statuses.h"
#include "src/pins/pinNumbers_t.h"

void initialiseLaunchControl(statuses &current, config6 &page6, const pinNumbers_t &pins);

void updateLaunchAndFlatShift(statuses &current, const config2 &page2, const config6 &page6, const config10 &page10, const config15 &page15);

/** @name Fork: the remote clutch ('E' command, group 0x41)
 * A second clutch input driven over the serial (a steering wheel button through the telemetry transmitter).
 * Each HOLD counts as an engaged clutch for REMOTE_CLUTCH_HOLD_MS and has to be renewed; RELEASE (or silence)
 * frees it. It is OR-ed with the clutch input pin, so launch and flat shift behave exactly as with the pedal
 * switch (same pin settings, same flatSArm split, same limits).
 * @{ */
constexpr uint16_t TS_CMD_REMOTE_CLUTCH_RELEASE = 0x4100U;
constexpr uint16_t TS_CMD_REMOTE_CLUTCH_HOLD    = 0x4101U;
constexpr uint16_t REMOTE_CLUTCH_HOLD_MS        = 400U;
/** @brief Handles an 'E' command. Returns false when it is not from the remote clutch group (0x41). */
bool handleRemoteClutchCommand(uint16_t command);
/** @} */

/** @brief Fork: the clutch input pin alone, without the remote clutch (for the transmitter's start interlock).
 * Read along with the clutch input: only while launch or flat shift is enabled. */
bool isClutchPinPressed(void);
