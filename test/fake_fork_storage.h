#pragma once

#include <stdint.h>

/**
 * @brief Fake EEPROM that only stores the fork specific data. Fork specific.
 *
 * Everything else reads as erased (0xFF) and writes to it are dropped, so the fake fits in the
 * simulator's RAM.
 */
struct fake_fork_storage_t
{
    uint8_t version = 0xFFU;      ///< Fork config version
    uint8_t attempts = 0xFFU;     ///< Cold start attempts
    uint16_t attemptWrites = 0U;
    uint8_t oilCalibration[100];  ///< Oil temperature calibration: CRC, bins, values
};
extern fake_fork_storage_t fakeForkStorage;

/** @brief Install the fake EEPROM, fully erased */
void resetFakeForkStorage(void);

/** @brief The fork config versions written by the firmware */
constexpr uint8_t FORK_CONFIG_V1 = 0xC5U;
constexpr uint8_t FORK_CONFIG_V2 = 0xC6U;
constexpr uint8_t FORK_CONFIG_V3 = 0xC7U;
constexpr uint8_t FORK_CONFIG_V4 = 0xC8U;
/** @brief The version every board ends up on */
constexpr uint8_t FORK_CONFIG_LATEST = FORK_CONFIG_V4;
