#pragma once

/**
 * @file
 * @brief Sensor fault detection with safe fallback values. Fork specific.
 *
 * A sensor whose reading sits at an electrical limit (shorted to ground or disconnected) for a while is
 * faulted: its value is replaced by a fixed safe value from the tune, the fault is shown in the live data and
 * the engine protection can act on it (see protectionController.h). The fault clears once the reading has
 * been back in range for the same time. Covers the coolant, air temperature, MAP and TPS sensors.
 *
 * The fan can be made to run while the coolant sensor is faulted, as the real temperature is unknown.
 */

#include "statuses.h"
#include "config_pages.h"

/** @name Bits of statuses::sensorFaults */
///@{
constexpr uint8_t SENSOR_FAULT_CLT = 0U;
constexpr uint8_t SENSOR_FAULT_IAT = 1U;
constexpr uint8_t SENSOR_FAULT_MAP = 2U;
constexpr uint8_t SENSOR_FAULT_TPS = 3U;
///@}

/**
 * @brief Check the first readings right away, so a sensor broken at power up is faulted before the engine cranks.
 *
 * Call it after the first sensor reads. Faulted sensors get their fallback value here too.
 */
void initialiseSensorFaults(statuses &current, const config15 &page15);

/** @brief Fault detection with the debounce time. Call at 10Hz. */
void sensorFaultControl(statuses &current, const config15 &page15);

/** @brief Feed a raw MAP sensor reading (0-1023). The MAP code drops out of range readings, which are what the detection needs. */
void sensorFaultMapSample(uint16_t adc);

/** @name Fallback values. Each returns the measured value unless the sensor is faulted. */
///@{
int16_t sensorFaultCoolant(int16_t measured, const statuses &current, const config15 &page15);
int16_t sensorFaultIat(int16_t measured, const statuses &current, const config15 &page15);
uint16_t sensorFaultMap(uint16_t measured, const statuses &current, const config15 &page15);
/** @param measured TPS (Unit: 0.5%) */
uint8_t sensorFaultTps(uint8_t measured, const statuses &current, const config15 &page15);
///@}

/** @brief The fan runs because the coolant sensor is faulted */
bool sensorFaultForcesFan(const statuses &current, const config15 &page15);

/**
 * @brief Sensor fault status byte for the live data/log
 *
 * - Bits 0-3: coolant, air temperature, MAP and TPS sensor faults (@ref SENSOR_FAULT_CLT etc.)
 * - Bit 4: fan forced on by a coolant sensor fault
 */
uint8_t getSensorFaultLogStatus(const statuses &current, const config15 &page15);
