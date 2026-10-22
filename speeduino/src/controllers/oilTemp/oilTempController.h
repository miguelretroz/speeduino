#pragma once

/**
 * @file
 * @brief Oil temperature input (2 wire NTC sensor with a pull-up resistor) with a hot oil alert. Fork specific.
 *
 * The sensor is calibrated like the coolant and air temperature sensors: TunerStudio's
 * "Calibrate Temperature Sensors" tool writes @ref oilTempCalibrationTable.
 */

#include "config_pages.h"
#include "src/pins/pinNumbers_t.h"

/** @brief The board pin for an analog input index from the tune (0 = A0), or NOT_A_PIN if the board doesn't have it */
uint8_t getOilTempInputPin(uint8_t analogIndex);

void initialiseOilTemp(const config15 &page15, const pinNumbers_t &pins);

/** @brief Process a new reading of the oil temperature input (raw ADC, 0-1023). Called at 4Hz. */
void oilTempControl(const config15 &page15, uint16_t adc);

/** @brief Oil temperature for the live data/log, with the +40 offset used by all temperatures */
uint8_t getOilTempLogValue(void);

/** @brief The hot oil alert is on (never with a faulted sensor) */
bool isOilTempHot(void);

/**
 * @brief Oil temperature status byte for the live data/log
 *
 * - Bit 0: oil hot alert
 * - Bit 1: sensor fault (open or short circuit)
 */
uint8_t getOilTempLogStatus(void);
