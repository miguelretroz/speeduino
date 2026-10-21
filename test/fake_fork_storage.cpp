#include <string.h>
#include "fake_fork_storage.h"
#include "storage.h"
#include "preprocessor.h"

// Must match storage.cpp
constexpr uint16_t EEPROM_FORK_CONFIG_VERSION   = 3672U;
constexpr uint16_t EEPROM_COLDSTART_ATTEMPTS    = 3673U;
constexpr uint16_t EEPROM_CALIBRATION_OIL_START = 3564U;
constexpr uint16_t EEPROM_PIDADAPT_IDLE         = 3666U;
constexpr uint16_t EEPROM_PIDADAPT_EGO          = 3667U;

fake_fork_storage_t fakeForkStorage;

static bool isOilCalibration(uint16_t address)
{
    return (address >= EEPROM_CALIBRATION_OIL_START) && (address < (EEPROM_CALIBRATION_OIL_START + sizeof(fakeForkStorage.oilCalibration)));
}

static byte fakeRead(uint16_t address)
{
    if (address == EEPROM_FORK_CONFIG_VERSION) { return fakeForkStorage.version; }
    if (address == EEPROM_COLDSTART_ATTEMPTS) { return fakeForkStorage.attempts; }
    if (isOilCalibration(address)) { return fakeForkStorage.oilCalibration[address - EEPROM_CALIBRATION_OIL_START]; }
    if (address == EEPROM_PIDADAPT_IDLE) { return fakeForkStorage.pidAdaptIdle; }
    if (address == EEPROM_PIDADAPT_EGO) { return fakeForkStorage.pidAdaptEgo; }
    return 0xFFU;
}

static void fakeWrite(uint16_t address, byte value)
{
    if (address == EEPROM_FORK_CONFIG_VERSION) { fakeForkStorage.version = value; }
    if (address == EEPROM_COLDSTART_ATTEMPTS)
    {
        fakeForkStorage.attempts = value;
        ++fakeForkStorage.attemptWrites;
    }
    if (isOilCalibration(address)) { fakeForkStorage.oilCalibration[address - EEPROM_CALIBRATION_OIL_START] = value; }
    if (address == EEPROM_PIDADAPT_IDLE)
    {
        fakeForkStorage.pidAdaptIdle = value;
        ++fakeForkStorage.pidAdaptWrites;
    }
    if (address == EEPROM_PIDADAPT_EGO)
    {
        fakeForkStorage.pidAdaptEgo = value;
        ++fakeForkStorage.pidAdaptWrites;
    }
}

static uint16_t fakeLength(void) { return 4096U; }

static uint16_t fakeWriteBlockSize(const statuses &current) { UNUSED(current); return UINT16_MAX; }

void resetFakeForkStorage(void)
{
    fakeForkStorage = fake_fork_storage_t();
    memset(fakeForkStorage.oilCalibration, 0xFF, sizeof(fakeForkStorage.oilCalibration));
    setStorageAPI({ .read = fakeRead, .write = fakeWrite, .length = fakeLength, .getMaxWriteBlockSize = fakeWriteBlockSize });
}
