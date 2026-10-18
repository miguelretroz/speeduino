#include "coldstart_fixture.h"
#include "storage.h"
#include "preprocessor.h"

// Must match storage.cpp
constexpr uint16_t EEPROM_COLDSTART_MARKER   = 3672U;
constexpr uint16_t EEPROM_COLDSTART_ATTEMPTS = 3673U;

fake_coldstart_storage_t fakeStorage;

static byte fakeRead(uint16_t address)
{
    if (address == EEPROM_COLDSTART_MARKER) { return fakeStorage.marker; }
    if (address == EEPROM_COLDSTART_ATTEMPTS) { return fakeStorage.attempts; }
    return 0xFFU;
}

static void fakeWrite(uint16_t address, byte value)
{
    if (address == EEPROM_COLDSTART_MARKER) { fakeStorage.marker = value; }
    if (address == EEPROM_COLDSTART_ATTEMPTS)
    {
        fakeStorage.attempts = value;
        ++fakeStorage.attemptWrites;
    }
}

static uint16_t fakeLength(void) { return 4096U; }

static uint16_t fakeWriteBlockSize(const statuses &current) { UNUSED(current); return UINT16_MAX; }

void resetFakeStorage(void)
{
    fakeStorage = fake_coldstart_storage_t();
    setStorageAPI({ .read = fakeRead, .write = fakeWrite, .length = fakeLength, .getMaxWriteBlockSize = fakeWriteBlockSize });
}

void fill_curve(uint8_t (&values)[6], uint8_t value)
{
    for (auto &element : values) { element = value; }
}

coldstart_context_t setup_coldstart(void)
{
    resetFakeStorage();
    coldstart_context_t context;
    initialiseColdStartConfig(context.page15);
    fakeStorage.attemptWrites = 0U;

    context.page15.coldStartEnable = 1U;
    context.page15.coldStartPin = TEST_COLDSTART_PIN;
    context.pins.pinColdStart = TEST_COLDSTART_PIN;
    context.page4.floodClear = 190U; // 95%: above the reset gesture TPS
    context.current.coolant = 10;
    context.current.TPS = 0U;
    context.current.battery10 = BATTERY_USB_ONLY;
    context.current.rotationStatus = EngineRotationStatus::Stopped;
    context.initialise();
    return context;
}
