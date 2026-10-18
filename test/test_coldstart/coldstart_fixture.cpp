#include "coldstart_fixture.h"
#include "fork_updates.h"

void fill_curve(uint8_t (&values)[6], uint8_t value)
{
    for (auto &element : values) { element = value; }
}

coldstart_context_t setup_coldstart(void)
{
    resetFakeForkStorage();
    coldstart_context_t context;
    doForkUpdates(context.page15);
    fakeForkStorage.attemptWrites = 0U;

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
