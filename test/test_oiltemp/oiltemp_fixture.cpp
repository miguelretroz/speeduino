#include "oiltemp_fixture.h"
#include "units.h"

oiltemp_context_t setup_oiltemp(void)
{
    oiltemp_context_t context;
    setDefaults(context.page15);
    setDefaultCalibration();
    context.page15.oilTempEnable = 1U;
    context.pins.pinOilTemp = A9;
    context.pins.pinOilTempAlert = TEST_ALERT_PIN;
    context.initialise();
    return context;
}

void setFlatCalibration(int16_t temperature)
{
    for (uint8_t x = 0U; x < oilTempCalibrationTable.size(); ++x)
    {
        oilTempCalibrationTable.values[x] = temperatureAddOffset(temperature);
        oilTempCalibrationTable.axis[x] = x * 33U;
    }
}
