#include <unity.h>
#include "globals.h"
#include "comms.h"
#include "comms_legacy.h"
#include "comms_secondary.h"
#include "../test_utils.h"

// Backport of upstream f77aad6f (202501-fixes): the USB and secondary serial share the TunerStudio receive
// state, so once TunerStudio connects over USB the secondary must stop using the TunerStudio protocol.

extern void lockOutSecondaryTunerStudio(const Stream &port, statuses &current);
extern uint8_t currentSecondaryCommand;

static void test_ts_connect_over_usb_locks_out_secondary(void)
{
  statuses current = {};
  current.allowTsOnSecondaryComms = true;

  lockOutSecondaryTunerStudio(Serial, current);

  TEST_ASSERT_FALSE(current.allowTsOnSecondaryComms);
}

static void test_ts_connect_over_secondary_keeps_secondary(void)
{
#if defined(CORE_AVR)
  statuses current = {};
  current.allowTsOnSecondaryComms = true;

  lockOutSecondaryTunerStudio(Serial3, current);

  TEST_ASSERT_TRUE(current.allowTsOnSecondaryComms);
#else
  TEST_IGNORE_MESSAGE("AVR only");
#endif
}

#if defined(CORE_AVR)
// USB is half way through receiving a packet, with nothing waiting on the secondary
static void prepareSecondaryCommand(bool allowTs)
{
  configPage9.secondarySerialProtocol = SECONDARY_SERIAL_PROTO_TUNERSTUDIO;
  currentStatus.allowTsOnSecondaryComms = allowTs;
  pPrimarySerial = &Serial;
  pSecondarySerial = &Serial3;
  serialStatusFlag = SERIAL_RECEIVE_INPROGRESS;
  serialReceiveStartTime = millis();
  serialSecondaryStatusFlag = SERIAL_INACTIVE;
  currentSecondaryCommand = 'Z';
}

static void restoreSerial(void)
{
  pPrimarySerial = &Serial;
  serialStatusFlag = SERIAL_INACTIVE;
  currentStatus.allowTsOnSecondaryComms = true;
}
#endif

static void test_secondary_skips_ts_protocol_when_locked_out(void)
{
#if defined(CORE_AVR)
  prepareSecondaryCommand(false);

  secondserial_Command();

  // The USB receive is left alone, and the secondary byte goes to the standalone commands ('n', 'Q'...)
  TEST_ASSERT_TRUE(pPrimarySerial == &Serial);
  TEST_ASSERT_EQUAL(SERIAL_RECEIVE_INPROGRESS, serialStatusFlag);
  TEST_ASSERT_EQUAL_UINT8(0xFFU, currentSecondaryCommand); // Empty port read
  restoreSerial();
#else
  TEST_IGNORE_MESSAGE("AVR only");
#endif
}

static void test_secondary_uses_ts_protocol_when_allowed(void)
{
#if defined(CORE_AVR)
  prepareSecondaryCommand(true);

  secondserial_Command();

  // Routed through the shared TunerStudio receive, which still has a packet in progress
  TEST_ASSERT_TRUE(pPrimarySerial == &Serial3);
  TEST_ASSERT_EQUAL_UINT8('Z', currentSecondaryCommand);
  restoreSerial();
#else
  TEST_IGNORE_MESSAGE("AVR only");
#endif
}

void testSecondaryTunerStudioLockout(void)
{
  SET_UNITY_FILENAME() {
    RUN_TEST_P(test_ts_connect_over_usb_locks_out_secondary);
    RUN_TEST_P(test_ts_connect_over_secondary_keeps_secondary);
    RUN_TEST_P(test_secondary_skips_ts_protocol_when_locked_out);
    RUN_TEST_P(test_secondary_uses_ts_protocol_when_allowed);
  }
}
