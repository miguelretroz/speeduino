#include <unity.h>
#include "bit_manip.h"
#include "decoders.h"
#include "decoder_init.h"
#include "decoder_builder.h"
#include "../test_utils.h"
#include "globals.h"
#include "decoder_name.h"
#include "pages.h"
#include "page_crc.h"

static void assert_decoder(const decoder_t &decoder)
{
    auto defaultDecoder = decoder_builder_t().build();

    // Secondary is optional
    if (decoder.secondary.edge!=TRIGGER_EDGE_NONE)
    {
        TEST_ASSERT_NOT_EQUAL_MESSAGE(decoder.secondary.callback, defaultDecoder.secondary.callback, "secondary");
        TEST_ASSERT_NOT_EQUAL_MESSAGE(NOT_A_PIN, pinNumbers.pinTrigger2, "pinTrigger2");
    }
    else
    {
        TEST_ASSERT_EQUAL_MESSAGE(decoder.secondary.callback, defaultDecoder.secondary.callback, "secondary");
        TEST_ASSERT_EQUAL_MESSAGE(NOT_A_PIN, pinNumbers.pinTrigger2, "pinTrigger2");
    }
    
    // Tertiary is optional
    if (decoder.tertiary.edge!=TRIGGER_EDGE_NONE)
    {
        TEST_ASSERT_NOT_EQUAL_MESSAGE(decoder.tertiary.callback, defaultDecoder.tertiary.callback, "tertiary");
        TEST_ASSERT_NOT_EQUAL_MESSAGE(NOT_A_PIN, pinNumbers.pinTrigger3, "pinTrigger3");
    }
    else
    {
        TEST_ASSERT_EQUAL_MESSAGE(decoder.tertiary.callback, defaultDecoder.tertiary.callback, "tertiary");
        TEST_ASSERT_EQUAL_MESSAGE(NOT_A_PIN, pinNumbers.pinTrigger3, "pinTrigger3");
    }
    
    // Mandatory
    TEST_ASSERT_NOT_EQUAL_MESSAGE(decoder.getRevolutionTime, defaultDecoder.getRevolutionTime, "getRevolutionTime");
    
    // Mandatory
    TEST_ASSERT_NOT_EQUAL_MESSAGE(decoder.pGetCrankAngle, defaultDecoder.pGetCrankAngle, "getCrankAngle");

    // Per tooth ignition is optional
    if (decoder.getFeatures().supportsPerToothIgnition)
    {
        // TEST_ASSERT_TRUE(configPage2.perToothIgn);
        TEST_ASSERT_NOT_EQUAL_MESSAGE(decoder.setEndTeeth, defaultDecoder.setEndTeeth, "setEndTeeth");
    }
    else
    {
        // TEST_ASSERT_FALSE(configPage2.perToothIgn);
        TEST_ASSERT_EQUAL_MESSAGE(decoder.setEndTeeth, defaultDecoder.setEndTeeth, "setEndTeeth");
    }

    // Mandatory
    TEST_ASSERT_NOT_EQUAL_MESSAGE(decoder.reset, defaultDecoder.reset, "reset");

    // Mandatory
    TEST_ASSERT_NOT_EQUAL_MESSAGE(decoder.isEngineRunning, defaultDecoder.isEngineRunning, "isEngineRunning");

    // Mandatory
    TEST_ASSERT_NOT_EQUAL_MESSAGE(decoder.getStatus, defaultDecoder.getStatus, "getStatus");

    // Mandatory
    TEST_ASSERT_NOT_EQUAL_MESSAGE(decoder.getFeatures, defaultDecoder.getFeatures, "getFeatures");
}

extern decoder_t buildDecoder(uint8_t decoder);

static uint8_t decoderIdentifier;
static void test_buildDecoder(void)
{
    configPage2.perToothIgn = true;
    pinNumbers.pinTrigger = 11;
    pinNumbers.pinTrigger2 = 12;
    pinNumbers.pinTrigger3 = 13;
    assert_decoder(buildDecoder(decoderIdentifier));
}

static void test_buildDecoder_all(void)
{
    configPage2.nCylinders = 4; // Needed to prevent division by zero on Renix.
    for (uint8_t decoder = 0; decoder < DECODER_MAX; ++decoder) {
        decoderIdentifier = decoder;
        auto decoderName = getDecoderName(decoder);
        RUN_TEST_POSTFIX_P(test_buildDecoder, decoderName);

    }
}

static void test_buildDecoder_attachesInterrupts(void)
{
    pinNumbers.pinTrigger = 11;
    pinNumbers.pinTrigger2 = 12;
    pinNumbers.pinTrigger3 = 13;

    configPage4.TrigSpeed = CRANK_SPEED;
    configPage4.sparkMode = IGN_MODE_SEQUENTIAL;
    configPage10.vvt2Enabled = true;

    auto decoder = buildDecoder(DECODER_MISSING_TOOTH);
    assert_decoder(decoder);
    
    TEST_ASSERT_TRUE(decoder.primary.isValid());
    TEST_ASSERT_TRUE(decoder.secondary.isValid());
    TEST_ASSERT_TRUE(decoder.tertiary.isValid());
}

static void test_buildDecoder_TurnsOffPerToothIgn(void)
{
    configPage2.perToothIgn = true;
    auto decoder = buildDecoder(DECODER_MISSING_TOOTH);
    assert_decoder(decoder);
    TEST_ASSERT_TRUE(configPage2.perToothIgn);

    configPage2.perToothIgn = true;
    decoder = buildDecoder(DECODER_HONDA_J32);
    assert_decoder(decoder);
    TEST_ASSERT_FALSE(configPage2.perToothIgn);
}

static void test_buildDecoder_OutOfRange(void)
{
    auto decoder = buildDecoder(DECODER_MAX+1U); // Check this doesn't crash.
    decoder.getCrankAngle();
}

// The loop rebuilds the decoder on every pass while the engine is stopped: the cached page CRCs may only be dropped
// when the build changed a page byte, and must be when it did
static void test_buildDecoder_page_crc(void)
{
    configPage2.nCylinders = 4;
    configPage2.perToothIgn = true;
    (void)buildDecoder(DECODER_MISSING_TOOTH);
    invalidateAllPageCRC32();
    uint32_t veSet = getPageCRC32(veSetPage);
    uint32_t ignSet = getPageCRC32(ignSetPage);

    // Nothing changed: the cache stays (a change behind its back doesn't show)
    configPage2.aeColdPct = configPage2.aeColdPct ^ 0x5AU;
    configPage4.ADCFILTER_TPS = configPage4.ADCFILTER_TPS ^ 0x5AU;
    (void)buildDecoder(DECODER_MISSING_TOOTH);
    TEST_ASSERT_EQUAL_UINT32(veSet, getPageCRC32(veSetPage));
    TEST_ASSERT_EQUAL_UINT32(ignSet, getPageCRC32(ignSetPage));

    // The ST170 setup overwrites the trigger teeth: page 4 is calculated again
    configPage4.triggerTeeth = 60U;
    configPage4.triggerMissingTeeth = 2U;
    invalidateAllPageCRC32();
    ignSet = getPageCRC32(ignSetPage);
    (void)buildDecoder(DECODER_ST170);
    TEST_ASSERT_EQUAL_UINT8(36U, configPage4.triggerTeeth);
    TEST_ASSERT_NOT_EQUAL(ignSet, getPageCRC32(ignSetPage));
    TEST_ASSERT_EQUAL_UINT32(calculatePageCRC32(ignSetPage), getPageCRC32(ignSetPage));

    // Per tooth ignition turned off: page 1 is calculated again
    configPage2.perToothIgn = true;
    invalidateAllPageCRC32();
    veSet = getPageCRC32(veSetPage);
    (void)buildDecoder(DECODER_HONDA_J32);
    TEST_ASSERT_FALSE(configPage2.perToothIgn);
    TEST_ASSERT_NOT_EQUAL(veSet, getPageCRC32(veSetPage));
    TEST_ASSERT_EQUAL_UINT32(calculatePageCRC32(veSetPage), getPageCRC32(veSetPage));
    invalidateAllPageCRC32();
}

void testDecoderInit(void)
{
  SET_UNITY_FILENAME() {
    test_buildDecoder_all();
    RUN_TEST_P(test_buildDecoder_attachesInterrupts);
    RUN_TEST_P(test_buildDecoder_TurnsOffPerToothIgn);
    RUN_TEST_P(test_buildDecoder_OutOfRange);
    RUN_TEST_P(test_buildDecoder_page_crc);
  }
}