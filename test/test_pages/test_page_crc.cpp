#include <unity.h>
#include <FastCRC.h>
#include "pages.h"
#include "page_crc.h"
#include "storage.h"
#include "globals.h"
#include "table3d.h"
#include "../test_utils.h"

static void setPageValues_Incremental(uint8_t pageNum, char seedValue)
{
    for (uint16_t offset=0; offset<getPageSize(pageNum); ++offset)
    {
        setPageValue(pageNum, offset, seedValue+(uint8_t)offset);
    }
}

// Every byte with an entity behind it gets a value that is never 0 (bytes with none read 0)
static void setPageValues_NonZero(uint8_t pageNum, uint8_t seedValue)
{
    for (uint16_t offset=0; offset<getPageSize(pageNum); ++offset)
    {
        setPageValue(pageNum, offset, (uint8_t)(((seedValue + offset) % 255U) + 1U));
    }
}

static void setAllPages_Incremental(char seedValue)
{
    for (uint8_t page=MIN_PAGE_NUM; page<MAX_PAGE_NUM; ++page)
    {
        setPageValues_Incremental(page, seedValue+page);
    }
}

// The original algorithm, one getPageValue() per byte: the TS protocol answer must not change
static uint32_t referencePageCRC32(uint8_t pageNum)
{
    FastCRC32 crcCalc;
    byte buffer = getPageValue(pageNum, 0);
    uint32_t crc = crcCalc.crc32(&buffer, 1U);
    for (uint16_t offset=1; offset<getPageSize(pageNum); ++offset)
    {
        buffer = getPageValue(pageNum, offset);
        crc = crcCalc.crc32_upd(&buffer, 1U);
    }
    return crc;
}

// Changes page RAM without setPageValue(), so a stale cache entry can be seen
static void bypassWrite(config_page_t *pPage, uint8_t offset)
{
    ((uint8_t*)pPage)[offset] = ((uint8_t*)pPage)[offset] ^ 0x5AU;
}

static void warmCache(void)
{
    for (uint8_t page=0U; page<MAX_PAGE_NUM; ++page)
    {
        (void)getPageCRC32(page);
    }
}

static void test_calculatePageCRC32(void)
{
    // boostvvtPage2 (12) & wmiMapPage contain multiple types of entities
    // which makes it a good test subjects
    setPageValues_Incremental(boostvvtPage2, 'X');
    TEST_ASSERT_EQUAL_UINT32(0x3C15D3B7, calculatePageCRC32(boostvvtPage2));

    setPageValues_Incremental(wmiMapPage, 'X');
    TEST_ASSERT_EQUAL_UINT32(0xD2F67FBE, calculatePageCRC32(wmiMapPage));
}

static void test_calculatePageCRC32_matches_byte_by_byte(void)
{
    setAllPages_Incremental('A');
    // Includes page 0 (empty) and numbers past the last page
    for (uint8_t page=0U; page<=MAX_PAGE_NUM+2U; ++page)
    {
        TEST_ASSERT_EQUAL_UINT32_MESSAGE(referencePageCRC32(page), calculatePageCRC32(page), "page");
    }
}

static void test_getPageCRC32_matches_calculation(void)
{
    invalidateAllPageCRC32();
    setAllPages_Incremental('c');
    for (uint8_t page=0U; page<=MAX_PAGE_NUM+2U; ++page)
    {
        TEST_ASSERT_EQUAL_UINT32(calculatePageCRC32(page), getPageCRC32(page));
        // Second time from the cache
        TEST_ASSERT_EQUAL_UINT32(calculatePageCRC32(page), getPageCRC32(page));
    }
}

static void test_setPageValue_invalidates_only_that_page(void)
{
    setAllPages_Incremental('k');
    warmCache();
    uint32_t oldIgnSet = getPageCRC32(ignSetPage);

    TEST_ASSERT_TRUE(setPageValue(veSetPage, 3U, getPageValue(veSetPage, 3U) + 1U));
    bypassWrite(&configPage4, 3U);

    // The written page is calculated again...
    TEST_ASSERT_EQUAL_UINT32(calculatePageCRC32(veSetPage), getPageCRC32(veSetPage));
    // ...the other one still answers from the cache (proves only veSetPage was dropped)
    TEST_ASSERT_EQUAL_UINT32(oldIgnSet, getPageCRC32(ignSetPage));
    TEST_ASSERT_NOT_EQUAL(calculatePageCRC32(ignSetPage), getPageCRC32(ignSetPage));

    invalidatePageCRC32(ignSetPage);
    TEST_ASSERT_EQUAL_UINT32(calculatePageCRC32(ignSetPage), getPageCRC32(ignSetPage));
}

static void test_setPageValue_table_page_invalidates(void)
{
    setAllPages_Incremental('t');
    warmCache();
    uint32_t before = getPageCRC32(veMapPage);
    TEST_ASSERT_TRUE(setPageValue(veMapPage, 100U, getPageValue(veMapPage, 100U) + 7U));
    TEST_ASSERT_NOT_EQUAL(before, getPageCRC32(veMapPage));
    TEST_ASSERT_EQUAL_UINT32(calculatePageCRC32(veMapPage), getPageCRC32(veMapPage));
}

static void test_loadAllPages_invalidates_all(void)
{
    setAllPages_Incremental('e');
    warmCache();
    bypassWrite(&configPage2, 1U);
    bypassWrite(&configPage6, 2U);
    bypassWrite(&configPage15, 3U);

    loadAllPages();

    for (uint8_t page=MIN_PAGE_NUM; page<MAX_PAGE_NUM; ++page)
    {
        TEST_ASSERT_EQUAL_UINT32_MESSAGE(calculatePageCRC32(page), getPageCRC32(page), "page");
    }
}

static void test_setTuneToEmpty_invalidates_all(void)
{
    setAllPages_Incremental('z');
    warmCache();
    setTuneToEmpty();
    for (uint8_t page=MIN_PAGE_NUM; page<MAX_PAGE_NUM; ++page)
    {
        TEST_ASSERT_EQUAL_UINT32_MESSAGE(calculatePageCRC32(page), getPageCRC32(page), "page");
    }
}

static void test_invalidate_out_of_range_page(void)
{
    // Must not touch the cache of real pages
    setAllPages_Incremental('o');
    warmCache();
    uint32_t before = getPageCRC32(boostvvtPage2);
    bypassWrite(&configPage15, 10U);
    invalidatePageCRC32(MAX_PAGE_NUM);
    invalidatePageCRC32(UINT8_MAX);
    TEST_ASSERT_EQUAL_UINT32(before, getPageCRC32(boostvvtPage2));
    invalidateAllPageCRC32();
}

static void refreshCalls(uint16_t calls)
{
    for (uint16_t call=0U; call<calls; ++call)
    {
        refreshPageCRC32(PAGE_CRC_REFRESH_BYTES);
    }
}

// Past the hold-off after the last change, plus enough calls for every page (16 bytes a call, sizes total ~3.6 KB,
// one more call per page to start it)
static void refreshAll(void)
{
    refreshCalls(PAGE_CRC_REFRESH_HOLDOFF + 400U);
}

static void test_refresh_fills_the_cache(void)
{
    setAllPages_Incremental('r');
    invalidateAllPageCRC32();
    refreshAll();

    // Everything comes from the cache now: change RAM behind its back and the old CRCs still answer
    uint32_t expected[MAX_PAGE_NUM];
    for (uint8_t page=0U; page<MAX_PAGE_NUM; ++page)
    {
        expected[page] = calculatePageCRC32(page);
    }
    bypassWrite(&configPage2, 4U);
    bypassWrite(&configPage15, 4U);
    for (uint8_t page=0U; page<MAX_PAGE_NUM; ++page)
    {
        TEST_ASSERT_EQUAL_UINT32_MESSAGE(expected[page], getPageCRC32(page), "page");
    }
    invalidateAllPageCRC32();
}

static void test_refresh_restarts_when_the_page_changes(void)
{
    setAllPages_Incremental('s');
    warmCache();
    invalidatePageCRC32(veMapPage);
    // Past the hold-off, then part of the page (288 bytes)
    refreshCalls(PAGE_CRC_REFRESH_HOLDOFF + 5U);
    TEST_ASSERT_TRUE(setPageValue(veMapPage, 10U, getPageValue(veMapPage, 10U) + 3U));
    TEST_ASSERT_TRUE(setPageValue(veMapPage, 250U, getPageValue(veMapPage, 250U) + 3U));
    refreshCalls(PAGE_CRC_REFRESH_HOLDOFF + 40U);
    bypassWrite(&configPage2, 5U); // Not veMapPage: only to check nothing else is recalculated
    TEST_ASSERT_EQUAL_UINT32(calculatePageCRC32(veMapPage), getPageCRC32(veMapPage));
    invalidateAllPageCRC32();
}

static void test_refresh_after_request_mid_way(void)
{
    // A request calculates the page the background is working on: the result stays right
    setAllPages_Incremental('q');
    warmCache();
    invalidatePageCRC32(ignMapPage);
    refreshCalls(PAGE_CRC_REFRESH_HOLDOFF + 2U);
    uint32_t requested = getPageCRC32(ignMapPage);
    refreshCalls(40U);
    TEST_ASSERT_EQUAL_UINT32(calculatePageCRC32(ignMapPage), requested);
    TEST_ASSERT_EQUAL_UINT32(requested, getPageCRC32(ignMapPage));
}

static void test_refresh_waits_for_writes_to_settle(void)
{
    setAllPages_Incremental('w');
    warmCache();

    // Within the hold-off nothing is calculated: a change behind the cache's back still shows
    invalidatePageCRC32(veSetPage);
    refreshCalls(PAGE_CRC_REFRESH_HOLDOFF - 1U);
    bypassWrite(&configPage2, 6U);
    TEST_ASSERT_EQUAL_UINT32(calculatePageCRC32(veSetPage), getPageCRC32(veSetPage));

    // Another change starts the hold-off again
    invalidatePageCRC32(veSetPage);
    refreshCalls(PAGE_CRC_REFRESH_HOLDOFF - 10U);
    TEST_ASSERT_TRUE(setPageValue(afrSetPage, 2U, getPageValue(afrSetPage, 2U) + 1U));
    refreshCalls(PAGE_CRC_REFRESH_HOLDOFF - 1U);
    bypassWrite(&configPage2, 7U);
    TEST_ASSERT_EQUAL_UINT32(calculatePageCRC32(veSetPage), getPageCRC32(veSetPage));

    // After it, the page is calculated in the background: the cache now hides a change behind its back
    invalidatePageCRC32(veSetPage);
    refreshCalls(PAGE_CRC_REFRESH_HOLDOFF + 40U);
    uint32_t refreshed = calculatePageCRC32(veSetPage);
    bypassWrite(&configPage2, 8U);
    TEST_ASSERT_EQUAL_UINT32(refreshed, getPageCRC32(veSetPage));
    invalidateAllPageCRC32();
}

static void test_setPageValue_same_value_keeps_cache(void)
{
    setAllPages_Incremental('u');
    warmCache();
    uint32_t before = getPageCRC32(ignSetPage);
    bypassWrite(&configPage4, 3U); // Only to see whether the cache was dropped

    TEST_ASSERT_TRUE(setPageValue(ignSetPage, 10U, getPageValue(ignSetPage, 10U)));
    TEST_ASSERT_EQUAL_UINT32(before, getPageCRC32(ignSetPage));

    uint8_t same[32];
    getPageValues(ignSetPage, 20U, same, sizeof(same));
    setPageValues(ignSetPage, 20U, same, sizeof(same));
    TEST_ASSERT_EQUAL_UINT32(before, getPageCRC32(ignSetPage));
    invalidateAllPageCRC32();
}

// Windows over every page, crossing entity boundaries and the end of the page
static constexpr uint8_t WINDOW_SIZE = 40U;
static constexpr uint8_t WINDOW_STEP = 23U;

static void test_getPageValues_matches_byte_by_byte(void)
{
    setAllPages_Incremental('g');
    uint8_t buffer[WINDOW_SIZE];
    for (uint8_t page=0U; page<=MAX_PAGE_NUM+1U; ++page)
    {
        for (uint16_t offset=0U; offset<getPageSize(page)+WINDOW_SIZE; offset+=WINDOW_STEP)
        {
            getPageValues(page, offset, buffer, sizeof(buffer));
            for (uint8_t index=0U; index<sizeof(buffer); ++index)
            {
                TEST_ASSERT_EQUAL_UINT8_MESSAGE(getPageValue(page, offset+index), buffer[index], "page byte");
            }
        }
    }
    // An offset near 64K must not wrap around to the start of the page
    (void)memset(buffer, 0xA5, sizeof(buffer));
    getPageValues(veSetPage, UINT16_MAX-10U, buffer, sizeof(buffer));
    for (uint8_t index=0U; index<sizeof(buffer); ++index)
    {
        TEST_ASSERT_EQUAL_UINT8(0U, buffer[index]);
    }
}

static void test_setPageValues_matches_byte_by_byte(void)
{
    uint8_t pattern[WINDOW_SIZE];
    uint8_t before[WINDOW_SIZE+2U];
    for (uint8_t page=MIN_PAGE_NUM; page<MAX_PAGE_NUM; ++page)
    {
        uint16_t pageSize = getPageSize(page);
        for (uint16_t offset=1U; offset<pageSize+WINDOW_SIZE; offset+=WINDOW_STEP)
        {
            // Never 0 before or after, so a byte with no entity behind it (reads 0) can be told apart
            setPageValues_NonZero(page, (uint8_t)offset);
            getPageValues(page, offset-1U, before, sizeof(before));
            for (uint8_t index=0U; index<sizeof(pattern); ++index)
            {
                pattern[index] = (uint8_t)(before[index+1U] == 0xFFU ? 1U : before[index+1U] + 1U);
            }

            setPageValues(page, offset, pattern, sizeof(pattern));

            // The bytes either side are untouched
            TEST_ASSERT_EQUAL_UINT8(before[0], getPageValue(page, offset-1U));
            TEST_ASSERT_EQUAL_UINT8(before[sizeof(before)-1U], getPageValue(page, offset+sizeof(pattern)));
            for (uint8_t index=0U; index<sizeof(pattern); ++index)
            {
                // As setPageValue(): written where an entity is, ignored (reads 0) where none is
                uint8_t expected = (before[index+1U] != 0U) ? pattern[index] : 0U;
                TEST_ASSERT_EQUAL_UINT8_MESSAGE(expected, getPageValue(page, offset+index), "page byte");
            }
        }
    }
    invalidateAllPageCRC32();
}

static void test_setPageValues_invalidates_page_crc(void)
{
    setAllPages_Incremental('v');
    warmCache();
    uint32_t oldIgnSet = getPageCRC32(ignSetPage);
    uint8_t changed[24];
    getPageValues(afrMapPage, 250U, changed, sizeof(changed));
    changed[5] = changed[5] + 1U;
    setPageValues(afrMapPage, 250U, changed, sizeof(changed));
    bypassWrite(&configPage4, 9U);

    TEST_ASSERT_EQUAL_UINT32(calculatePageCRC32(afrMapPage), getPageCRC32(afrMapPage));
    TEST_ASSERT_EQUAL_UINT32(oldIgnSet, getPageCRC32(ignSetPage));
    invalidateAllPageCRC32();
}

static void test_setPageValues_refreshes_table_lookup(void)
{
    // A block write must drop the table's lookup cache, like setPageValue() does
    for (uint8_t index=0U; index<fuelTable.axisX.size(); ++index)
    {
        fuelTable.axisX[index] = (uint8_t)(10U + (index * 10U));
        fuelTable.axisY[index] = (uint8_t)(10U + (index * 10U));
    }
    uint8_t values[64];
    (void)memset(values, 50, sizeof(values));
    for (uint16_t offset=0U; offset<fuelTable.values.size(); offset+=sizeof(values))
    {
        setPageValues(veMapPage, offset, values, sizeof(values));
    }
    TEST_ASSERT_EQUAL_UINT8(50U, get3DTableValue(&fuelTable, 60U, 6000U));

    (void)memset(values, 80, sizeof(values));
    for (uint16_t offset=0U; offset<fuelTable.values.size(); offset+=sizeof(values))
    {
        setPageValues(veMapPage, offset, values, sizeof(values));
    }
    TEST_ASSERT_EQUAL_UINT8(80U, get3DTableValue(&fuelTable, 60U, 6000U));
    invalidateAllPageCRC32();
}

#if defined(__AVR__)
// The Mega answers these inside the main loop: keep them short (the simulator counts cycles, so this is exact)
static void test_page_access_time(void)
{
    char message[64];
    setAllPages_Incremental('h');
    invalidateAllPageCRC32();

    uint32_t worstCalc = 0U;
    for (uint8_t page=MIN_PAGE_NUM; page<MAX_PAGE_NUM; ++page)
    {
        uint32_t start = micros();
        (void)calculatePageCRC32(page);
        worstCalc = (std::max)(worstCalc, micros()-start);
    }
    (void)getPageCRC32(fuelMap2Page);
    uint32_t start = micros();
    (void)getPageCRC32(fuelMap2Page);
    uint32_t cached = micros()-start;

    invalidateAllPageCRC32();
    refreshCalls(PAGE_CRC_REFRESH_HOLDOFF);
    uint32_t worstRefresh = 0U;
    for (uint16_t call=0U; call<400U; ++call)
    {
        start = micros();
        refreshPageCRC32(PAGE_CRC_REFRESH_BYTES);
        worstRefresh = (std::max)(worstRefresh, micros()-start);
    }

    uint8_t block[128];
    start = micros();
    getPageValues(fuelMap2Page, 100U, block, sizeof(block));
    uint32_t read = micros()-start;
    for (uint8_t index=0U; index<sizeof(block); ++index)
    {
        block[index] = block[index] + 1U;
    }
    start = micros();
    setPageValues(fuelMap2Page, 100U, block, sizeof(block));
    uint32_t write = micros()-start;

    (void)snprintf(message, sizeof(message), "us: page crc %lu, cached %lu, refresh %lu, read %lu, write %lu",
                   worstCalc, cached, worstRefresh, read, write);
    TEST_MESSAGE(message);
    TEST_ASSERT_LESS_THAN_UINT32(2000U, worstCalc);
    TEST_ASSERT_LESS_THAN_UINT32(50U, cached);
    TEST_ASSERT_LESS_THAN_UINT32(250U, worstRefresh);
    TEST_ASSERT_LESS_THAN_UINT32(500U, read);
    TEST_ASSERT_LESS_THAN_UINT32(500U, write);
    invalidateAllPageCRC32();
}
#endif

void testPageCrc(void) {
    SET_UNITY_FILENAME() {
        RUN_TEST_P(test_calculatePageCRC32);
        RUN_TEST_P(test_calculatePageCRC32_matches_byte_by_byte);
        RUN_TEST_P(test_getPageCRC32_matches_calculation);
        RUN_TEST_P(test_setPageValue_invalidates_only_that_page);
        RUN_TEST_P(test_setPageValue_table_page_invalidates);
        RUN_TEST_P(test_loadAllPages_invalidates_all);
        RUN_TEST_P(test_setTuneToEmpty_invalidates_all);
        RUN_TEST_P(test_invalidate_out_of_range_page);
        RUN_TEST_P(test_refresh_fills_the_cache);
        RUN_TEST_P(test_refresh_restarts_when_the_page_changes);
        RUN_TEST_P(test_refresh_after_request_mid_way);
        RUN_TEST_P(test_refresh_waits_for_writes_to_settle);
        RUN_TEST_P(test_setPageValue_same_value_keeps_cache);
        RUN_TEST_P(test_getPageValues_matches_byte_by_byte);
        RUN_TEST_P(test_setPageValues_matches_byte_by_byte);
        RUN_TEST_P(test_setPageValues_invalidates_page_crc);
        RUN_TEST_P(test_setPageValues_refreshes_table_lookup);
#if defined(__AVR__)
        RUN_TEST_P(test_page_access_time);
#endif
    }
}
