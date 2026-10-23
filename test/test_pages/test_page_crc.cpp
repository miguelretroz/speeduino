#include <unity.h>
#include <FastCRC.h>
#include "pages.h"
#include "page_crc.h"
#include "storage.h"
#include "globals.h"
#include "../test_utils.h"

static void setPageValues_Incremental(uint8_t pageNum, char seedValue)
{
    for (uint16_t offset=0; offset<getPageSize(pageNum); ++offset)
    {
        setPageValue(pageNum, offset, seedValue+(uint8_t)offset);
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

// Enough background calls for every page: one page finishes per call at most, sizes total ~3.6 KB
static void refreshAll(void)
{
    for (uint16_t call=0U; call<400U; ++call)
    {
        refreshPageCRC32(PAGE_CRC_REFRESH_BYTES);
    }
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
    // Half way through the page (288 bytes)
    for (uint8_t call=0U; call<5U; ++call)
    {
        refreshPageCRC32(PAGE_CRC_REFRESH_BYTES);
    }
    TEST_ASSERT_TRUE(setPageValue(veMapPage, 10U, getPageValue(veMapPage, 10U) + 3U));
    TEST_ASSERT_TRUE(setPageValue(veMapPage, 250U, getPageValue(veMapPage, 250U) + 3U));
    for (uint8_t call=0U; call<40U; ++call)
    {
        refreshPageCRC32(PAGE_CRC_REFRESH_BYTES);
    }
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
    refreshPageCRC32(PAGE_CRC_REFRESH_BYTES);
    uint32_t requested = getPageCRC32(ignMapPage);
    for (uint8_t call=0U; call<40U; ++call)
    {
        refreshPageCRC32(PAGE_CRC_REFRESH_BYTES);
    }
    TEST_ASSERT_EQUAL_UINT32(calculatePageCRC32(ignMapPage), requested);
    TEST_ASSERT_EQUAL_UINT32(requested, getPageCRC32(ignMapPage));
}

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
    }
}
