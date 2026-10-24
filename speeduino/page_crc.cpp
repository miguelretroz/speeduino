#include <algorithm>
#include <FastCRC.h>
#include "page_crc.h"
#include "pages.h"
#include "bit_manip.h"

// Not on the hot path: size over speed (also keeps the CRC routine from being copied in, ~2 KB on the Mega)
#pragma GCC optimize("Os")

// The CRC of a page is asked by TunerStudio (all pages when it connects) and by anything else on the TS protocol
// (e.g. a dash on the secondary serial). It is answered inside the main loop, so it has to be quick:
// - the page bytes go to the CRC straight from memory, a whole entity at a time (looking the entity up for every
//   byte made a page take 10 to 70 ms on the Mega);
// - the value is cached per page until something changes the page (see invalidatePageCRC32());
// - refreshPageCRC32() recalculates the changed pages, a few bytes per call from the loop, so a request almost always
//   finds the page in the cache.

// Zero length CRC calls (start, finish) still need a real pointer: with a null one, the hardware CRC of the Teensy 3.x
// computes the end of the data as 0 - 4 and reads memory until it faults
static const uint8_t CRC_NO_DATA = 0U;

// Bytes with no entity behind them read as 0: fed to the CRC from this
static constexpr uint8_t ZERO_CHUNK_SIZE = 16U;

static uint32_t crcCache[MAX_PAGE_NUM];
// Bit N set: crcCache[N] holds the CRC of page N
static uint16_t crcCacheValid = 0U;
static_assert(MAX_PAGE_NUM <= 16U, "One valid bit per page");
static constexpr uint16_t ALL_PAGES_VALID = (uint16_t)((1UL << MAX_PAGE_NUM) - 1UL);

// Background refresh: the page being calculated (NO_PAGE: none) and how far it got
static constexpr uint8_t NO_PAGE = UINT8_MAX;
static uint8_t refreshPage = NO_PAGE;
#if CRC_SW
static uint16_t refreshOffset = 0U;
static uint16_t refreshSize = 0U;
static FastCRC32 refreshCrc;
static page_iterator_t refreshEntity;
#endif

// Feeds the page bytes [start, end) to the CRC. iter: as forEachPageSpan().
static void crcPageBytes(FastCRC32 &crcCalc, page_iterator_t &iter, uint16_t start, uint16_t end)
{
  forEachPageSpan(iter, start, end,
    [&crcCalc](const entity_t &entity, uint16_t entityOffset, uint16_t length)
    {
      const byte *pBytes = getEntityBytes(entity);
      if (pBytes != nullptr)
      {
        (void)crcCalc.crc32_upd(pBytes + entityOffset, length);
      }
      else
      {
        const uint8_t zeros[ZERO_CHUNK_SIZE] = { 0U };
        while (length > 0U)
        {
          uint16_t count = (std::min)(length, (uint16_t)ZERO_CHUNK_SIZE);
          (void)crcCalc.crc32_upd(zeros, count);
          length = length - count;
        }
      }
    });
}

// The bytes the CRC covers: an empty page (or a page number past the last one) reads as one 0 byte
static uint16_t crcLength(uint8_t pageNum)
{
  return (std::max)(getPageSize(pageNum), (uint16_t)1U);
}

uint32_t calculatePageCRC32(uint8_t pageNum)
{
  FastCRC32 crcCalc;
  (void)crcCalc.crc32(&CRC_NO_DATA, 0U); // Start a new CRC
  page_iterator_t iter = page_begin(pageNum);
  crcPageBytes(crcCalc, iter, 0U, crcLength(pageNum));
  // Ends the CRC of the bytes fed so far
  return crcCalc.crc32_upd(&CRC_NO_DATA, 0U);
}

uint32_t getPageCRC32(uint8_t pageNum)
{
  if (pageNum >= MAX_PAGE_NUM)
  {
    // Not a real page: one 0 byte, nothing to cache
    return calculatePageCRC32(pageNum);
  }
  if (!BIT_CHECK(crcCacheValid, pageNum))
  {
    crcCache[pageNum] = calculatePageCRC32(pageNum);
    BIT_SET(crcCacheValid, pageNum);
  }
  return crcCache[pageNum];
}

void invalidatePageCRC32(uint8_t pageNum)
{
  if (pageNum < MAX_PAGE_NUM)
  {
    BIT_CLEAR(crcCacheValid, pageNum);
    if (pageNum == refreshPage)
    {
      // Changed half way through: start it again
      refreshPage = NO_PAGE;
    }
  }
}

void invalidateAllPageCRC32(void)
{
  crcCacheValid = 0U;
  refreshPage = NO_PAGE;
}

static uint8_t firstInvalidPage(void)
{
  uint8_t pageNum = 0U;
  while (BIT_CHECK(crcCacheValid, pageNum))
  {
    ++pageNum;
  }
  return pageNum;
}

void refreshPageCRC32(uint8_t maxBytes)
{
  if (crcCacheValid == ALL_PAGES_VALID)
  {
    return;
  }

#if CRC_SW
  if ((refreshPage == NO_PAGE) || BIT_CHECK(crcCacheValid, refreshPage))
  {
    // Nothing in progress, or a request already calculated it
    refreshPage = firstInvalidPage();
    refreshOffset = 0U;
    refreshSize = crcLength(refreshPage);
    refreshEntity = page_begin(refreshPage);
    (void)refreshCrc.crc32(&CRC_NO_DATA, 0U);
    // The look ups above take about as long as the bytes: the bytes go in the next call
    return;
  }

  uint16_t stop = refreshOffset + (std::min)((uint16_t)maxBytes, (uint16_t)(refreshSize - refreshOffset));
  crcPageBytes(refreshCrc, refreshEntity, refreshOffset, stop);
  refreshOffset = stop;

  if (refreshOffset >= refreshSize)
  {
    crcCache[refreshPage] = refreshCrc.crc32_upd(&CRC_NO_DATA, 0U);
    BIT_SET(crcCacheValid, refreshPage);
    refreshPage = NO_PAGE;
  }
#else
  // Hardware CRC (Teensy 3.x): one CRC unit with one state, shared with the serial packets, so a CRC can't be left
  // half way between calls (the loop only calls this with no serial transfer in progress). A whole page is quick on
  // these.
  (void)maxBytes;
  (void)getPageCRC32(firstInvalidPage());
#endif
}
