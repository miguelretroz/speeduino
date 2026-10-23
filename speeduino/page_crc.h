#pragma once
#include <Arduino.h>

/*
 * Calculates and returns the CRC32 value of a given page of memory
 */
uint32_t calculatePageCRC32(uint8_t pageNum /**< [in] The page number to compute CRC for. */);

/**
 * @brief The CRC32 of a page, as calculatePageCRC32(), from a per page cache.
 *
 * Use this to answer the TS protocol. Anything that changes page bytes after boot without setPageValue()
 * must call invalidatePageCRC32() for that page, or this returns the CRC from before the change.
 */
uint32_t getPageCRC32(uint8_t pageNum /**< [in] The page number to compute CRC for. */);

/** @brief The bytes of a page changed: the next getPageCRC32() for it calculates it again. */
void invalidatePageCRC32(uint8_t pageNum /**< [in] The page number (out of range is ignored). */);

/** @brief Every page may have changed (EEPROM load, migrations, tune reset). */
void invalidateAllPageCRC32(void);

/**
 * @brief Recalculates the invalidated pages in the background, at most maxBytes per call (the whole page on boards
 * with a hardware CRC unit), so getPageCRC32() finds them in the cache.
 */
void refreshPageCRC32(uint8_t maxBytes /**< [in] Page bytes to process in this call. */);

/** @brief Page bytes per refreshPageCRC32() call from the loop (at 1 kHz): about 0.1 ms on the Mega. */
constexpr uint8_t PAGE_CRC_REFRESH_BYTES = 16U;
