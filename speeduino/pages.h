#pragma once
#include <Arduino.h>
#include "table3d.h"
#include "config_pages.h"

/**
 * Page size in bytes
 */
uint16_t getPageSize(uint8_t pageNum /**< [in] The page number */ );

// These are the page numbers that the Tuner Studio serial protocol uses to transverse the different map and config pages.
constexpr uint8_t veSetPage     = 1; //Note that this and the veMapPage were swapped in Feb 2019 as the 'algorithm' field must be declared in the ini before it's used in the fuel table
constexpr uint8_t veMapPage     = 2;
constexpr uint8_t ignMapPage    = 3;
constexpr uint8_t ignSetPage    = 4;
constexpr uint8_t afrMapPage    = 5;
constexpr uint8_t afrSetPage    = 6;
constexpr uint8_t boostvvtPage  = 7;
constexpr uint8_t seqFuelPage   = 8;
constexpr uint8_t canbusPage    = 9;
constexpr uint8_t warmupPage    = 10;
constexpr uint8_t fuelMap2Page  = 11;
constexpr uint8_t wmiMapPage    = 12;
constexpr uint8_t progOutsPage  = 13;
constexpr uint8_t ignMap2Page   = 14;
constexpr uint8_t boostvvtPage2 = 15;
constexpr uint8_t MIN_PAGE_NUM  = veSetPage;
constexpr uint8_t MAX_PAGE_NUM  = (boostvvtPage2+1U);

/** @name Calibration page numbers */
///@{
constexpr uint8_t O2_CALIBRATION_PAGE = MAX_PAGE_NUM+1; ///< O2 sensor calibration table page number
constexpr uint8_t IAT_CALIBRATION_PAGE = O2_CALIBRATION_PAGE+1; ///< IAT sensor calibration table page number
constexpr uint8_t CLT_CALIBRATION_PAGE = IAT_CALIBRATION_PAGE+1; ///< Coolant sensor calibration table page number

/** @name Calibration page sub-indexes */
///@{
constexpr uint8_t PAGE_IDX_CALIBRATION_CRC = 0; ///< Index of a calibration table CRC on a "page"
constexpr uint8_t PAGE_IDX_CALIBRATION_VALUES = 1; ///< Index of a calibration table values on a "page"
constexpr uint8_t PAGE_IDX_CALIBRATION_BINS = 2; ///< Index of a calibration table bins on a "page"
///@}

///@}

/** @brief Fill the tune (config pages & tables) with zeroes. */
void setTuneToEmpty(void);

// ============================== Per-byte page access ==========================

/** @brief Gets a single value from a page, with data aligned as per the ini file */
byte getPageValue(  uint8_t pageNum,       /**< [in] The page number to retrieve data from. */
                    uint16_t pageOffset    /**< [in] The address in the page that should be returned. This is as per the page definition in the ini. */
                    );

/** 
 * @brief Sets a single value from a page, with data aligned as per the ini file 
 * 
 * @returns true if value set, false otherwise
 */
bool setPageValue(  uint8_t pageNum,        /**< [in] The page number to update. */
                    uint16_t pageOffset,    /**< [in] The offset within the page.  */
                    byte value              /**< [in] The new value */
                    );


// ============================== Page Iteration ==========================

// A logical TS page is actually multiple in memory entities. Allow iteration
// over those entities.

// Type of entity
enum class EntityType : uint8_t { 
    Raw,        // A block of memory
    Table,      // A 3D table
    NoEntity,   // No entity, but a valid offset
    End         // The offset was past any known entity for the page
};

/** @brief The *unique* location of an entity within all pages */
struct entity_page_location_t {
    uint8_t page;   // The index of the page the entity belongs to
    uint8_t index;  // The sub-index of the item within the page

    constexpr entity_page_location_t(void)
    : page(0U)
    , index(0U)
    {        
    }
    explicit constexpr entity_page_location_t(uint8_t pageNum, uint8_t pageSubIndex)
    : page(pageNum)
    , index(pageSubIndex)
    {        
    }

    constexpr entity_page_location_t next(void) const
    {
        return entity_page_location_t(page, index+1);
    }

    friend bool operator==(const entity_page_location_t &lhs, const entity_page_location_t &rhs)
    {
        return lhs.page==rhs.page
            && lhs.index==rhs.index;
    }

    friend bool operator!=(const entity_page_location_t &lhs, const entity_page_location_t &rhs)
    {
        return !(lhs==rhs);
    }    
};

struct entity_t
{
    EntityType type; ///< The entity type
    union 
    {
        table3d_t *pTable;      ///< If the entity is a table, this points to the table
        config_page_t *pRaw;    ///< If the entity is a raw block, this points to it
    };
    TableType table_key = TableType::table_type_None; ///< If the entity is a table, this can be used to get the table type
    uint16_t size = 0U;  ///< Size of the entity in bytes *on the page*

    constexpr entity_t(void)
    : type(EntityType::NoEntity)
    , pRaw(nullptr)
    {
    }
    explicit constexpr entity_t(EntityType theType, uint16_t theSize)
    : type(theType)
    , pRaw(nullptr)
    , size(theSize)
    {
    }
   explicit constexpr entity_t(table3d_t *table, TableType key, uint16_t theSize)
    : type(EntityType::Table)
    , pTable(table)
    , table_key(key)
    , size(theSize)
    {
    }
    explicit constexpr entity_t(config_page_t *entity, uint16_t theSize)
    : type(EntityType::Raw)
    , pRaw(entity)
    , size(theSize)
    {
    }

    inline bool isEntityAddressWithin(uint16_t entityAddress) const
    {
        return entityAddress<size;
    }
};

// An entity within a page - needs to include the entity start position
struct page_entity_t : entity_t
{
    uint16_t start; // The start position of the entity, in bytes, from the start of the page

    constexpr page_entity_t(void)
    : start(0U)
    {
    }
    explicit constexpr page_entity_t(const entity_t &entity, uint16_t base)
    : entity_t(entity)
    , start(base)
    {
    }

    /**
     * @brief Check if the offset is within the entity address range
     * 
     * @param pageAddress Address offset from the start of the page
     * @return true if the offset is within the entity address range, false otherwise
     */
    inline bool isPageAddressWithin(uint16_t pageAddress) const
    {
        return pageAddress >= start && pageAddress < start+size;
    }
};

// A entity on a logical page.
struct page_iterator_t {
    page_entity_t entity;
    entity_page_location_t location;

    constexpr page_iterator_t(void) = default;
    constexpr page_iterator_t(const page_entity_t &theEntity, const entity_page_location_t &entityLocation)
    : entity(theEntity)
    , location(entityLocation)    
    {
    }
};

// ============================== Per-byte entity access ==========================

/** @brief Gets a single value from an entity, with data aligned as per the ini file */
byte getEntityValue(const entity_t &entity,  /**< [in] The entity to update */ 
                    uint16_t offset          /**< [in] The offset within the entity */
                    );

/**
 * @brief Where the page bytes of an entity are in memory, in page order: a raw block, or a 3D table (its values, X axis
 * and Y axis are consecutive byte arrays). nullptr: the entity has no data (reads as 0, writes are ignored).
 */
byte* getEntityBytes(const entity_t &entity /**< [in] The entity */);

/** @brief Sets a single value from a page, with data aligned as per the ini file */
bool setEntityValue(entity_t &entity,  /**< [in] The entity to update */ 
                    uint16_t offset,   /**< [in] The offset within the entity */
                    byte value         /**< [in] The new value */
                    );

/**
 * @brief Initiates iteration over a pages entities.
 * 
 * Test `entity.type==End` to determine the end of the page.
 */
page_iterator_t page_begin(uint8_t pageNum /**< [in] The page number to iterate over. */);

/** @brief Moves the iterator to the next sub-entity on the page */
page_iterator_t advance(const page_iterator_t &it /**< [in] The current iterator */);

/**
 * @brief Splits the page bytes [start, end) by entity: calls spanFunc(entity, entityOffset, length) for each piece,
 * in page order.
 *
 * iter is an entity of the page at or before start (page_begin() always works) and is left on the entity holding
 * the last byte, so a caller can continue from there. The next entity is only looked up when there are bytes for it:
 * a call ending on an entity boundary leaves that look up to the next call. Past the last entity the entity type is
 * End (no data).
 */
template <typename TSpanFunc>
void forEachPageSpan(page_iterator_t &iter, uint16_t start, uint16_t end, TSpanFunc spanFunc)
{
    while (start < end)
    {
        if (iter.entity.type == EntityType::End)
        {
            spanFunc(iter.entity, 0U, (uint16_t)(end - start));
            start = end;
        }
        else
        {
            uint16_t entityEnd = iter.entity.start + iter.entity.size;
            if (start < entityEnd)
            {
                uint16_t stop = (end < entityEnd) ? end : entityEnd;
                spanFunc(iter.entity, (uint16_t)(start - iter.entity.start), (uint16_t)(stop - start));
                start = stop;
            }
            else
            {
                iter = advance(iter);
            }
        }
    }
}

/** @brief Copies page bytes to a buffer: the same bytes as getPageValue() for each offset, without looking up the
 * entity for every byte. */
void getPageValues(uint8_t pageNum,     /**< [in] The page number */
                   uint16_t pageOffset, /**< [in] The first page byte */
                   byte *pBuffer,       /**< [out] Receives length bytes */
                   uint16_t length      /**< [in] How many bytes */
                   );

/** @brief Writes page bytes from a buffer, as setPageValue() for each offset (bytes with no entity behind them are
 * ignored), without looking up the entity for every byte. */
void setPageValues(uint8_t pageNum,       /**< [in] The page number */
                   uint16_t pageOffset,   /**< [in] The first page byte */
                   const byte *pBuffer,   /**< [in] length bytes */
                   uint16_t length        /**< [in] How many bytes */
                   );
