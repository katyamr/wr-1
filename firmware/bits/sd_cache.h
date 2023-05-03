#pragma once

#include <stdint.h>
#include "FatStructs.h"

namespace sd_card {

//==============================================================================
/**
   \brief Cache for an SD data block
*/
union cache_t {
    /** Used to access cached file data blocks. */
    uint8_t  data[512];
    /** Used to access cached FAT16 entries. */
    uint16_t fat16[256];
    /** Used to access cached FAT32 entries. */
    uint32_t fat32[128];
    /** Used to access cached directory entries. */
    dir_t    dir[16];
    /** Used to access a cached MasterBoot Record. */
    mbr_t    mbr;
    /** Used to access to a cached FAT boot sector. */
    fbs_t    fbs;
};

}
