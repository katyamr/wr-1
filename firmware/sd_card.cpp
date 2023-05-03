#include "spi.h"

#include "sd_card.h"
#include "bits/sd_card.h"

#include <Arduino.h>

namespace {

// card types
enum {
    /** Standard capacity V1 SD card */
    SD_CARD_TYPE_SD1 = 1,
    /** Standard capacity V2 SD card */
    SD_CARD_TYPE_SD2 = 2,
    /** High Capacity SD card */
    SD_CARD_TYPE_SDHC = 3,
};

uint8_t status;
uint8_t type;

bool cache_dirty;
uint32_t cache_block;
uint32_t cache_mirror_block;

}

namespace sd_card {

uint8_t command(uint8_t cmd);
uint8_t command(uint8_t cmd, uint32_t arg);
uint8_t acommand(uint8_t acmd, uint32_t arg);

uint8_t cache_flush(uint8_t blocking) {
    if (!cache_dirty) {
        return true;
    }

    if (!write_block(cache_block, cache.data, blocking)) {
        return false;
    }

    if (!blocking) {
        return true;
    }

    cacheMirrorBlockFlush(blocking);

    cache_dirty = false;
}

uint8_t cache_read(uint32_t block) {
    if (cache_block == block) {
        return true;
    }

    if (!cache_flush()) {
        return false;
    }

    if (!read_data(block, cache.data)) {
        return false;
    }

    cache_block = block;
    cache_dirty = false;

    return true;
}

bool init() {
    uint32_t t0 = millis();

    spi::init();

    // must supply min of 74 clock cycles with CS high.
    for (uint8_t i = 0; i < 10; i++) { spi::recv(); }

    while (command(CMD0) != R1_IDLE_STATE) {
        if (millis() - t0 > SD_INIT_TIMEOUT) {
            error(SD_CARD_ERROR_CMD0);
            goto failed;
        }
    }

    if (command(CMD8, 0x1AA) & R1_ILLEGAL_COMMAND) {
        type = SD_CARD_TYPE_SD1;
    } else {
        // only need last byte of r7 response
        spi::skip(3);

        status = spi::recv();
        Serial.print("  st ");
        Serial.print(status);
        Serial.print("\r\n");

        if (status == 0xAA) {
            type = SD_CARD_TYPE_SD2;
        } else {
            error(SD_CARD_ERROR_CMD8);
            goto failed;
        }
    }

    // initialize card and send host supports SDHC if SD2
    uint32_t arg = type == SD_CARD_TYPE_SD2 ? 0x40000000 : 0;

    while (acommand(ACMD41, arg) != R1_READY_STATE) {
        if (millis() - t0 > SD_INIT_TIMEOUT) {
            error(SD_CARD_ERROR_ACMD41);
            goto failed;
        }
    }

    if (type == SD_CARD_TYPE_SD2) {
        if (command(CMD58) != R1_READY_STATE) {
            error(SD_CARD_ERROR_CMD58);
            goto failed;
        }

        status = spi::recv();
        Serial.print("  st ");
        Serial.print(status);
        Serial.print("\r\n");

        if ((status & 0xC0) == 0xC0) {
            type = SD_CARD_TYPE_SDHC;
        }

        // discard rest of ocr - contains allowed voltage range
        spi::skip(3);
    }

    spi::cs_high();

    Serial.print(millis() - t0);
    Serial.print("\r\n");

    return true;

failed:

    spi::cs_high();

    return false;
}

bool wait(uint32_t t) {
    uint32_t d, t0 = 0;
    for (;;) {
        if (spi::recv() == 0xFF) {
            return true;
        }
        d = millis();
        if (t0 != 0) {
            if (d - t0 > t) {
                return false;
            }
        } else {
            t0 = d;
        }
    }
}

bool wait_start(uint32_t t = SD_READ_TIMEOUT) {
    uint32_t d, t0 = 0;
    while (spi::recv() == 0xFF) {
        d = millis();
        if (t0 != 0) {
            if (d - t0 > t) {
                error(SD_CARD_ERROR_READ_TIMEOUT);
                return false;
            }
        } else {
            t0 = d;
        }
    }

    if (spi::last_data() != DATA_START_BLOCK) {
        error(SD_CARD_ERROR_READ);
        return false;
    }

    return true;
}

uint8_t command(uint8_t cmd) {
    return command(cmd, 0);
}

//------------------------------------------------------------------------------
/** Skip remaining data in a block when in partial block read mode. */
void read_end() {
    if (inBlock_) {
        // skip data and crc
        while (offset_++ < 514) {
            spiRec();
        }

        inBlock_ = 0;
    }
}
//------------------------------------------------------------------------------
// send command and return error code.  Return status
uint8_t command(uint8_t cmd, uint32_t arg) {
    // end read if in partialBlockRead mode
    read_end();

    // select card
    spi::cs_low();

    // wait up to 300 ms if busy
    wait(300);

    // send command
    spi::transfer(cmd | 0x40);

    // send argument
    for (int8_t s = 24; s >= 0; s -= 8) {
        spi::transfer(arg >> s);
    }

    // send CRC
    uint8_t crc = 0xFF;
    if (cmd == CMD0) {
        crc = 0x95;  // correct crc for CMD0 with arg 0
    } else if (cmd == CMD8) {
        crc = 0x87;  // correct crc for CMD8 with arg 0x1AA
    } else {
        crc = 0xFF;
    }
    spi::transfer(crc);

    Serial.print("sd_card::command(");
    Serial.print(cmd);
    Serial.print(",");
    Serial.print(arg);
    Serial.print("): ");

    // wait for response
    for (uint8_t i = 0; i != 0xFF; ++i) {
        status = spi::recv();

        Serial.print(status);

        if ((status & 0x80) == 0) {
            break;
        }

        Serial.print(",");
    }

    Serial.print("\r\n");

    return status;
}

uint8_t acommand(uint8_t acmd, uint32_t arg) {
    command(CMD55);
    return command(acmd, arg);
}

//------------------------------------------------------------------------------
/**
   Read part of a 512 byte block from an SD card.

   \param[in] block Logical block to be read.
   \param[in] offset Number of bytes to skip at start of block
   \param[out] dst Pointer to the location that will receive the data.
   \param[in] count Number of bytes to read
   \return The value one, true, is returned for success and
   the value zero, false, is returned for failure.
*/
uint8_t read_data(uint32_t block,
                  uint16_t offset, uint16_t count, uint8_t* dst) {
    if (count == 0) {
        return true;
    }
    if ((count + offset) > 512) {
        goto fail;
    }
    if (!inBlock_ || block != block_ || offset < offset_) {
        block_ = block;
        // use address if not SDHC card
        if (type != SD_CARD_TYPE_SDHC) {
            block <<= 9;
        }
        if (command(CMD17, block)) {
            error(SD_CARD_ERROR_CMD17);
            goto fail;
        }
        if (!wait_start()) {
            goto fail;
        }

        offset_ = 0;
        inBlock_ = 1;
    }

    // skip data before offset
    spi::skip(offset - offset_);
    offset_ = offset;

    // transfer data
    spi::recv(dst, count);

    offset_ += count;
    if (!partialBlockRead_ || offset_ >= 512) {
        // read rest of data, checksum and set chip select high
        read_end();
        spi::cs_high();
    }
    return true;

  fail:
    chipSelectHigh();
    return false;
}

/**
   Writes a 512 byte block to an SD card.

   \param[in] block Logical block to be written.
   \param[in] src Pointer to the location of the data to be written.
   \param[in] blocking If the write should be blocking.
   \return The value one, true, is returned for success and
   the value zero, false, is returned for failure.
*/
uint8_t write_block(uint32_t block, const uint8_t* src, uint8_t blocking) {
    // use address if not SDHC card
    if (type != SD_CARD_TYPE_SDHC) {
        block <<= 9;
    }
    if (command(CMD24, block)) {
        error(SD_CARD_ERROR_CMD24);
        goto fail;
    }

    spi::send(token);
    spi::send(src, 512);
    spi::send(0xFF);  // dummy crc
    spi::send(0xFF);  // dummy crc

    status = spi::recv();
    if ((status & DATA_RES_MASK) != DATA_RES_ACCEPTED) {
        error(SD_CARD_ERROR_WRITE);
        chipSelectHigh();
        return false;
    }

    if (blocking) {
        // wait for flash programming to complete
        if (!wait(SD_WRITE_TIMEOUT)) {
            error(SD_CARD_ERROR_WRITE_TIMEOUT);
            goto fail;
        }
        // response is r2 so get and check two bytes for nonzero
        if (command(CMD13, 0) || spi::recv()) {
            error(SD_CARD_ERROR_WRITE_PROGRAMMING);
            goto fail;
        }
    }
    chipSelectHigh();
    return true;

  fail:
    chipSelectHigh();
    return false;
}

}

#if 0


// These are required for initialisation and use of sdfatlib
Sd2Card card;
SdVolume volume;
SdFile root;

boolean SDClass::begin(uint8_t csPin) {
  if (root.isOpen()) {
    root.close();
  }

  /*

    Performs the initialisation required by the sdfatlib library.

    Return true if initialization succeeds, false otherwise.

  */
  return card.init(SPI_HALF_SPEED, csPin) &&
         volume.init(card) &&
         root.openRoot(volume);
}

boolean SDClass::exists(const char *filepath) {
    /*

       Returns true if the supplied file path exists.

    */
  return walkPath(filepath, root, callback_pathExists);
}

boolean callback_pathExists(SdFile& parentDir, const char *filePathComponent,
                            boolean /* isLastComponent */, void * /* object */) {
  /*

    Callback used to determine if a file/directory exists in parent
    directory.

    Returns true if file path exists.

  */
  SdFile child;

  boolean exists = child.open(parentDir, filePathComponent, O_RDONLY);

  if (exists) {
    child.close();
  }

  return exists;
}

boolean walkPath(const char *filepath, SdFile& parentDir,
                 boolean(*callback)(SdFile& parentDir,
                                    const char *filePathComponent,
                                    boolean isLastComponent,
                                    void *object),
                 void *object = NULL) {
  /*

     When given a file path (and parent directory--normally root),
     this function traverses the directories in the path and at each
     level calls the supplied callback function while also providing
     the supplied object for context if required.

       e.g. given the path '/foo/bar/baz'
            the callback would be called at the equivalent of
      '/foo', '/foo/bar' and '/foo/bar/baz'.

     The implementation swaps between two different directory/file
     handles as it traverses the directories and does not use recursion
     in an attempt to use memory efficiently.

     If a callback wishes to stop the directory traversal it should
     return false--in this case the function will stop the traversal,
     tidy up and return false.

     If a directory path doesn't exist at some point this function will
     also return false and not subsequently call the callback.

     If a directory path specified is complete, valid and the callback
     did not indicate the traversal should be interrupted then this
     function will return true.

  */


  SdFile subfile1;
  SdFile subfile2;

  char buffer[PATH_COMPONENT_BUFFER_LEN];

  unsigned int offset = 0;

  SdFile *p_parent;
  SdFile *p_child;

  SdFile *p_tmp_sdfile;

  p_child = &subfile1;

  p_parent = &parentDir;

  while (true) {

    boolean moreComponents = getNextPathComponent(filepath, &offset, buffer);

    boolean shouldContinue = callback((*p_parent), buffer, !moreComponents, object);

    if (!shouldContinue) {
      // TODO: Don't repeat this code?
      // If it's one we've created then we
      // don't need the parent handle anymore.
      if (p_parent != &parentDir) {
        (*p_parent).close();
      }
      return false;
    }

    if (!moreComponents) {
      break;
    }

    boolean exists = (*p_child).open(*p_parent, buffer, O_RDONLY);

    // If it's one we've created then we
    // don't need the parent handle anymore.
    if (p_parent != &parentDir) {
      (*p_parent).close();
    }

    // Handle case when it doesn't exist and we can't continue...
    if (exists) {
      // We alternate between two file handles as we go down
      // the path.
      if (p_parent == &parentDir) {
        p_parent = &subfile2;
      }

      p_tmp_sdfile = p_parent;
      p_parent = p_child;
      p_child = p_tmp_sdfile;
    } else {
      return false;
    }
  }

  if (p_parent != &parentDir) {
    (*p_parent).close(); // TODO: Return/ handle different?
  }

  return true;
}

// this little helper is used to traverse paths
SdFile SDClass::getParentDir(const char *filepath, int *index) {
  // get parent directory
  SdFile d1;
  SdFile d2;

  d1.openRoot(volume); // start with the mostparent, root!

  // we'll use the pointers to swap between the two objects
  SdFile *parent = &d1;
  SdFile *subdir = &d2;

  const char *origpath = filepath;

  while (strchr(filepath, '/')) {

    // get rid of leading /'s
    if (filepath[0] == '/') {
      filepath++;
      continue;
    }

    if (! strchr(filepath, '/')) {
      // it was in the root directory, so leave now
      break;
    }

    // extract just the name of the next subdirectory
    uint8_t idx = strchr(filepath, '/') - filepath;
    if (idx > 12) {
      idx = 12;  // don't let them specify long names
    }
    char subdirname[13];
    strncpy(subdirname, filepath, idx);
    subdirname[idx] = 0;

    // close the subdir (we reuse them) if open
    subdir->close();
    if (! subdir->open(parent, subdirname, O_READ)) {
      // failed to open one of the subdirectories
      return SdFile();
    }
    // move forward to the next subdirectory
    filepath += idx;

    // we reuse the objects, close it.
    parent->close();

    // swap the pointers
    SdFile *t = parent;
    parent = subdir;
    subdir = t;
  }

  *index = (int)(filepath - origpath);
  // parent is now the parent directory of the file!
  return *parent;
}

  File SDClass::open(const char *filepath, uint8_t mode) {
    /*

       Open the supplied file path for reading or writing.

       The file content can be accessed via the `file` property of
       the `SDClass` object--this property is currently
       a standard `SdFile` object from `sdfatlib`.

       Defaults to read only.

       If `write` is true, default action (when `append` is true) is to
       append data to the end of the file.

       If `append` is false then the file will be truncated first.

       If the file does not exist and it is opened for writing the file
       will be created.

       An attempt to open a file for reading that does not exist is an
       error.

    */

    int pathidx;

    // do the interactive search
    SdFile parentdir = getParentDir(filepath, &pathidx);
    // no more subdirs!

    filepath += pathidx;

    if (! filepath[0]) {
      // it was the directory itself!
      return File(parentdir, "/");
    }

    // Open the file itself
    SdFile file;

    // failed to open a subdir!
    if (!parentdir.isOpen()) {
      return File();
    }

    if (! file.open(parentdir, filepath, mode)) {
      return File();
    }
    // close the parent
    parentdir.close();

    if ((mode & (O_APPEND | O_WRITE)) == (O_APPEND | O_WRITE)) {
      file.seekSet(file.fileSize());
    }
    return File(file, filepath);
  }

//------------------------------------------------------------------------------
/**
   Open a file or directory by name.

   \param[in] dirFile An open SdFat instance for the directory containing the
   file to be opened.

   \param[in] fileName A valid 8.3 DOS name for a file to be opened.

   \param[in] oflag Values for \a oflag are constructed by a bitwise-inclusive
   OR of flags from the following list

   O_READ - Open for reading.

   O_RDONLY - Same as O_READ.

   O_WRITE - Open for writing.

   O_WRONLY - Same as O_WRITE.

   O_RDWR - Open for reading and writing.

   O_APPEND - If set, the file offset shall be set to the end of the
   file prior to each write.

   O_CREAT - If the file exists, this flag has no effect except as noted
   under O_EXCL below. Otherwise, the file shall be created

   O_EXCL - If O_CREAT and O_EXCL are set, open() shall fail if the file exists.

   O_SYNC - Call sync() after each write.  This flag should not be used with
   write(uint8_t), write_P(PGM_P), writeln_P(PGM_P), or the Arduino Print class.
   These functions do character at a time writes so sync() will be called
   after each byte.

   O_TRUNC - If the file exists and is a regular file, and the file is
   successfully opened and is not read only, its length shall be truncated to 0.

   \note Directory files must be opened read only.  Write and truncation is
   not allowed for directory files.

   \return The value one, true, is returned for success and
   the value zero, false, is returned for failure.
   Reasons for failure include this SdFile is already open, \a difFile is not
   a directory, \a fileName is invalid, the file does not exist
   or can't be opened in the access mode specified by oflag.
*/
uint8_t SdFile::open(SdFile* dirFile, const char* fileName, uint8_t oflag) {
  uint8_t dname[11];
  dir_t* p;

  // error if already open
  if (isOpen()) {
    return false;
  }

  if (!make83Name(fileName, dname)) {
    return false;
  }
  vol_ = dirFile->vol_;
  dirFile->rewind();

  // bool for empty entry found
  uint8_t emptyFound = false;

  // search for file
  while (dirFile->curPosition_ < dirFile->fileSize_) {
    uint8_t index = 0XF & (dirFile->curPosition_ >> 5);
    p = dirFile->readDirCache();
    if (p == NULL) {
      return false;
    }

    if (p->name[0] == DIR_NAME_FREE || p->name[0] == DIR_NAME_DELETED) {
      // remember first empty slot
      if (!emptyFound) {
        emptyFound = true;
        dirIndex_ = index;
        dirBlock_ = SdVolume::cacheBlockNumber_;
      }
      // done if no entries follow
      if (p->name[0] == DIR_NAME_FREE) {
        break;
      }
    } else if (!memcmp(dname, p->name, 11)) {
      // don't open existing file if O_CREAT and O_EXCL
      if ((oflag & (O_CREAT | O_EXCL)) == (O_CREAT | O_EXCL)) {
        return false;
      }

      // open found file
      return openCachedEntry(0XF & index, oflag);
    }
  }
  // only create file if O_CREAT and O_WRITE
  if ((oflag & (O_CREAT | O_WRITE)) != (O_CREAT | O_WRITE)) {
    return false;
  }

  // cache found slot or add cluster if end of file
  if (emptyFound) {
    p = cacheDirEntry(SdVolume::CACHE_FOR_WRITE);
    if (!p) {
      return false;
    }
  } else {
    if (dirFile->type_ == FAT_FILE_TYPE_ROOT16) {
      return false;
    }

    // add and zero cluster for dirFile - first cluster is in cache for write
    if (!dirFile->addDirCluster()) {
      return false;
    }

    // use first entry in cluster
    dirIndex_ = 0;
    p = SdVolume::cacheBuffer_.dir;
  }
  // initialize as empty file
  memset(p, 0, sizeof(dir_t));
  memcpy(p->name, dname, 11);

  // set timestamps
  if (dateTime_) {
    // call user function
    dateTime_(&p->creationDate, &p->creationTime);
  } else {
    // use default date/time
    p->creationDate = FAT_DEFAULT_DATE;
    p->creationTime = FAT_DEFAULT_TIME;
  }
  p->lastAccessDate = p->creationDate;
  p->lastWriteDate = p->creationDate;
  p->lastWriteTime = p->creationTime;

  // force write of entry to SD
  if (!SdVolume::cacheFlush()) {
    return false;
  }

  // open entry in cache
  return openCachedEntry(dirIndex_, oflag);
}
//------------------------------------------------------------------------------


#endif
