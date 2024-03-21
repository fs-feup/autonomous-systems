/*
**             Copyright 2017 by Kvaser AB, Molndal, Sweden
**                         http://www.kvaser.com
**
** This software is dual licensed under the following two licenses:
** BSD-new and GPLv2. You may use either one. See the included
** COPYING file for details.
**
** License: BSD-new
** ==============================================================================
** Redistribution and use in source and binary forms, with or without
** modification, are permitted provided that the following conditions are met:
**     * Redistributions of source code must retain the above copyright
**       notice, this list of conditions and the following disclaimer.
**     * Redistributions in binary form must reproduce the above copyright
**       notice, this list of conditions and the following disclaimer in the
**       documentation and/or other materials provided with the distribution.
**     * Neither the name of the <organization> nor the
**       names of its contributors may be used to endorse or promote products
**       derived from this software without specific prior written permission.
**
** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
** AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
** IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
** ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
** LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
** CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
** SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
** BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
** IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
** ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
** POSSIBILITY OF SUCH DAMAGE.
**
**
** License: GPLv2
** ==============================================================================
** This program is free software; you can redistribute it and/or modify
** it under the terms of the GNU General Public License as published by
** the Free Software Foundation; either version 2 of the License, or
** (at your option) any later version.
**
** This program is distributed in the hope that it will be useful,
** but WITHOUT ANY WARRANTY; without even the implied warranty of
** MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
** GNU General Public License for more details.
**
** You should have received a copy of the GNU General Public License
** along with this program; if not, write to the Free Software
** Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA
**
**
** IMPORTANT NOTICE:
** ==============================================================================
** This source code is made available for free, as an open license, by Kvaser AB,
** for use with its applications. Kvaser AB does not accept any liability
** whatsoever for any third party patent or other immaterial property rights
** violations that may result from any usage of this source code, regardless of
** the combination of source code and various applications that it can be used
** in, or with.
**
** -----------------------------------------------------------------------------
*/

/*
** Description: Windows version of some of the calls used in logio in firmware
**
** ---------------------------------------------------------------------------
*/

#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include "logio_w32.h"
#include "kvdebug.h"

#define LIO_MAX_NUM_KMF_FILES 2048
#define LIO_TRACKS 1
#define LIO_SET_ZERO {0}
// This will optimize the code for Memorator Professional
#define gTrack 0
#define LIO_SECTOR_CACHE 16
#define LIO_VOL_DATA_BLK 0
#define LIO_FILENAME      "LOGDAT%02d.KMF"
#define LIO_FILENAME_NEW  "LOG%05d.KMF"

#define LIO_SECTORS_PER_MB 2048

Callback guiCallback = NULL;

struct {
  uint32_t first_sector;      // To the driver: first sector no
  uint32_t count;             // To the driver: sector count
  LioResult lres;
  uint8_t buffer[LIO_SECTOR_CACHE][LIO_SECTOR_SIZE];
} w32SectorCache;
uint8_t win32FifoMode = 0;


FILE *lioFiles[LIO_MAX_NUM_KMF_FILES];
char kmf_name[1024] = "";
char kmf_mode[16] = "rb";
uint8_t closeFiles = 0x00;

// For LioVolumeDataBlock
static uint32_t endSector[LIO_TRACKS];
//static uint32_t firstSector[LIO_TRACKS];
static uint32_t startSector[LIO_TRACKS];
static uint16_t totalNumberOfKMFfiles = 0;
static uint32_t gfileSize = 0;


static uint16_t curPayload[LIO_TRACKS];
static LioDataBlock *curDataBlock[LIO_TRACKS];
static uint8_t lioMajorVersion;
static uint8_t lioMinorVersion;

/*
 * =============================================================================
 * lioComputeBitrate
 * =============================================================================
 */
uint32_t lioComputeBitrate (LioTriggerBlockChanInfo *thchan)
{
  uint32_t btrate = thchan->clockInHz;
  uint16_t nQuanta = thchan->prSeg + thchan->phSeg1 + thchan->phSeg2 + 1;

  if (thchan->psc) {
    btrate /= thchan->psc;
  }

  if (nQuanta) {
    btrate /= nQuanta;
  }

  return btrate;
}

/*
 * =============================================================================
 * lioSetInitialValues
 * =============================================================================
 */
static void lioSetInitialValues (void) {
  int k;
  totalNumberOfKMFfiles = 0;
  gfileSize = 0;
  lioMajorVersion       = 0;
  lioMinorVersion       = 0;

  PRINTF(("lioSetInitialValues()\n"));
  for (k=0;k < LIO_TRACKS; k++) {
    curDataBlock[k]              = NULL;
  }
}

/*
 * =============================================================================
 * lioAllocDataBlock
 * =============================================================================
 */
LioResult lioAllocDataBlock (void)
{
  if (curDataBlock[gTrack] != NULL) {
    PRINTF(("lioAllocDataBlock, already allocated\n"));
    return lio_Error;
  }
  curDataBlock[gTrack] = (LioDataBlock *)malloc(LIO_SECTOR_SIZE);
  curPayload[gTrack] = 0;
  return lio_OK;
}

/*
 * =============================================================================
 * lioFreeDataBlock
 * =============================================================================
 */
LioResult lioFreeDataBlock (void)
{
  if (curDataBlock[gTrack] == NULL) {
    PRINTF(("lioFreeDataBlock(%d), not allocated\n",gTrack));
    return lio_Error;
  }
  free(curDataBlock[gTrack]);
  curDataBlock[gTrack] = NULL;
  return lio_OK;
}

/*
 * =============================================================================
 * getCurDataBlock
 *
 * Used by externa programs to access the datablock.
 * =============================================================================
 */
void getCurDataBlock (LioBlock** pData) {
  *pData = (LioBlock*) curDataBlock[gTrack];
}

/*
 * =============================================================================
 * lioInitialize
 * =============================================================================
 */
LioResult lioInitialize (void)
{
  int i;
  lioSetInitialValues();
  for (i = 0; i < LIO_MAX_NUM_KMF_FILES; i++) {
    lioFiles[i] = NULL;
  }
  memset(&w32SectorCache, 0, sizeof(w32SectorCache));

  return lio_OK;
}


/*
 * =============================================================================
 * lioOkToShutdown
 * =============================================================================
 */
LioResult lioOkToShutdown(void)
{
  int i;
  for (i = 0; i < LIO_MAX_NUM_KMF_FILES; i++) {
    if (lioFiles[i] != NULL) {
      fflush(lioFiles[i]);
    }
  }
  return lio_OK;
}

/*
 * =============================================================================
 * lioSetPath_W32
 *
 * Set the path where to look for the KMF-file(s) and the
 * mode that are used to open them
 * =============================================================================
 */
LioResult lioSetPath_W32(const char *path, const char *mode)
{
  if (path) strncpy(kmf_name, path, sizeof(kmf_name) - 1);
  if (mode) strncpy(kmf_mode, mode, sizeof(kmf_mode) - 1);
  return lio_OK;
}

/*
 * =============================================================================
 * lioReadSector
 * =============================================================================
 */
LioResult lioReadSector(uint32_t sectorNum)
{
  uint32_t fileNum = sectorNum / LIO_FILESIZE; // (/* 2 * */ MB_PER_GB * LIO_SECTORS_PER_MB);
  uint32_t fileSector = sectorNum % LIO_FILESIZE; //(/* 2 * */ MB_PER_GB * LIO_SECTORS_PER_MB);
  Crc crc = 0;
  LioBlock *lb = (LioBlock*) curDataBlock[gTrack];

  if (fileNum >= LIO_MAX_NUM_KMF_FILES) {
    PRINTF(("fileNum too big, %u >= %u, line = %d\n",
      fileNum, LIO_MAX_NUM_KMF_FILES, __LINE__));
    return lio_Error;
  }

  if (sectorNum > gfileSize) {
    PRINTF(("sectorNum too big (or reached EOF), %u >= %u, line = %d\n",
      sectorNum, gfileSize, __LINE__));
    return lio_EOF;
  }
  else if (sectorNum == gfileSize) {
    PRINTF(("(EOF %u, line = %d)\n", sectorNum, __LINE__));
    return lio_EOF;
  }

  // Read sectors from cache or disk
  if (w32SectorCache.count != 0 && w32SectorCache.first_sector <= sectorNum &&
      sectorNum <= w32SectorCache.first_sector + w32SectorCache.count - 1) {
      // The selected sector is already loaded; copy to the sector buffer
      //PRINTF(("CACHE: %lu [%lu %lu] -> %lu", sectorNum, w32SectorCache.first_sector,
      //  w32SectorCache.first_sector + w32SectorCache.count - 1, sectorNum - w32SectorCache.first_sector));
      memcpy(lb, &(w32SectorCache.buffer[sectorNum - w32SectorCache.first_sector][0]),LIO_SECTOR_SIZE);
  } else {
    uint32_t stat;
    uint32_t lastSectorInFile = 0;

    if (lioFiles[fileNum] == NULL) {
        PRINTF(("lioFiles[%d] == NULL, line = %d\n", fileNum, __LINE__));
    return lio_Error;
    }

    if (lb == NULL) {
      PRINTF(("buf == NULL, line = %d\n", __LINE__));
      return lio_Error;
    }

    stat = fseeko(lioFiles[fileNum], fileSector * LIO_SECTOR_SIZE, SEEK_SET);
    if (stat != 0) {
      PRINTF(("fseek failed, GLE = %d, line = %d\n", errno, __LINE__));
      return lio_Error;
    }

     // Do not try to read utside the current file
    w32SectorCache.count = LIO_SECTOR_CACHE;
    w32SectorCache.first_sector = sectorNum;

    if (gfileSize != 0) {
      if (fileNum == (uint32_t)(totalNumberOfKMFfiles -1)) {
        // The last file might be smaller than LIO_FILESIZE
        lastSectorInFile = gfileSize;
      } else {
        lastSectorInFile = (fileNum+1)*LIO_FILESIZE;
      }
      if (w32SectorCache.first_sector + w32SectorCache.count > lastSectorInFile) {
        w32SectorCache.count = lastSectorInFile - w32SectorCache.first_sector;
        if (w32SectorCache.count > LIO_SECTOR_CACHE) {
          PRINTF(("Warning: Can't cache %d sectors at sector %u(%u).\n",
            w32SectorCache.count, fileSector, lastSectorInFile));
          w32SectorCache.count = LIO_SECTOR_CACHE;
        }
      }
    }
    stat = (uint32_t)fread(w32SectorCache.buffer, LIO_SECTOR_SIZE, w32SectorCache.count, lioFiles[fileNum]);

    if (stat != w32SectorCache.count) {
      PRINTF(("Warning: Requested %u sectors at sector %u(%u), got %u.\n",
      w32SectorCache.count, fileSector, lastSectorInFile, stat));
      w32SectorCache.count = stat;
      if (stat < 1) {
        PRINTF(("ReadFile failed, GLE = %d, line = %d\n", errno, __LINE__));
        return lio_Error;
      }
    }

    fflush(lioFiles[fileNum]);
    memcpy(lb, &(w32SectorCache.buffer[0][0]),LIO_SECTOR_SIZE);
  }

  crc = crcAdd(lb->data, CRC_INIT, sizeof(lb->data));
  if (crc != lb->crc) {
    PRINTF(("crc error (%04x != %04x), line = %d\n", crc, lb->crc, __LINE__));
   return lio_CRCError;
  }

  return lio_OK;
}

/*
 * =============================================================================
 * lioWriteSector
 * =============================================================================
 */
LioResult lioWriteSector(uint32_t sectorNum)
{
  uint32_t fileNum = sectorNum / LIO_FILESIZE; //(/* 2 * */ MB_PER_GB * LIO_SECTORS_PER_MB);
  uint32_t fileSector = sectorNum % LIO_FILESIZE; //(/* 2 * */ MB_PER_GB * LIO_SECTORS_PER_MB);
  int stat;
  LioBlock *lb = (LioBlock*) curDataBlock[gTrack];


  if (fileNum >= LIO_MAX_NUM_KMF_FILES) {
    PRINTF(("fileNum too big, %u >= %u, line = %d\n",
      fileNum, LIO_MAX_NUM_KMF_FILES, __LINE__));
    return lio_Error;
  }

  if (lioFiles[fileNum] == NULL) {
    PRINTF(("lioFiles[%d] == NULL, line = %d\n", fileNum, __LINE__));
    return lio_Error;
  }
  if (lb == NULL) {
    PRINTF(("lb == NULL, line = %d\n", __LINE__));
    return lio_Error;
  }

  lb->crc = crcAdd(lb->data, CRC_INIT, sizeof(lb->data));

  stat = fseeko(lioFiles[fileNum], fileSector * LIO_SECTOR_SIZE, SEEK_SET);

  if (stat != 0) {
    PRINTF(("fseek failed, GLE = %d, line = %d\n", errno, __LINE__));
    return lio_Error;
  }

  stat = (uint32_t)fwrite(lb, LIO_SECTOR_SIZE, 1, lioFiles[fileNum]);
  if (stat != 1) {
    PRINTF(("WriteFile failed, GLE = %d, line = %d\n", errno, __LINE__));
    return lio_Error;
  }
  // The cache is not valid now.
  w32SectorCache.count = 0;
  return lio_OK;
}

/*
 * =============================================================================
 * lioReadVolumeDataBlock
 * =============================================================================
 */
LioResult lioReadVolumeDataBlock (void)
{
  LioResult stat;
  LioVolumeDataBlock *lvdb = (LioVolumeDataBlock *)curDataBlock[gTrack];

  if (curDataBlock[gTrack] == NULL) {
    PRINTF(("curDataBlock has to be allocated to be reused, line == %d\n", __LINE__));
    return lio_Error;
  }
  if (curPayload[gTrack] != 0) {
    PRINTF(("curPayload has to be zero, line == %d\n", __LINE__));
    return lio_Error;
  }
  memset(lvdb, 0, sizeof(LioVolumeDataBlock));

  stat = lioReadSector(LIO_VOL_DATA_BLK + startSector[gTrack]);
  gfileSize = lvdb->fileSize;
  totalNumberOfKMFfiles = lvdb->numOfKMFFiles;
  endSector[gTrack] = lvdb->fileSize;
  return stat;
}


/*
 * =============================================================================
 * lioGetVersion
 * =============================================================================
 */
void lioGetVersion (uint8_t *major, uint8_t *minor)
{
  *major = lioMajorVersion;
  *minor = lioMinorVersion;
}


/*
 * =============================================================================
 * lioOpenLogfile
 * =============================================================================
 */
LioResult lioOpenLogfile(uint32_t *numSectors)
{
  int i;
  LioResult stat = lio_OK;
  uint32_t numFiles = 0;
  *numSectors = 0;

  // Just temporary
  totalNumberOfKMFfiles = LIO_MAX_NUM_KMF_FILES;

  for (i = 0; (uint32_t)i < totalNumberOfKMFfiles; i++) {
    char filename[512];

    if (strlen(kmf_name) > 0) {
      strcpy(filename, kmf_name);
      sprintf(&filename[strlen(kmf_name)], LIO_FILENAME_NEW, i);
    }
    else {
      sprintf(filename, LIO_FILENAME_NEW, i);
    }

    PRINTF(("fopen: (%s) %s\n",kmf_mode, filename));
    if (closeFiles == 0) {
      lioFiles[i] = fopen(filename, kmf_mode);
      if (lioFiles[i] == NULL) {
        if (strlen(kmf_name) > 0) {
          strcpy(filename, kmf_name);
          sprintf(&filename[strlen(kmf_name)], LIO_FILENAME, i);
        }
        else {
          sprintf(filename, LIO_FILENAME, i);
        }

        PRINTF(("fopen: (%s) %s\n",kmf_mode, filename));
        lioFiles[i] = fopen(filename, kmf_mode);
      }

    }
    if (lioFiles[i] == NULL) {
      PRINTF(("Unable to open file '%s' in mode %s, line = %d\n", filename, kmf_mode,__LINE__));
      return lio_FileError;
    }
    else {
      uint32_t fileSec;
      fseeko (lioFiles[i],0, SEEK_END);
      fileSec = (uint32_t)(ftello (lioFiles[i]) / LIO_SECTOR_SIZE);
      *numSectors += fileSec;
      numFiles++;
      PRINTF(("Sectors read: %u\n",*numSectors));
      PRINTF(("Open: '%s', which is %u sectors\n", filename, fileSec));
    }

    if (i == 0) {
      // There's a Catch 22 here. We need endSector set
      // to be able to get lioReadSector to read it.
      endSector[gTrack] = *numSectors;
      gfileSize = *numSectors;

      stat = lioReadVolumeDataBlock();
      if (stat != lio_OK) {
        PRINTF(("lioReadVolumeDataBlock failed, stat = %d\n", stat));
      }
      else {
        LioVolumeDataBlock *lvdb = (LioVolumeDataBlock *)curDataBlock[gTrack];
        lioMajorVersion = (uint8_t)((lvdb->version & 0xFF00) >> 8);
        lioMinorVersion = (uint8_t)(lvdb->version & 0x00FF);

        if (!lioMajorVersion) {
          lioMajorVersion = lioMinorVersion;
          lioMinorVersion = 0;
        }
        PRINTF(("VDB: version = %u.%u\n", lioMajorVersion, lioMinorVersion));
        PRINTF(("VDB: totalNumberOfKMFfiles = %u\n", totalNumberOfKMFfiles));

        PRINTF(("VDB: endSector = %u\n", endSector[gTrack]));
        PRINTF(("VDB: totalNumberOfKMFfiles = %u\n", totalNumberOfKMFfiles));
      }
      if (!totalNumberOfKMFfiles) {
        PRINTF(("VDB: totalNumberOfKMFfiles wrong, %u\n", totalNumberOfKMFfiles));
        totalNumberOfKMFfiles = 1;
      }
      else if (totalNumberOfKMFfiles > LIO_MAX_NUM_KMF_FILES) {
        PRINTF(("VDB: totalNumberOfKMFfiles wrong, %u\n", totalNumberOfKMFfiles));
        totalNumberOfKMFfiles = LIO_MAX_NUM_KMF_FILES;
      }
    }
  }

  PRINTF(("*numSectors = %d (%u MB)\n", *numSectors, *numSectors/LIO_SECTORS_PER_MB));
  if (gfileSize != *numSectors) {
    PRINTF(("gfileSize (%u) != *numSectors (%u)\n",
                   gfileSize, *numSectors));
    gfileSize = *numSectors;
  }

  closeFiles |= (0x01 << gTrack);

  if (stat != lio_OK) {
    PRINTF(("lioReadVolumeDataBlock failed, stat = %d\n", stat));
  }
  return stat;
}

/*
 * =============================================================================
 * lioOpenLogfile
 * =============================================================================
 */
void lioShutdown(void)
{
  PRINTF(("lioShutdown [%d] 0x%02x\n",gTrack, closeFiles));
  closeFiles &= ~(0x01 << gTrack);
  if (0 == closeFiles) {
    int i;

    for (i = 0; i < LIO_MAX_NUM_KMF_FILES; i++) {
      if (lioFiles[i] != NULL) {
        char filename[512];
        sprintf(filename, LIO_FILENAME_NEW, i);
        PRINTF(("Close: (%s)'%s'\n", kmf_mode, filename));
        fclose(lioFiles[i]);
        lioFiles[i] = NULL;
      }
    }
  }
  // Reset global variables
  curPayload[gTrack] = 0;
}



//===========================================================================
// From diskio_w32.c in m32firm
#define CRC_BITS (8*sizeof(Crc))
int usbSpeed = 0;

const Crc crctab[256] = {
   0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
   0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
   0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
   0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
   0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
   0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
   0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
   0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
   0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
   0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
   0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
   0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
   0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
   0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
   0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
   0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
   0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
   0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
   0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
   0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
   0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
   0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
   0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
   0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
   0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
   0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
   0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
   0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
   0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
   0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
   0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
   0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78
};

/*
 * =============================================================================
 * crcAdd
 * =============================================================================
 */
Crc crcAdd(void *buf, Crc crc, int32_t n)
{
  int i;
  for (i = 0; i < n; i++) {
    crc = (Crc)(crctab[(crc ^ ((uint8_t*)buf)[i]) & 0xff] ^ ((crc >> 8) & ((1<<(CRC_BITS-8))-1)));
  }
  return crc;
}


