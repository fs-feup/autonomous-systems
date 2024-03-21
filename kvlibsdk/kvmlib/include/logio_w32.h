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
** Description: Windows version of some of the calls used in logio in firmaware
**
** -----------------------------------------------------------------------------
*/

#ifdef __cplusplus
extern "C" {
#endif

#include "callback.h"
#include "diskio.h"
#include "compilerassert.h"

#include <stdbool.h>

extern int usbSpeed;
extern Callback guiCallback;

#ifndef MIN
#  define MIN(x, y) (((x) < (y)) ? (x) : (y))
#endif
#ifndef MAX
#  define MAX(x, y) (((x) > (y)) ? (x) : (y))
#endif



#define MAX_NR_OF_CAN_CHANNELS 8

// Used for  identifying blocks easily during development.
// Sectors 0-7 are well defined, sectors over 8: if first 4 bytes
// match the magic cockie, the contents is known, otherwise not (probably data)
#define PRETRIGGER_MAGIC 0x74657250  // hex for Pret
#define TRIGGER_MAGIC    0x67697254  // hex for Trig
#define DATA_MAGIC       0x41544144  // hex for DATA
#define HOUSE_MAGIC      0x53554f48  // hex for HOUS

// Put a housekeeping block a each 4MB
#define LIO_SECTORS_BTW_TRIGGERS 0x1FFFL


#define LIO_SECTOR_SIZE DIO_SECTOR_SIZE

typedef struct liosector_s {
  DioSector     dioSect;
  uint32_t      sNo;
  DioResult     status;
  DioCommand    dioCmd;
  uint16_t      counter;
  uint8_t       autoRelease;
  uint8_t       write;
} LioSector;

// Pointer to the (big!) disk sector buffer.
extern LioSector *sectBuf;

#define SECT_BUF_SIZE MEMSIZE_DISK_FULL

typedef enum {
  lio_OK = 0,
  lio_queueFull = 1,
  lio_CRCError = 2,
  lio_SectorErased = 3,
  lio_FileError = 4,
  lio_DiskError = 5,
  lio_DiskFull_Dir = 6,
  lio_DiskFull_Data = 7,
  lio_EOF = 8,
  lio_SeqError = 9,
  lio_Error = 10,
  lio_FileSystemCorrupt = 11,
  lio_UnsupportedVersion = 12,
  lio_NotImplemented = 13,
  lio_FatalError = 14,
  lio_State = 15
} LioResult;


// Version 1 == 16-byte records; used in Original Memorator.
// Version 2 == 24-byte records; used in Memorator II.
// Version 3 == 24-byte records; used in Memorator Pro.
// Version 4 == 24-byte records; used in Memorator Light.
#define LIO_VERSION 3
#define LIO_VERSION_FD 5

// Used by external programs
#define LIO_MULTITRACK_VERSION 4

typedef struct {
  uint8_t data[LIO_SECTOR_SIZE - sizeof(Crc)];
  Crc     crc;
} LioBlock;
CompilerAssert(sizeof(LioBlock) == LIO_SECTOR_SIZE);

typedef struct {
  uint8_t busStatus; // *CxSTRH
  uint8_t rxErrCounter;
  uint8_t txErrCounter;
  uint8_t transType;
  uint8_t transLineMode;

  uint8_t psc;
  uint8_t prSeg;
  uint8_t phSeg1;
  uint8_t phSeg2;
  uint8_t sjw;
  uint8_t samples;

  uint8_t padding;

  uint32_t clockInHz; // CAN Clock in Hz

  uint8_t padding2[8];
  // maybe add iop-information?

} LioTriggerBlockChanInfo;
CompilerAssert(sizeof(LioTriggerBlockChanInfo) == 24);


// NOTE: There is plenty of room for more information in LioTriggerBlock
typedef struct {
  uint32_t magic;

  // House keeping information
  uint32_t prevWPSecNum;
  uint32_t prevTriggerBlockSecNum;
  uint32_t preTriggerBlockSecNum;
  uint8_t  preTriggerIsWrapped;
  uint8_t  padding0[3];
  // The number of sectors in pre-trigger (alignment problem, se logio.c)
  uint32_t preTriggerSectors;

  // Device information
  uint32_t serialNumber;  // main.c
  uint32_t eanhi;
  uint32_t eanlo;

  uint16_t fwver_build;   // FIRMWARE_BUILD_VERSION
  uint8_t  fwver_major;   // FIRMWARE_MAJOR_VERSION
  uint8_t  fwver_minor;   // FIRMWARE_MINOR_VERSION
  uint8_t  fwver_beta;    // FIRMWARE_BUILD_BETA

  uint8_t  hiresTimerFqMHz;
  uint8_t  hwType;
  uint8_t  channels;

 // The RTC time when this block was written
  uint32_t unixTime;
  uint32_t unixTimeSubsecond; // fractional part of the RTC, (uint32_t)-1 means not in use
  uint8_t  padding1[4];
  // SAFE_READ_HIRES_TIMER_US64
  uint64_t usTime;
  uint64_t timestamp;

  // Disk information
  DioDiskInfo diskinfo;

  LioTriggerBlockChanInfo channel[MAX_NR_OF_CAN_CHANNELS];

  uint8_t  padding2[438 - sizeof(DioDiskInfo) - sizeof(LioTriggerBlockChanInfo)*MAX_NR_OF_CAN_CHANNELS];
  Crc      crc;
} LioTriggerBlock;
CompilerAssert(sizeof(LioTriggerBlock) == LIO_SECTOR_SIZE);


typedef struct {
  uint32_t magic;
  uint32_t prevWPSecNum;
  uint8_t  padding[502];
  Crc      crc;
} LioPreTriggerBlock;
CompilerAssert(sizeof(LioPreTriggerBlock) == LIO_SECTOR_SIZE);


// The space available for CAN messages in a data block
#if !defined(LOG_PERFORMANCE_TEST)
# define LIO_DATA_PAYLOAD_SIZE (LIO_SECTOR_SIZE - 16)
#else
# define LIO_DATA_PAYLOAD_SIZE (LIO_SECTOR_SIZE - 20)
#endif

/*
 * This is a typical block with logged data.
 * payload       - Payload
 * calendarTime  - Calendar time when writing; rt_clock format
 * pretriggerSectorSeqNum - Pretriger sector sequence number
 * sectorSeqNum  - Sector sequence number
 * reserved      - Reserved, must be zero
 * crc           - CRC16 checksum
 */
typedef struct {
  uint8_t  payload[LIO_DATA_PAYLOAD_SIZE];
  uint32_t unixTime;
  uint32_t pretriggerSectorSeqNum;
  uint32_t sectorSeqNum;
#if defined LOG_PERFORMANCE_TEST
  uint16_t rtcTime;
  uint16_t rxlength;
#endif
  uint16_t reserved;
  Crc      crc;
} LioDataBlock;
CompilerAssert(sizeof(LioDataBlock) == LIO_SECTOR_SIZE);

/*
 * reserved      - Reserved, must be zero
 * magicNumber   - Magic number
 * fileSize      - File size, in sectors
 * numOfKMFFiles - Total number of KMF files
 * padding[]
 * crc           - CRC16 checksum
 */
typedef struct {
  uint16_t version; // LIO_VERSION, has to be first in LioVolumeDataBlock
  uint16_t padding0;
  uint32_t reserved;
  uint32_t magicNumber;
  uint32_t fileSize;
  uint16_t numOfKMFFiles;
  uint16_t trackVersion;
  uint16_t tracks;
  uint8_t  trackInfo[LIO_SECTOR_SIZE - 24]; // pair of uint32_t [start, stop] for each track
  Crc      crc;
} LioVolumeDataBlock;
CompilerAssert(sizeof(LioVolumeDataBlock) == LIO_SECTOR_SIZE);

/*
 * lastKnownWPSecNum           - Last known write pointer (sector number)
 * lastKnownTriggerSecNum      - Last known trigger sector (sector number)
 * lastKnownSectorSeqNum       - Last known sector sequence number
 * sectorHighWaterMarkSecNum   - Sector highwater mark (sector number)
 * highWaterHasWrapped         - true if the entire logging area has been used,
 * padding[]
 * fifoMode                    - FIFO mode
 * volumeClean                 - Volume clean
 * VCBWriteSeqNum              - VCB write sequence number
 * crc                         - CRC16 checksum
*/
typedef struct {
  uint32_t lastKnownWPSecNum;
  uint32_t lastKnownTriggerSecNum;
  uint32_t lastKnownSectorSeqNum;
  uint32_t sectorHighWaterMarkSecNum;
  uint8_t  highWaterHasWrapped;

  uint8_t  padding[LIO_SECTOR_SIZE - 22];
  uint8_t  fifoMode;
  uint8_t  volumeClean;
  uint8_t  VCBWriteSeqNum;

  Crc      crc;
} LioVolumeControlBlock;
CompilerAssert(sizeof(LioVolumeControlBlock) == LIO_SECTOR_SIZE);

//------------------------------------------------------------------------------
/* kmf operations */

LioResult lioWriteSystemBlocks(uint32_t numFiles, uint32_t numSectors);


LioResult lioOpenLogfile(uint32_t *numSectors);
LioResult lioEraseData(void);

/* helper routines */
void printTHinfo(LioTriggerBlock *ltb);
uint32_t lioComputeBitrate (LioTriggerBlockChanInfo *thchan);

/* initialization */
LioResult lioInitialize(void);
void lioShutdown(void);
LioResult lioOkToShutdown(void);
void lioSetPreTriggerSectors(uint32_t sectors);
uint32_t lioGetPreTriggerSectors(void);

/* Change virtual disk (track) */
void lioSetTrack(uint8_t track);

/* global io operations */
LioResult lioReadSector(uint32_t sectorNum);
LioResult lioWriteSector(uint32_t sectorNum);
LioResult lioFlushBuffer(void);
LioResult lioWriteVolumeDataBlock(void);
LioResult lioWriteVolumeControlBlock(void);

/* io operations on current datablock*/
LioResult lioWriteCmd(uint8_t *cmd);
LioResult lioWriteCommit(void);
LioResult lioAllocDataBlock(void);
LioResult lioFreeDataBlock(void);

/* Size and usage information */
int lioIsFlushInProgress(void);
LioResult lioGetDiskSize(uint32_t *diskSize);
LioResult lioGetUsedDiskSize(uint32_t *usedDiskSize);
uint8_t lioIsDiskFull(void);
uint8_t lioPostTriggerStop(void);
void lioPostTriggerReset(void);
/* Note: Can only be called after lioOpenLogfile! */
void lioGetVersion (uint8_t *major, uint8_t *minor);

/* File parsing functions */
LioResult lioGetNewestTriggerBlock(uint32_t *newestTriggerSecNum);
LioResult lioGetOlderTriggerBlock(uint32_t triggerBlockSecNum, uint32_t *olderTriggerSecNum);

LioResult lioGetVolumeControlBlockData(uint32_t *myLastKnownWPSecNum,
                                       uint32_t *myLastKnownTriggerSecNum,
                                       uint32_t *mySectorHighWaterMarkSecNum,
                                       uint32_t *myLastKnownSectorSeqNum,
                                       uint8_t  *myFifoMode,
                                       uint8_t  *myVolumeClean);
/* State changing operations */
LioResult lioStartLogEngine(uint32_t newPretriggerSize);
LioResult lioStopLogEngine(void);
LioResult lioBeginLog();
LioResult lioEndLog();
LioResult lioStopWithFullDisk(void);

LioResult lioWriteSystemBlocks(uint32_t numFiles, uint32_t numSectors);

// Useful for programs that uses lioReadSector/lioWriteSector etc
void getCurDataBlock(LioBlock** pData);


// Functions for test only!
LioResult lioSetFifoMode(uint8_t myFifoMode);
LioResult lioSetLastKnownSectorSeqNum(uint32_t myLastKnownSectorSeqNum);
// end of testfunctions

LioResult lioIsHighWaterWrapped(uint8_t *myHighWaterHasWrapped);

// Memory handling
LioResult lioInitSectBufs(void);
LioDataBlock * lioGetSect(void);
LioDataBlock * LioGetSectorUnsafe(void);
void lioDropSect(LioDataBlock *sect);
void lioListSectorUsage(void);

LioResult lioSetPath_W32(const char *path, const char *mode);
LioResult lioAllocateLogfile(uint32_t numSectors);


#ifdef __cplusplus
}
#endif
