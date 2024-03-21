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
 ** Description:
 **   Library for accessing the Minihydra CAN FD logger functions. Called via a
 **   wrapper DLL with the same API.
 ** ---------------------------------------------------------------------------
 */

#define NOMINMAX
#include <algorithm>
#include <cstdio>
#include <cstring>
#include <time.h>
#include <stdlib.h>

#include <fcntl.h>
#include <math.h>
#include <stdint.h>
#include "kvdebug.h"
#include "kvaMemolib.h"

#include "logio_w32.h"
#include "callback.h"
#include "os_util.h"
#include "hydra_host_cmds.h"

// Linux Drivers
#include "kcan_ioctl.h"
#include "vcan_ioctl.h"

#include "logger_config_file.h"
#include "util.h"

#include "TimeConv.h"
#include "triggerList.h"
#include "canlib_version.h"



#define LOCK_NAME          "KvaserMemolib2-df0538de-1ac4-4da4-b6de-24c528a35980"

// =====================================================================
// Global variables and macros for message sort
// =====================================================================
#define MSG_BUFFER_SIZE 1024 * 100
#define INC_MSG_BUFFER_PTR(x, y) { y = (x) + 1; if ((y) >= MSG_BUFFER_SIZE) y = 0; }
#define GLOBAL_CHAN_IDX MAX_NO_CHANNELS
#define GLOBAL_CHAN 0xff
hydraHostCmdLogFD msgbuf[MAX_NO_CHANNELS+1][MSG_BUFFER_SIZE] = {};
hydraHostCmdLogFD *msgbuffers[MAX_NO_CHANNELS+1] = {0};
unsigned int wp[MAX_NO_CHANNELS+1] = {0};
unsigned int rp[MAX_NO_CHANNELS+1] = {0};
int endOfLogFile = 0;


// =====================================================================
// Forward declarations
// =====================================================================
static KvaMemoStatus getLioVolumeDataBlock(MemoHandle h);
static KvaMemoStatus getLioVolumeControlBlock0(MemoHandle h);
static KvaMemoStatus getLioVolumeControlBlock1(MemoHandle h);
static KvaMemoStatus getLioDataBlock(MemoHandle h, unsigned int sector);
static KvaMemoStatus loadLioDataSector(MemoHandle h);
static KvaMemoStatus memoReadLioSector(MemoHandle h, uint32_t sector);
static KvaMemoStatus memoWriteLioSector(MemoHandle h, uint32_t sector);
static KvaMemoStatus WINAPI memoDiskReadLogicalSectorFast(MemoHandle h, unsigned int sectorNo);
static KvaMemoStatus getTracksFromLioDataBlock(MemoHandle h, TriggerList *list);
static KvaMemoStatus WINAPI memoDiskReadDBaseSector(MemoHandle h, uint32_t sectorNo, void *buf, size_t bufsize);
static KvaMemoStatus WINAPI memoDiskWriteDBaseSector(MemoHandle h, uint32_t sectorNo, void *buf, size_t bufsize);
static void resetgDBFileIO(void);
static KvaMemoStatus WINAPI memoOpenDBaseFile(MemoHandle h, uint32_t *startSector, uint32_t *endSector);
static KvaMemoStatus WINAPI memoCloseDBaseFile(MemoHandle h);
static KvaMemoStatus reopenDiskReadWrite(void);
static KvaMemoStatus reopenDiskReadOnly(void);
static KvaMemoStatus WINAPI memoWriteConfigDevice(const MemoHandle h, const uint8_t *const buf, const size_t buflen);
static KvaMemoStatus WINAPI memoWriteConfigDisk(const MemoHandle h, const uint8_t *const buf, const size_t buflen);
static KvaMemoStatus WINAPI memoReadConfigDevice(const MemoHandle h, uint8_t *const buf,
                                                 const size_t buflen, size_t *const actual_len);
static KvaMemoStatus WINAPI memoReadConfigDisk(const MemoHandle h, uint8_t *const buf,
                                               const size_t buflen, size_t *const actual_len);

KvaMemoStatus msgBufReset (void);

// =====================================================================
// Global variables and types
// =====================================================================
typedef enum {
  MemoHandleMemorator = 0,
} MemoHandleType;

// The size of the disk
uint32_t gDiskSize = 0;

// The struct used for fast io
KCANY_MEMO_DISK_IO_FAST gFastIo;

// Global variable from diskio_w32.c
extern int usbSpeed;

// This is the data sector used by all file operations
unsigned char lio_data_buffer[LIO_SECTOR_SIZE];

// Global struct for RTC messages from trigger blocks
hcmdLogRtcTimeFD rtcMessageFD;

// Global struct for trigger messages from housekeeping blocks
hcmdLogTrigFD trgMessageFD;

// Global struct for version messages
hcmdLogVerFD verMessageFD;

// A struct used to handle database
static struct {
  FILE    *fileHandle;
  char    *fileName;
  uint32_t startSector;
  uint32_t endSector;
  uint32_t currentSector;
} gDBFileIO;

// The global list of triggers
TriggerList triggerList = {NULL,NULL,0,0,NULL,NULL,0,0};

// =====================================================================
// memoConfBase
// =====================================================================
class memoConfBase
{
  protected:
    uint64_t  time;  // time saved in ns

  public:
    memoConfBase();
    ~memoConfBase();
    LockHandle      lockHandle;
    unsigned        verMajor;
    unsigned        verMinor;
    int             channels;
    int             transType[MAX_NO_CHANNELS];
    unsigned        bitrate[MAX_NO_CHANNELS];
    unsigned        hiresTimerFqMHz;
    bool            timeBackFilter;

    // Calendar start time of the log file. Standard unix time format.
    time_t          startTime;
    time_t          endTime;

    // Get time in ns.
    uint64_t currentTime64() {
      return time;
    }

    void updateTime(uint64_t x) {
      time = timeBackFilter ? std::max(time, x) : x;
    }

    void resetCurrentTime() {
      time = 0LL;
    }

    // Convert clock cycles to ns.
    uint64_t toNs(uint64_t ticks) {
      uint64_t ns = 0;
      if (hiresTimerFqMHz) {
        ns = (ticks*1000LL) / hiresTimerFqMHz;
      }
      return ns;
    }
};


// =====================================================================
// memoConf
// =====================================================================
class memoConf : public memoConfBase
{
  public:
    memoConf();
    ~memoConf();

    MemoHandleType      handleType;
    IoctlHandle         memoHandle;
    memoDevice          mDevice;
    LioBlock            *ld;
    int                 ldPos;              // Next event to return
    int                 currentFileIndx;
  std::string           kmfFileName;
    // The new logio need this
    uint32_t            currentSector;    // The sector read into LioBlock
    uint32_t            currentSequence;  // The sector sequence number
    Trigger             *trigger;         // A pointer to the trigger corresponding
                                          // to currentFileIndx
    bool                createTrigger;    // Create a fake trigger from Housekeeping block
    bool                sendVersion;      // Send a version event
};


// =====================================================================
// Local functions, only used internally.
// =====================================================================
static KvaMemoStatus connectToMemorator(MemoHandle h, int32 memoNr,
                                        int32 flags);

static void          updateTimeStamp(memoConfBase &mc, hydraHostCmdLogFD *e);

static bool          translateLogEvent(memoConfBase &h, hydraHostCmdLogFD *e,
                                       memoLogEventEx *me);

static void          closeLogFile(MemoHandle h);

memoConfBase::memoConfBase()
{
  int i;
  verMajor = 0;
  verMinor = 0;
  resetCurrentTime();
  lockHandle = INVALID_LOCK_HANDLE;
  for (i = 0; i < MAX_NO_CHANNELS; i++) {
    transType[i] = 0;
    bitrate[i] = 0;
  }
  channels = MAX_NO_CHANNELS;
  startTime = 0;
  endTime = 0;
  hiresTimerFqMHz = 0;
  timeBackFilter = true;

  // Initialize global variables
  memset(&gDBFileIO, 0, sizeof(gDBFileIO));
  for (int k = 0; k < MAX_NO_CHANNELS+1; k++) {
    msgbuffers[k] = msgbuf[k];
  }

  // Initialize global trigger message
  memset(&trgMessageFD, 0, sizeof(trgMessageFD));
  trgMessageFD.len = sizeof(trgMessageFD);
  trgMessageFD.logCmdNo = CMD_LOG_TRIG_FD;
  trgMessageFD.type = TRIGVAR_TYPE_DISK_FULL;

  // Initialize global RTC message
  memset(&rtcMessageFD, 0, sizeof(rtcMessageFD));
  rtcMessageFD.len = sizeof(rtcMessageFD);
  rtcMessageFD.logCmdNo = CMD_LOG_RTC_TIME_FD;

  // Initialize global version message
  memset(&verMessageFD, 0, sizeof(verMessageFD));
  verMessageFD.len = sizeof(verMessageFD);
  verMessageFD.logCmdNo = CMD_LOG_VERSION_FD;
}

memoConfBase::~memoConfBase()
{
  os_destroy_named_lock(lockHandle);
  lockHandle = INVALID_LOCK_HANDLE;
}


memoConf::memoConf()
: memoConfBase()
{
  memoHandle = INVALID_IOCTL_HANDLE;
  mDevice = memo_MEMO5;
  ld = 0;
  ldPos = 0;
  currentFileIndx = -1;
  handleType = MemoHandleMemorator;
  currentSector = 0;
  currentSequence = 0;
  trigger = NULL;
  createTrigger = false;
  sendVersion = true;
} // memoConf::memoConf()


memoConf::~memoConf()
{
  os_close_device(memoHandle);
  kmfFileName.clear();
}


//===========================================================================
static KvaMemoStatus LioResultToStatus(LioResult r)
{
  KvaMemoStatus stat;

  switch (r) {
    case lio_OK:            stat = MemoStatusOK; break;

    case lio_queueFull:     stat = MemoStatusERR_QUEUE_FULL; break;
    case lio_CRCError:      stat = MemoStatusERR_CRC_ERROR; break;
    case lio_SectorErased:  stat = MemoStatusERR_SECTOR_ERASED; break;
    case lio_FileError:     stat = MemoStatusERR_FILE_ERROR; break;
    case lio_DiskError:     stat = MemoStatusERR_DISK_ERROR; break;
    case lio_DiskFull_Dir:  stat = MemoStatusERR_DISKFULL_DIR; break;
    case lio_DiskFull_Data: stat = MemoStatusERR_DISKFULL_DATA; break;
    case lio_EOF:           stat = MemoStatusEOF; break;
    case lio_SeqError:      stat = MemoStatusERR_SEQ_ERROR; break;
    case lio_Error:         stat = MemoStatusFail; break;
    case lio_FileSystemCorrupt:   stat = MemoStatusERR_FILE_SYSTEM_CORRUPT; break;
    case lio_UnsupportedVersion:  stat = MemoStatusERR_UNSUPPORTED_VERSION; break;
    case lio_NotImplemented:      stat = MemoStatusERR_NOT_IMPLEMENTED; break;
    case lio_FatalError:    stat = MemoStatusERR_FATAL_ERROR; break;
    default:
      stat = MemoStatusFail; break;
  }
  return stat;
}


//===========================================================================
static KvaMemoStatus DioResultToStatus(DioResult r)
{
  KvaMemoStatus stat;

  switch (r) {
    case dio_OK:            stat = MemoStatusOK; break;

    case dio_QueueFull:     stat = MemoStatusERR_QUEUE_FULL; break;
    case dio_CRCError:      stat = MemoStatusERR_CRC_ERROR; break;
    case dio_SectorErased:  stat = MemoStatusERR_SECTOR_ERASED; break;
//  dio_Pending;
    case dio_IllegalRequest:  stat = MemoStatusERR_ILLEGAL_REQUEST; break;
    case dio_FileNotFound:    stat = MemoStatusERR_FILE_NOT_FOUND; break;
    case dio_FileError:       stat = MemoStatusERR_FILE_ERROR; break;
    case dio_NotImplemented:  stat = MemoStatusERR_NOT_IMPLEMENTED; break;
    case dio_NotFormatted:    stat = MemoStatusERR_NOT_FORMATTED; break;
    case dio_Timeout:         stat = MemoStatusERR_TIMEOUT; break;
    case dio_WrongDiskType:   stat = MemoStatusERR_WRONG_DISK_TYPE; break;
    case dio_DiskError:       stat = MemoStatusERR_DISK_ERROR; break;
    case dio_DiskCommError:   stat = MemoStatusERR_DEVICE_COMM_ERROR; break;
    case dio_NoDisk:          stat = MemoStatusERR_NO_DISK; break;
    case dio_NoMemory:        stat = MemoStatusERR_FATAL_ERROR; break;
    case dio_UserCancel:      stat = MemoStatusERR_USER_CANCEL; break;
    case dio_CpldError:       stat = MemoStatusERR_FIRMWARE; break;
    case dio_ConfigError:     stat = MemoStatusERR_CONFIG_ERROR; break;
    default:
      stat = MemoStatusFail; break;
  }
  return stat;
}


//===========================================================================
static KvaMemoStatus MemoResultToStatus(KCANY_MEMO_INFO *info)
{
  KvaMemoStatus ret = MemoStatusOK;

  if (info->status == MEMO_STATUS_FAILED) {
    //PRINTF(("errcodes = stat: %d, dio: %d, lio: %d\n", info->status, info->dio_status, info->lio_status));
    if (info->dio_status) {
      ret = DioResultToStatus((DioResult)info->dio_status);
    }
    else if (info->lio_status) {
      ret = LioResultToStatus((LioResult)info->lio_status);
    }
    else {
      ret = MemoStatusFail;
    }
  }
  return ret;
}



/*
 * =============================================================================
 *   memoInitializeLibrary
 * =============================================================================
 */

void WINAPI memoInitializeLibrary(void)
{
  PRINTF(("memoInitializeLibrary()\n"));
}


/*
 * =============================================================================
 *   memoOpen
 * =============================================================================
 */
MemoHandle WINAPI memoOpen(KvaMemoStatus *stat, int32 memoNr, int32 flags,
                           memoDevice mDevice)
{
  KvaMemoStatus s;
  memoConf *mc = new memoConf();
  mc->mDevice = mDevice;

  PRINTF(("memoOpen %d flags=0x%x dev=%d\n", memoNr, flags, mDevice));
  mc->lockHandle = os_create_named_lock(LOCK_NAME);
  if (mc->lockHandle == INVALID_LOCK_HANDLE) {
     s = MemoStatusERR_OCCUPIED;
  } else {
    switch (mDevice) {
      case memo_MEMO5:
        // We're talking to a real Memorator 5 with LIO ver 5
        s = connectToMemorator((MemoHandle)mc, memoNr, flags);
        break;

      case memo_DISK5:
        // We're reading from KMF LOGIO version 5 disk file.
        s = MemoStatusOK;
        break;

      default:

        // Something else; or a Memo I.
        s = MemoStatusERR_PARAM;
        break;
    }
  }

  if (s != MemoStatusOK) {
    delete mc;
    mc = 0;
  }

  PRINTF(("memoOpen fd -> stat=%d, mc=%p\n", s, mc));

  if (stat) *stat = s;
  return (MemoHandle)mc;
} // memoOpen




/*
 * =============================================================================
 *   connectToMemorator
 * =============================================================================
 */
static KvaMemoStatus connectToMemorator(MemoHandle h, int32 memoNr, int32 /* flags */)
{
  memoConf *mc = (memoConf *)h;
  IoctlHandle hnd = INVALID_IOCTL_HANDLE;


  // Try twice.
  hnd = os_open_device(memoNr);
  if (hnd == INVALID_IOCTL_HANDLE) {
    os_sleep(100);
    hnd = os_open_device(memoNr);
  }

  if (hnd == INVALID_IOCTL_HANDLE)
    return MemoStatusERR_DEVICE_COMM_ERROR;

  mc->memoHandle = hnd;
  return MemoStatusOK;
}


//===========================================================================
// memoProbeDeviceVersion
//===========================================================================
KvaMemoStatus WINAPI memoProbeDeviceVersion(memoDevice mDevice,
                                            int32 memoNr,
                                            uint32 *firmwareVersion,
                                            uint32 *fileVersion,
                                            uint32 *configVersion)
{
  memoConf *mc;
  KvaMemoStatus stat = MemoStatusFail;
  unsigned int major, minor, build, flags;
  uint8_t lioMajor, lioMinor, configMajor, configMinor;

  if (mDevice != memo_MEMO5) return MemoStatusFail;

  mc = new memoConf();
  mc->memoHandle = os_open_device(memoNr);
  if (mc->memoHandle) {
    stat = memoGetSoftwareVersionInfo((MemoHandle)mc, memo_SWINFO_FIRMWARE,
                                      &major, &minor, &build, &flags);
    if (stat == MemoStatusOK) {
      *firmwareVersion = (major << 24) + (minor << 16);
      // NOTE: fileVersion & configVersion can't be read here right now so
      // we fake them.
      fwVerToConfigVer(major, minor, &lioMajor, &lioMinor,
                                     &configMajor, &configMinor);

      *fileVersion = toVersion(lioMajor, lioMinor);
      *configVersion = toVersion(configMajor, configMinor);
    }
  }
  delete mc;

  return stat;
}


//===========================================================================
// memoProbeFileVersion
//===========================================================================
KvaMemoStatus WINAPI memoProbeFileVersion(const char *fn,
                                          uint32 *fileVersion,
                                          uint32 *configVersion)
{
  FILE *f;
  LioVolumeDataBlock vol;
  KvaMemoStatus stat = MemoStatusFail;
  std::string path, drive, dir, file, ext;
  uint8_t configMajor, configMinor;
  uint8_t lioMajor, lioMinor;

  *fileVersion = *configVersion = 0;

  // The GUI sends the name of the old KMF file here, so
  // let's try that one first and then try the new name.

  // Recast the old filename into something useful
  if ( !os_splitpath(std::string(fn), drive, dir, file, ext) ) {
    return MemoStatusERR_PARAM;
  }

  f = fopen(fn, "rb");
  if (f) {
    PRINTF(("%s is an old KMF-file.\n", fn));
  } else {
    os_makepath(path, drive, dir, LOGDATA_FILE_NAME, "");
    PRINTF(("memoProbeFileVersion: %s\n",path.c_str()));
    f = fopen(path.c_str(), "rb");
  }

  if (!f) {
    os_makepath(path, drive, dir, LOGDATA_FILE_NAME2, "");
    PRINTF(("memoProbeFileVersion: %s\n",path.c_str()));
    f = fopen(path.c_str(), "rb");
  }

  // Make sure that we opened
  if (!f) return stat;

  if (fread(&vol, sizeof(vol), 1, f) == 1) {
    splitVersion(vol.version, &lioMajor, &lioMinor);
    *fileVersion = toVersion(lioMajor, lioMinor);
    lioVerToConfigVer(lioMajor, lioMinor, &configMajor, &configMinor);
    *configVersion = toVersion(configMajor, configMinor);
    stat = MemoStatusOK;
  }
  fclose(f);
  PRINTF(("KMF version = %u.%u\n", lioMajor, lioMinor));
  PRINTF(("LIF version = %u.%u\n", configMajor, configMinor));
  return stat;
}


//===========================================================================
// memoGetDbaseFile
//
// Load a file from Memorator to PC
//
// Inputs
//  h      A MemoHandle
//  path   Location to where the extracted will be saved, e.g. C:\temp
//
// Outputs
//  dbname The name of the extracted file.
//===========================================================================
// path is where the file will end up and dbname is the name of the file.
//
KvaMemoStatus WINAPI memoGetDbaseFile(MemoHandle h, int filenumber, char *path, char *dbname)
{
  size_t           res = 0;
  KvaMemoStatus    status;
  uint8_t          buf[SD_DISK_SECTOR_SIZE] = {0};
  uint32_t         sector;
  uint32_t fileSize, bytes, bytesWritten;
  uint32_t startSector, endSector;
  char dbfile[13];
  FILE *hFile;
  std::string filename;

  if (!path || !dbname) return MemoStatusERR_PARAM;
  if (filenumber != 0) return MemoStatusERR_FILE_NOT_FOUND;

  // Is there a database area on the disk?
  status = memoOpenDBaseFile(h, &startSector, &endSector);
  if (status != MemoStatusOK) return status;

  // Get the first sector which contains length and filename
  memset(buf,0,sizeof(buf));
  status = memoDiskReadDBaseSector(h, startSector, buf, SD_DISK_SECTOR_SIZE);
  if (status != MemoStatusOK) {
    PRINTF(("memoDiskReadDBaseSector failed with %d", status));
    return status;
  }

  memcpy(&fileSize,&buf[0], sizeof(fileSize));
  // Check size; the area might be empty or erased
  if (fileSize == 0) {
    PRINTF(("memoGetDbaseFile: Erased or empty database file.\n"));
    status = memoCloseDBaseFile(h);
    return MemoStatusERR_FILE_NOT_FOUND;
  }
  memset(dbfile,0,sizeof(dbfile));
  memcpy(&dbfile,&buf[4],12);
  filename = std::string(path) +  os_filesep() + std::string(dbfile);
  PRINTF(("Saving as %s\n", filename.c_str()));
  dbname = strncpy(dbname,(char*) dbfile,13);

  // Open the result file
  hFile = fopen(filename.c_str(), "wb");
  if (hFile == NULL) {
    PRINTF(("Could not create the file '%s'\n", filename.c_str()));
    return MemoStatusERR_FILE_ERROR;
  }
  // Write the first data
  if (fileSize > SD_DISK_SECTOR_SIZE - 16) {
    bytes = SD_DISK_SECTOR_SIZE - 16;
  }
  else {
    bytes = fileSize;
  }
  res = fwrite(&buf[16], 1, bytes, hFile);
  if (res != bytes) {
    PRINTF(("memoGetDbaseFile: fwrite failed %Ilu",res));
    fclose(hFile);
    return MemoStatusERR_FILE_ERROR;
  }
  bytesWritten = bytes;

  // Write the rest of the data
  for (sector=startSector+1; sector < endSector; sector++) {
    status = memoDiskReadDBaseSector(h, sector, buf, SD_DISK_SECTOR_SIZE);
    if (status != MemoStatusOK) {
      PRINTF(("memoDiskReadDBaseSector failed with %d", status));
      fclose(hFile);
      return status;
    }
    if (fileSize - bytesWritten >=  SD_DISK_SECTOR_SIZE) {
      bytes = SD_DISK_SECTOR_SIZE;
    } else {
      bytes = fileSize - bytesWritten;
    }
    res = fwrite(buf, 1, bytes, hFile);
    if (res != bytes) {
      PRINTF(("memoGetDbaseFile: fwrite failed %Ilu",res));
      fclose(hFile);
      return MemoStatusERR_FILE_ERROR;
    }
    bytesWritten += bytes;
    if (bytesWritten == fileSize) {
      break;
    }
  }

  fclose(hFile);

  status = memoCloseDBaseFile(h);
  if (status != MemoStatusOK) return status;

  return MemoStatusOK;
}

//===========================================================================
// memoPutDbaseFile
//
// Save a file from PC to Memorator
//
// Inputs
//  h      A MemoHandle
//  path   The full path and name of the file, e.g. C:\temp\myfile.data
//         Note that the filename will be trucated to an 8.3 filename.
//
//===========================================================================
KvaMemoStatus WINAPI memoPutDbaseFile(MemoHandle h, int filenumber, char *filename)
{
  KvaMemoStatus    status;

  uint8_t buf[SD_DISK_SECTOR_SIZE] = {0};
  size_t bytesRead;
  uint32_t bytes, bytesWritten;
  uint32_t startSector, endSector;
  uint32_t sector = 0;
  FILE *hFile;
  uint64_t fileSize;
  char fname8[9];
  char ext4[5];

  if (filenumber != 0) return MemoStatusERR_FILE_NOT_FOUND;
  if (!filename) return MemoStatusERR_PARAM;

  // Open the infile and get the file size
  hFile = fopen(filename, "rb");
  if (hFile == NULL) {
    PRINTF(("Could not open the file '%s'\n", filename));
    return MemoStatusERR_FILE_ERROR;
  }
  if (! os_get_file_size(hFile, &fileSize) ) {
    PRINTF(("Could not get the file size from the file '%s'\n", filename));
    fclose(hFile);
    return MemoStatusERR_FILE_ERROR;
  }

  // Is there a database area on the disk?
  status = memoOpenDBaseFile(h, &startSector, &endSector);
  if (status != MemoStatusOK) {
    PRINTF(("Could not access the database area.\n"));
    fclose(hFile);
    return status;
  }

  // Check if the selected file will fit into the database area
  if ((endSector-startSector) * SD_DISK_SECTOR_SIZE < fileSize) {
    PRINTF(("Not enough space for the selected file.\n"));
    PRINTF(("%s is %I64lu bytes. Database area is %u bytes.\n",
            filename, fileSize, (endSector-startSector) * SD_DISK_SECTOR_SIZE));
    fclose(hFile);
    return MemoStatusERR_ILLEGAL_REQUEST;
  }

  // Copy size and file name (8.3) to the file
  memset(buf, 0, sizeof(buf));
  // Using only the lower bytes of filesize due to small file sizes
  memcpy(&buf[0], &fileSize, 4);

  // Parse full filename to extract the name and convert to 8.3
  std::string drive, dir, path, fname, ext;
  if ( !os_splitpath(std::string(filename), drive, dir, fname, ext)) {
    return MemoStatusERR_PARAM;
  }

  memset(fname8, 0, sizeof(fname8));
  memset(ext4, 0, sizeof(ext4));
  strncpy(fname8, fname.c_str(), 8);
  strncpy(ext4, ext.c_str(), 4);
  os_makepath(path, "", "", std::string(fname8), std::string(ext4));
  memcpy(&buf[4], path.c_str(), path.size());

  // Complete the first sector with data from the file
  bytes = SD_DISK_SECTOR_SIZE - 16;
  bytesRead = fread(&buf[16], 1, bytes, hFile);
  status = memoDiskWriteDBaseSector(h, startSector, buf, SD_DISK_SECTOR_SIZE);
  if (status != MemoStatusOK) {
      PRINTF(("memoDiskDBaseSector failed stat=%d\n", status));
      fclose(hFile);
      return status;
  }
  bytesWritten = (uint32_t) bytesRead;

  // Continue with the rest of the file
  sector = startSector + 1;
  bytes = SD_DISK_SECTOR_SIZE;
  while (sector <= endSector) {

    bytesRead = fread(buf, 1, bytes, hFile);
    if (bytesRead == 0) {
      // No more data to read
      break;
    }
    status = memoDiskWriteDBaseSector(h, sector, buf, SD_DISK_SECTOR_SIZE);
    if (status != MemoStatusOK) {
      PRINTF(("memoDiskDBaseSector failed stat=%d\n", status));
      fclose(hFile);
      return status;
    }
    bytesWritten += (uint32_t) bytesRead;
    sector++;
  }

  // Update the number of bytes written.
  memset(buf, 0, sizeof(buf));
  status = memoDiskReadDBaseSector(h, startSector, buf, SD_DISK_SECTOR_SIZE);
   if (status != MemoStatusOK) {
     PRINTF(("memoDiskReadDBaseSector failed stat=%d\n", status));
     fclose(hFile);
     return status;
   }
  memcpy(&buf[0], &bytesWritten, sizeof(bytesWritten));

  status = memoDiskWriteDBaseSector(h, startSector, buf, SD_DISK_SECTOR_SIZE);
  if (status != MemoStatusOK) {
    PRINTF(("memoDiskWriteDBaseSector failed stat=%d\n", status));
    fclose(hFile);
    return status;
  }

  fclose(hFile);

  status = memoCloseDBaseFile(h);
  if (status != MemoStatusOK) return status;

  return MemoStatusOK;
}


//===========================================================================
// memoEraseDbaseFile
//
// Erase a database file from the SD card.
//
// Inputs
//  h      A MemoHandle
//
//===========================================================================
KvaMemoStatus WINAPI memoEraseDbaseFile(MemoHandle h, int filenumber)
{
  KvaMemoStatus    status;

  uint8_t  buf[SD_DISK_SECTOR_SIZE];
  uint32_t startSector, endSector;

  if (filenumber != 0) return MemoStatusERR_FILE_NOT_FOUND;

  // Is there a database area on the disk?
  status = memoOpenDBaseFile(h, &startSector, &endSector);
  if (status != MemoStatusOK) {
    PRINTF(("Could not access the database area.\n"));
    return status;
  }

  memset(buf, 0, sizeof(buf));
  // Erase the first sector
  status = memoDiskWriteDBaseSector(h, startSector, buf, SD_DISK_SECTOR_SIZE);
  if (status != MemoStatusOK) {
      PRINTF(("memoDiskDBaseSector failed stat=%d\n", status));
      return status;
  }

  status = memoCloseDBaseFile(h);
  if (status != MemoStatusOK) return status;

  return MemoStatusOK;
}


//===========================================================================
// memoWriteConfig
//===========================================================================
KvaMemoStatus WINAPI memoWriteConfig(MemoHandle h, void *buf, size_t buflen)
{
  PRINTF(("memoWriteConfig\n"));
  if (!buf || !h ) return MemoStatusERR_PARAM;

  memoConf *mc = (memoConf *)h;

  switch (mc->mDevice) {
  case memo_MEMO5:
    return memoWriteConfigDevice(h, (uint8_t*)buf, buflen);
  case memo_DISK5:
    return memoWriteConfigDisk(h, (uint8_t*)buf, buflen);
  default:
    return MemoStatusERR_PARAM;
  }
}


//===========================================================================
// memoWriteConfigDevice
//===========================================================================
static KvaMemoStatus WINAPI memoWriteConfigDevice(const MemoHandle h, const uint8_t *const buf, const size_t buflen)
{
  int              r              = 0;
  hMemoDataFsInfoB *mfsb;
  KCANY_MEMO_INFO  info;
  KvaMemoStatus    stat;
  uint32_t         start_sector   = 0;
  uint32_t         end_sector     = 0;
  uint32_t         current_sector = 0;
  uint32_t         written        = 0;
  KvaMemoStatus    compare_stat;
  uint8_t          *p;
  size_t           read_len;
  uint8_t          sec[SD_DISK_SECTOR_SIZE];

  if (!buf) return MemoStatusERR_PARAM;

  memset(&info, 0, sizeof(info));
  info.subcommand = MEMO_SUBCMD_GET_FS_INFO_B;
  memoConf *mc = (memoConf *)h;

  r = os_ioctl(mc->memoHandle, KCANY_IOCTL_MEMO_GET_DATA, &info, sizeof(info));

  if (!r) return MemoStatusERR_DEVICE_COMM_ERROR;

  if ((stat = MemoResultToStatus(&info)) != MemoStatusOK) return stat;

  mfsb = (hMemoDataFsInfoB*) info.buffer;

  if ((mfsb->first_param_sector | mfsb->last_param_sector) == 0) {
    return MemoStatusERR_NOT_FORMATTED;
  }

  start_sector   = mfsb->first_param_sector;
  (void)start_sector; // Avoid cppcheck warning.
  current_sector = mfsb->first_param_sector;
  end_sector     = mfsb->last_param_sector;
  PRINTF(("write: start %d, current %d, end %d; buflen=%Ilu (0x%lx)\n",
          start_sector, current_sector, end_sector, buflen, buflen));


  while (written < buflen) {
    uint16_t len;

    if (current_sector > end_sector) {
      // File too large
      return MemoStatusERR_ILLEGAL_REQUEST;
    }

    len = (buflen-written) < SD_DISK_SECTOR_SIZE ? (uint16_t)(buflen-written) : SD_DISK_SECTOR_SIZE;

    if (buflen-written < SD_DISK_SECTOR_SIZE) {
      len = (uint16_t)(buflen-written);
    } else {
      len = SD_DISK_SECTOR_SIZE;
    }

    memset(sec, 0, SD_DISK_SECTOR_SIZE);
    memcpy(sec, &((uint8_t*)buf)[written], len);

    stat = memoDiskWritePhysicalSector(h, current_sector,
                                      sec,
                                      SD_DISK_SECTOR_SIZE);
    if (stat != MemoStatusOK) {
      PRINTF(("memoDiskWritePhysicalSector failed stat=%d\n", stat));
      return stat;
    }

    written += len;
    current_sector++;
  }
  // Readback check...
  p = (uint8_t*)malloc(buflen);
  compare_stat = memoReadConfig(h, p, buflen, &read_len);
  if (compare_stat != MemoStatusOK) {
    // Just keep the status code
    PRINTF(("memoWriteConfigDevice returned %d\n", compare_stat));
  }
  else if (buflen != read_len) {
    compare_stat = MemoStatusERR_FILE_ERROR;
    PRINTF(("memoWriteConfigDevice buflen got %Ilu expected: %Ilu written: %d\n",
            read_len, buflen, written));
  }
  else if (memcmp(buf, p, read_len)) {
    compare_stat = MemoStatusERR_FILE_ERROR;
    PRINTF(("memoWriteConfigDevice compare mismatch\n"));
  }

  free(p);

  return compare_stat;
} // memoWriteConfigDevice


//===========================================================================
// memoWriteConfigDisk
//===========================================================================
static KvaMemoStatus WINAPI memoWriteConfigDisk(const MemoHandle h, const uint8_t *const buf, const size_t buflen)
{
  PRINTF(("memoWriteConfigDisk\n"));

  if (!buf || !h || (buflen > PARAM_FILE_SIZE)) {
    return MemoStatusERR_PARAM;
  }

  // Find actual length of config data and check file content.
  {
    KvaMemoStatus    status;
    uint8_t          *endbuf = (uint8_t *)buf + buflen;
    size_t           L;
    BlockHead        *bh;
    uint8_t          *rp;

    rp = (uint8_t *)buf;
    bh = (BlockHead*) buf;
    L = 0;
    status = MemoStatusERR_CONFIG_ERROR;

    while (rp + sizeof(BlockHead) <= endbuf) {
      if (bh->id == 0) {
        PRINTF(("BLOCK_ID_ZERO\n"));
        if (L == 0) {
          PRINTF(("Config file is empty."));
          status = MemoStatusOK;
        } else {
          status = MemoStatusERR_CONFIG_ERROR;
        }
        break;
      }

      if (bh->len == 0) {
        PRINTF(("Error: Config block length is zero."));
        status = MemoStatusERR_CONFIG_ERROR;
        break;
      }

      if (bh->id == BLOCK_ID_END) {
        PRINTF(("BLOCK_ID_END\n"));
        L += sizeof(BlockHead);
        status = MemoStatusOK;
        break;
      }

      PRINTF(("BLOCK_ID: %02d (%u)\n", bh->id, bh->len));
      rp += bh->len;
      L += bh->len;
      bh = (BlockHead*) rp;
    } // while

    if (status != MemoStatusOK) {
      PRINTF(("[%s,%d] Invalid context\n", __FILE__, __LINE__));
      return status;
    }
    PRINTF(("Total length: %lu\n", L));
  }

  memoConf *mc = (memoConf *)h;

  if(mc->kmfFileName.empty()) {
    PRINTF(("[%s,%d] Internal error.\n", __FILE__, __LINE__));
    return MemoStatusERR_DEVICE_COMM_ERROR;
  }
  // Recast the path to the kvm log file to the path of the config file.
  std::string drive, dir, path, base, ext;
  if ( !os_splitpath(mc->kmfFileName, drive, dir, base, ext)) {
    return MemoStatusERR_PARAM;
  }
  os_makepath(path, drive, dir, CONFIG_FILE_NAME, "");
  PRINTF(("memoWriteConfigDisk(%s)\n", path.c_str()));

  // Open the configuration file on SD card.
  FILE* configFile = fopen(path.c_str(), "wb");
  if (!configFile) {
    return MemoStatusERR_DISK_ERROR;
  }

  // Write buffer to file.
  size_t bytesWritten = fwrite(buf, sizeof(buf[0]), buflen, configFile);
  if (bytesWritten != buflen) {
    fclose(configFile);
    return MemoStatusERR_DISKFULL_DATA;
  }

  // Must write an extra sector, otherwise memoReadConfigDevice() does not
  // work.
  size_t stuffLen = SD_DISK_SECTOR_SIZE * 2 - buflen % SD_DISK_SECTOR_SIZE;
  PRINTF(("memoWriteConfigDisk: stuffLen(%lu).\n", stuffLen));
  if ((stuffLen > 0) && ((buflen + stuffLen) <= PARAM_FILE_SIZE)) {
    uint8_t *stuffBuf = (uint8_t*) malloc(stuffLen);
    if (!stuffBuf) {
      fclose(configFile);
      return MemoStatusFail;
    }
    memset(stuffBuf, 0, stuffLen);
    (void)fwrite(stuffBuf, sizeof(stuffBuf[0]), stuffLen, configFile);
    free(stuffBuf);
  }

  fclose(configFile);

  return MemoStatusOK;
} // memoWriteConfigDisk


//===========================================================================
// memoReadConfig
//===========================================================================
KvaMemoStatus WINAPI memoReadConfig(MemoHandle h, void *buf,
                                    size_t buflen, size_t *actual_len)

{
  if (!buf || !h ) return MemoStatusERR_PARAM;

  memoConf *mc = (memoConf *)h;

  PRINTF(("memoReadConfig(%i)\n", mc->mDevice));

  switch (mc->mDevice) {
  case memo_MEMO5:
    return memoReadConfigDevice(h, (uint8_t*)buf, buflen, actual_len);
  case memo_DISK5:
    return memoReadConfigDisk(h, (uint8_t*)buf, buflen, actual_len);
  default:
    return MemoStatusERR_PARAM;
  }
} // memoReadConfig


/*
 * =============================================================================
 *   memoReadConfigDevice
 * =============================================================================
 */
static KvaMemoStatus WINAPI memoReadConfigDevice(const MemoHandle h, uint8_t *const buf,
                                                 const size_t buflen, size_t *const actual_len)
{
  int              r = 0;
  hMemoDataFsInfoB *mfsb;
  KCANY_MEMO_INFO  info;
  KvaMemoStatus    stat;
  uint8_t         *tmpbuf;
  uint8_t         *p;
  uint32_t         i;
  size_t           L;
  bool             done = false;
  BlockHead        *bh;
  uint8_t          *rp;

  if (!buf) return MemoStatusERR_PARAM;
  PRINTF(("memoReadConfigDevice\n"));
  memset(&info, 0, sizeof(info));
  info.subcommand = MEMO_SUBCMD_GET_FS_INFO_B;
  memoConf *mc = (memoConf *)h;

  r = os_ioctl(mc->memoHandle, KCANY_IOCTL_MEMO_GET_DATA, &info, sizeof(info));

  if (!r) return MemoStatusERR_DEVICE_COMM_ERROR;

  if ((stat = MemoResultToStatus(&info)) != MemoStatusOK) return stat;

  mfsb = (hMemoDataFsInfoB*) info.buffer;

  if ((mfsb->first_param_sector | mfsb->last_param_sector) == 0) {
    return MemoStatusERR_NOT_FORMATTED;
  }

  // If the number of sectors is larger than this something is wrong:
  if (mfsb->last_param_sector - mfsb->first_param_sector >
      PARAM_FILE_SIZE / SD_DISK_SECTOR_SIZE) {
    PRINTF(("If the number of sectors is larger than this something is wrong\n"));
    return MemoStatusERR_ILLEGAL_REQUEST;
  }

  // Allocate space for the entire allowed PARAM.LIF
  tmpbuf = (uint8_t*) malloc(PARAM_FILE_SIZE);
  if (tmpbuf == NULL) return MemoStatusFail;


  p = tmpbuf;
  rp = tmpbuf;
  bh = (BlockHead*) tmpbuf;
  L = 0;
  for (i=mfsb->first_param_sector; i<mfsb->last_param_sector; i++) {
    stat = memoDiskReadPhysicalSector(h, i, p, SD_DISK_SECTOR_SIZE);
    if (stat != MemoStatusOK) {
      PRINTF(("[%s,%d] memoReadConfigDevice, status(%i)\n", __FILE__, __LINE__, stat));
      break;
    }
    p += SD_DISK_SECTOR_SIZE;

    while (rp + sizeof(BlockHead) <= p) {

      if (bh->id == 0) {
        PRINTF(("BLOCK_ID_ZERO\n"));
        if (L == 0) {
          PRINTF(("Config file is empty.\n"));
          stat = MemoStatusOK;
        } else {
          stat = MemoStatusERR_CONFIG_ERROR;
          PRINTF(("[%s,%d] memoReadConfigDevice\n", __FILE__, __LINE__));
        }
        done = true;
        break;
      }

      if (bh->len == 0) {
        PRINTF(("Error: Config block length is zero.\n"));
        stat = MemoStatusERR_CONFIG_ERROR;
        done = true;
        break;
      }

      if (bh->id == BLOCK_ID_END) {
        PRINTF(("BLOCK_ID_END\n"));
        L += sizeof(BlockHead);
        stat = MemoStatusOK;
        done = true;
        break;
      }

      PRINTF(("BLOCK_ID: %02d (%u)\n", bh->id, bh->len));
      rp += bh->len;
      L += bh->len;
      bh = (BlockHead*) rp;
    } // while

    if (done) break;

    // Handle EOF without BLOCK_ID_END or illegal block sizes
    stat = MemoStatusERR_CONFIG_ERROR;
  } // for

  PRINTF(("Read %lu bytes configuration.\n", L));
  memcpy(buf, tmpbuf, MIN(L, buflen));
  if (actual_len) *actual_len = L;
  free(tmpbuf);

  if (MemoStatusOK == stat && L > buflen) {
    PRINTF(("Error: Output buflen is too small; needed %lu bytes but only got %lu.\n",
    L, buflen));
    stat = MemoStatusERR_ILLEGAL_REQUEST;
  }

  return stat;

} // memoReadConfigDevice


/*
 * =============================================================================
 *   memoClose
 * =============================================================================
 */
KvaMemoStatus WINAPI memoClose(MemoHandle h)
{
  if (!h) return MemoStatusERR_PARAM;

  memoConf * mc = (memoConf *)h;

  // Leave the config mode (relevant only for a Memorator)
  if (mc->mDevice == memo_MEMO5) {
  (void) memoConfigModeSetInterval(h, 0);
  }

  // Delete the trigger list
  if (triggerList.count) DeleteTriggerList(&triggerList);
  os_close_device(mc->memoHandle);
  mc->memoHandle = INVALID_IOCTL_HANDLE;
  delete mc;
  return MemoStatusOK;
} // memoClose


/*
 * =============================================================================
 *   memoReadFilesystemInfo
 *   Returns data about the (real) FAT/FAT32 file system.
 * =============================================================================
 */
KvaMemoStatus WINAPI memoReadFilesystemInfo(MemoHandle h, fileSysInfo *fSysInfo)
{

  int r = 0;
  hMemoDataFsInfo *mfs;
  KvaMemoStatus    stat;
  KCANY_MEMO_INFO info;
  memoConf *mc = (memoConf *)h;

  if (!fSysInfo) return MemoStatusERR_PARAM;

  memset(&info, 0, sizeof(info));
  info.subcommand = MEMO_SUBCMD_GET_FS_INFO;
  r = os_ioctl (mc->memoHandle, KCANY_IOCTL_MEMO_GET_DATA, &info, sizeof(info));

  if (!r) {
    return MemoStatusERR_DEVICE_COMM_ERROR;
  }


  if ((stat = MemoResultToStatus(&info)) != MemoStatusOK) return stat;

  mfs = (hMemoDataFsInfo*) info.buffer;
  if (mfs->fat_type == 0) {
    PRINTF(("fat_type==0\n"));
    return MemoStatusERR_NOT_FORMATTED;
  }

  fSysInfo->cluster_size      = mfs->cluster_size;
  fSysInfo->fat_size          = mfs->fat_size;
  fSysInfo->fat_type          = mfs->fat_type;
  fSysInfo->dir_entries       = mfs->dir_entries;
  fSysInfo->fat1_start        = mfs->fat1_start;
  fSysInfo->first_data_sector = mfs->first_data_sector;
  fSysInfo->last_data_sector  = mfs->last_data_sector;

  return MemoStatusOK;
} // memoReadFilesystemInfo


/*
 * =============================================================================
 *   memoGetFileSystemUsage
 * =============================================================================
 */
KvaMemoStatus WINAPI memoGetFileSystemUsage(MemoHandle h, fileSysUsage *fus)
{
  KvaMemoStatus status;
  PRINTF(("memoGetFileSystemUsage"));
  if (!h) return MemoStatusERR_PARAM;
  if (!fus) return MemoStatusERR_PARAM;

  status = memoGetFileTrackUsage(h,0,fus);
  PRINTF(("Files: %d(%d) Size:%u(%u)", fus->fileCount, fus->maxFileCount,
          fus->usedDiskSize, fus->diskSize));
  PRINTF(("status: %d",status));
  return status;
} // memoGetFileSystemUsage


/*
 * =============================================================================
 * memoGetHardwareInfo
 * =============================================================================
 */
KvaMemoStatus WINAPI memoGetHardwareInfo(MemoHandle h, MemoHardwareInfo *hinfo)
{
  int r = 0;
  hMemoDataMiscInfo *miff;
  KCANY_MEMO_INFO   info;
  KvaMemoStatus    stat;

  PRINTF(("memoGetHardwareInfo\n"));
  if (!hinfo) return MemoStatusERR_PARAM;
  if (!h) return MemoStatusERR_PARAM;

  memoConf *mc = (memoConf *)h;
  if (mc->mDevice != memo_MEMO5) return MemoStatusERR_PARAM;

  memset(&info, 0, sizeof(info));
  info.subcommand = MEMO_SUBCMD_GET_MISC_INFO;
  info.buflen = sizeof(hMemoDataMiscInfo);

  r = os_ioctl(mc->memoHandle, KCANY_IOCTL_MEMO_GET_DATA, &info, sizeof(info));

  if (!r) return MemoStatusERR_DEVICE_COMM_ERROR;

  if ((stat = MemoResultToStatus(&info)) != MemoStatusOK) return stat;

  miff = (hMemoDataMiscInfo*) info.buffer;

  memset(hinfo, 0, sizeof(*hinfo));
  hinfo->diskFlags = miff->diskPresent? MEMO_DISKFLG_IS_PRESENT : 0;
  hinfo->diskFlags |= miff->diskWriteProtected? MEMO_DISKFLG_IS_WRITE_PROTECTED : 0;

  hinfo->powerFlags =  (miff->powerStatus & MEMO_POWER_BAT_FAULT) ? MEMO_POWERFLG_BATTERY_FAULT : 0;
  hinfo->powerFlags |= (miff->powerStatus & MEMO_POWER_BAT_FAULT_NTC) ? MEMO_POWERFLG_BATTERY_FAULT_NTC : 0;
  hinfo->powerFlags |= (miff->powerStatus & MEMO_POWER_BAT_CHARGING) ? MEMO_POWERFLG_BATTERY_CHARGING : 0;
  hinfo->powerFlags |= (miff->powerStatus & MEMO_POWER_BAT_POWER_OK) ? MEMO_POWERFLG_BATTERY_OK : 0;
  hinfo->powerFlags |= (miff->powerStatus & MEMO_POWER_EXTPOWER_OK) ? MEMO_POWERFLG_EXTERNAL_POWER : 0;
  hinfo->powerFlags |= (miff->powerStatus & MEMO_POWER_USBPOWER_OK) ? MEMO_POWERFLG_USB_POWER : 0;

  hinfo->temperature = -273;
  if (miff->temperatureEncoding == MEMO_TEMPERATURE_MHYDRA) {
    // Termistor TN204C104, 100k ohm vid 25C med beta(25/85)=4141K
    const double R_129   = 1.1;
    const double Beta    = 4141.0;
    const double NTC_T_0 = 273.15;
    double R_act, T = -NTC_T_0;
    double V_adc = ((double) miff->temperature) / 256.0;
    R_act = (1.0 / (1.0 - V_adc) - 1.0) * R_129;
    if (R_act > 0.0) T = 1.0 / (1.0/298 - log(1.0/R_act)/Beta) - NTC_T_0;
    hinfo->temperature = (int32) floor(T + 0.5);
  } else if (miff->temperatureEncoding == MEMO_TEMPERATURE_MEMO5) {
    hinfo->temperature = miff->temperature - 273;
  }

  hinfo->vBattery = miff->battery;

  return MemoStatusOK;
}


/*
 * =============================================================================
 *   memoReadDiskInfo
 * =============================================================================
 */
KvaMemoStatus WINAPI memoReadDiskInfo(MemoHandle h, diskInfo *dInf)
{
  int r = 0;
  hMemoDataDiskInfoA *infA;
  hMemoDataDiskInfoB *infB;
  KCANY_MEMO_INFO infoA, infoB;
  KvaMemoStatus    stat;

  memoConf *mc = (memoConf *)h;

  if (!dInf) return MemoStatusERR_PARAM;
  if (!h) return MemoStatusERR_PARAM;
  if (mc->mDevice != memo_MEMO5) return MemoStatusERR_PARAM;

  memset(&infoA, 0, sizeof(infoA));
  infoA.subcommand = MEMO_SUBCMD_GET_DISK_INFO_A;
  r = os_ioctl (mc->memoHandle, KCANY_IOCTL_MEMO_GET_DATA, &infoA, sizeof(infoA));

  if (!r) {
    PRINTF(("MEMO_SUBCMD_GET_DISK_INFO_A failed\n"));
    return MemoStatusERR_DEVICE_COMM_ERROR;
  }

  if ((stat = MemoResultToStatus(&infoA)) != MemoStatusOK) return stat;

  infA = (hMemoDataDiskInfoA*) infoA.buffer;

  memset(&infoB, 0, sizeof(infoB));
  infoB.subcommand = MEMO_SUBCMD_GET_DISK_INFO_B;
  r = os_ioctl(mc->memoHandle, KCANY_IOCTL_MEMO_GET_DATA, &infoB, sizeof(infoB));

  if (!r) {
    PRINTF(("MEMO_SUBCMD_GET_DISK_INFO_B failed\n"));
    return MemoStatusERR_DEVICE_COMM_ERROR;
  }

  if ((stat = MemoResultToStatus(&infoB)) != MemoStatusOK) return stat;

  infB = (hMemoDataDiskInfoB*) infoB.buffer;

  memset(dInf, 0, sizeof(diskInfo));
  dInf->disk_type        = infB->disk_type;
  dInf->version          = infB->version;
  dInf->data_size        = infB->data_size;
  dInf->read_time        = infB->read_time;
  dInf->wr_factor        = infB->wr_factor;
  dInf->read_blk_size    = infB->read_blk_size;
  dInf->wr_blk_size      = infB->wr_blk_size;
  dInf->trans_speed      = infB->trans_speed;
  dInf->file_format      = infB->file_format;
  dInf->erase_value      = infB->erase_value;
  dInf->m_id             = infA->mfgId;
  memcpy(&dInf->oem_id, &infA->oemId, sizeof(dInf->oem_id));
  strncpy(dInf->product_name, infA->productName, sizeof(dInf->product_name));
  dInf->product_revision = infA->productRev;
  dInf->serial_number    = infB->serialNumber;
  dInf->date_code        = infA->dateCode;

  PRINTF(("\n\nPrint diskInfo hello memorator:\n\n"));
  PRINTF(("disk_type:             %8d\n", dInf->disk_type));
  PRINTF(("version:               %8d\n", dInf->version));
  PRINTF(("data_size:             %8d\n", dInf->data_size));
  PRINTF(("read_time:             %8d\n", dInf->read_time));
  PRINTF(("wr_factor:             %8d\n", dInf->wr_factor));
  PRINTF(("read_blk_size:         %8d\n", dInf->read_blk_size));
  PRINTF(("wr_blk_size:           %8d\n", dInf->wr_blk_size));
  PRINTF(("trans_speed:           %8d\n", dInf->trans_speed));
  PRINTF(("file_format:           %8d\n", dInf->file_format));
  PRINTF(("erase_value:           %8d\n", dInf->erase_value));
  PRINTF(("m_id:                  %8x\n", dInf->m_id));
  PRINTF(("oem_id:                %8x\n", dInf->oem_id));
  PRINTF(("product_name:          %s \n", dInf->product_name));
  PRINTF(("product_revision:      %8x\n", dInf->product_revision));
  PRINTF(("serial_number:         %8d\n", dInf->serial_number));
  PRINTF(("date_code:             %8x\n", dInf->date_code));

  return MemoStatusOK;

} // memoReadDiskInfo


/*
 * =============================================================================
 *   memoFormatDisk
 * =============================================================================
 */
KvaMemoStatus WINAPI memoFormatDisk(MemoHandle h, uint32 *format_stat)
{
  memoConf *mc = (memoConf *)h;

  if (!mc) return MemoStatusERR_PARAM;
  if (!format_stat) return MemoStatusERR_PARAM;
  char *tmp = (char*) malloc(mc->kmfFileName.size()+1);
  if (!tmp) return MemoStatusERR_PARAM;
  strncpy(tmp, mc->kmfFileName.c_str(), mc->kmfFileName.size()+1);
  return memoFormatDiskEx(h,
                          tmp,
                          MEMOLIB_FORMAT_FAT16|MEMOLIB_FORMAT_RECREATE,
                          0,0,0,
                          format_stat);
  free(tmp);
}


//===========================================================================
/*
 * "path" is empty (for a Memorator), or the path to a disk reader
 * (e.g. "E:", without trailing slash), or a complete path to a KMF
 * file.
 *
 * "flags" is any combination of the MEMOLIB_FORMAT_xxx flags.
 *
 * "reserve_space" is the amount of space to reserve in MB on the disk when
 * creating the KMF file.
 *
 * "max_space" is the size of the KMF file to be created. If zero,
 * create as large as possible.
 *
 * "dbaseSpace" is the amount of space to reserve in MB on the disk
 * for databases.
 *
 * "format_stat" is a pointer to an uint32 that will contain the result
 * of the formatting operation.
 *
 *
 * MEMOLIB_FORMAT_FAT16 and MEMOLIB_FORMAT_FAT32 are meaningful only
 * for a Memorator.
 *
 * If neither MEMOLIB_FORMAT_FAT16 nor MEMOLIB_FORMAT_FAT32 is given,
 * the disk will not be formatted. Instead the existing format will be
 * kept.
 *
 * Specifying MEMOLIB_FORMAT_RECREATE means that the KMF file will be
 * removed, if it exists, and then re-created.
 *
 * MEMOLIB_FORMAT_CLEAR_DISK (for a disk or a memorator) means that no
 * formatting will take place, but all files will be removed.
 *
 * MEMOLIB_FORMAT_SIZE_IS_MB means that reserve_space and dbaseSpace are
 * given in megabytes. If this flag isn't specified, these fields are
 * given in percent.
 *
 */
KvaMemoStatus WINAPI memoFormatDiskEx(MemoHandle h,
                                      char *path,
                                      uint32 flags,
                                      uint32 reserve_space,
                                      uint32 max_space,
                                      uint32 dbaseSpace,
                                      uint32 *format_stat)
{
  memoConf *mc = (memoConf *)h;
  KvaMemoStatus status = MemoStatusOK;

  // Check various illegal parameter combinations.
  if (!format_stat) return MemoStatusERR_PARAM;
  if (!mc) return MemoStatusERR_PARAM;

  // Invalidate disk size and the cache by reseting the global fast io buffer
  memset(&gFastIo,0, sizeof(gFastIo));
  gDiskSize = 0;

  if ((flags & (MEMOLIB_FORMAT_PATH_DISKREADER|MEMOLIB_FORMAT_PATH_DISKFILE)) == 0 ) {
    //
    // A connected Memorator II
    //
    int ret = 0;
    KCANY_MEMO_INFO info;
    memoConf * mc = (memoConf *)h;
    hMemoInitDiskResp *data;
    uint8_t major, minor;

    if (mc->mDevice != memo_MEMO5) return MemoStatusERR_PARAM;

    memset(&info, 0, sizeof(info));

    if (flags & (MEMOLIB_FORMAT_FAT16|MEMOLIB_FORMAT_FAT32)) {
      //
      // Low-level format the disk.
      //
      PRINTF(("Low-level format the disk"));
      // The disk is formatted, so there will be no file to erase
      hMemoInitDiskReq *subcmd = (hMemoInitDiskReq *) info.buffer;
      info.subcommand = MEMO_SUBCMD_FORMAT_DISK;
      info.buflen = sizeof(hMemoInitDiskReq);
      info.timeout = 180*1000; // Three minutes.

      subcmd->maxDataSize = max_space;
      subcmd->dbaseSpace = dbaseSpace;
      subcmd->reserveSpace = reserve_space;
      subcmd->fileSystem = (flags & MEMOLIB_FORMAT_FAT16)? 16 : 32;

      ret = os_ioctl (mc->memoHandle, KCANY_IOCTL_MEMO_PUT_DATA, &info, sizeof(info));

      data = (hMemoInitDiskResp*)(info.buffer);
      PRINTF(("memoFormatDiskEx FORMAT_DISK ret=0x%x dioStatus=%d\n",
              ret,
              data->dioStatus));

      if (!ret) return MemoStatusERR_DEVICE_COMM_ERROR;

      *format_stat = data->dioStatus << 8;

      if (data->dioStatus != 0) return DioResultToStatus((DioResult)(data->dioStatus));

    }

    if (flags & MEMOLIB_FORMAT_RECREATE) {
      //
      // Create LOGDATA.KMF.
      //
      //PRINTF(("Create LOGDAT00.KMF"));
      hMemoInitDiskReq *subcmd = (hMemoInitDiskReq *) info.buffer;
      memset(&info, 0, sizeof(info));
      info.subcommand = MEMO_SUBCMD_CLEAR_DATA;
      info.buflen = sizeof(hMemoInitDiskReq);
      info.timeout = 180*1000; // Three minutes.

      subcmd->maxDataSize = max_space;
      subcmd->dbaseSpace = dbaseSpace;
      subcmd->reserveSpace = reserve_space;
      subcmd->fileSystem = 16;

      ret = os_ioctl(mc->memoHandle,KCANY_IOCTL_MEMO_PUT_DATA, &info, sizeof(info));

      data = (hMemoInitDiskResp*)(info.buffer);
      PRINTF(("memoFormatDisk CLEAR_DATA ret=0x%x lioStatus=%d\n",
              ret,
              data->lioStatus));

      if (!ret) return MemoStatusERR_DEVICE_COMM_ERROR;
      *format_stat += data->lioStatus;
      if (data->lioStatus != 0) return LioResultToStatus((LioResult)(data->lioStatus));

      // Partition the disk if needed
      memset(lio_data_buffer,0,sizeof(lio_data_buffer));
      status = getLioVolumeDataBlock(h);
      LioVolumeDataBlock* lvdb = (LioVolumeDataBlock*) lio_data_buffer;
      if (MemoStatusOK != status) {
        return status;
      }
      splitVersion(lvdb->version, &major, &minor);
      PRINTF(("lvdb->version: %u.%u", major, minor));
      if (LIO_MULTITRACK_VERSION == major) {
       PRINTF(("memoPartitionDisk for LIO_MULTITRACK_VERSION."));
       status = memoPartitionDisk(h, path, 2);
      }
      else {
        status = memoPartitionDisk(h, path, 1);
      }
    }
    if (MemoStatusOK != status) return status;
  }
  else {
    //
    // Must be a logio3 KMF file, or a disk reader.
    //

    PRINTF(("Must be a KMF file, or a disk reader"));
    if (mc->mDevice != memo_DISK5) return MemoStatusERR_PARAM;

    status = reopenDiskReadWrite();
    if (status != MemoStatusOK) {
      (void) reopenDiskReadOnly();
      PRINTF(("memoFormatDiskEx: File is write-protected.\n"));
      return MemoStatusERR_WRITE_PROT;
    }

    status = memoPartitionDisk(h, path, 1);
    if (status != MemoStatusOK) return status;

    // Flush the files
    status = reopenDiskReadOnly();
    if (status != MemoStatusOK) return status;
  }

  return memoValidateDisk(h, false, false);
}



//===========================================================================
KvaMemoStatus WINAPI memoClearDataDisk(MemoHandle h, uint32 *stat)
{
  memoConf * mc = (memoConf *)h;

  switch (mc->mDevice) {
    case memo_MEMO5:
      return memoFormatDiskEx(h,
                              NULL,
                              MEMOLIB_FORMAT_RECREATE,
                              0,0,0,
                              stat);

    case memo_DISK5:
      if (mc->kmfFileName.empty() ) return MemoStatusERR_PARAM;
      return memoFormatDiskEx(h,
                              NULL,
                              MEMOLIB_FORMAT_RECREATE|MEMOLIB_FORMAT_PATH_DISKREADER,
                              0,0,0,
                              stat);

    default:
      return MemoStatusERR_PARAM;
  }
}



/*
 * =============================================================================
 * memoGetRTC
 * Time is returned in standard unix format (seconds since 1/1 1970).
 * It is recommended to set the time using UTC to avoid problems with
 * DST.
 * =============================================================================
 */
KvaMemoStatus WINAPI memoGetRTC(MemoHandle h, uint32 *t)
{
  int r = 0;
  KCANY_MEMO_INFO infoA;
  memoConf *mc = (memoConf *)h;
  KvaMemoStatus    stat;

  memset(&infoA, 0, sizeof(infoA));
  infoA.subcommand = MEMO_SUBCMD_GET_RTC_INFO;
  infoA.buflen = sizeof(t);
  r = os_ioctl (mc->memoHandle, KCANY_IOCTL_MEMO_GET_DATA, &infoA, sizeof(infoA));

  if (!r) return MemoStatusERR_DEVICE_COMM_ERROR;

  if ((stat = MemoResultToStatus(&infoA)) != MemoStatusOK) return stat;

  memcpy(t, infoA.buffer, sizeof(*t));

  return MemoStatusOK;
} // memoGetRTC

/*
 * =============================================================================
 *   memoSetRTC
 * =============================================================================
 */
KvaMemoStatus WINAPI memoSetRTC(MemoHandle h, uint32 t)
{
  int r = 0;
  KCANY_MEMO_INFO infoA;
  memoConf *mc = (memoConf *)h;
  KvaMemoStatus    stat;

  memset(&infoA, 0, sizeof(infoA));
  memcpy(infoA.buffer, &t, sizeof(t));
  infoA.subcommand = MEMO_SUBCMD_PUT_RTC_INFO;
  infoA.buflen = sizeof(t);
  r = os_ioctl (mc->memoHandle, KCANY_IOCTL_MEMO_PUT_DATA, &infoA, sizeof(infoA));

  if (!r) return MemoStatusERR_DEVICE_COMM_ERROR;

  if ((stat = MemoResultToStatus(&infoA)) != MemoStatusOK) return stat;

  return MemoStatusOK;

} // memoSetRTC

/*
 * =============================================================================
 *   memoFlashAllLeds
 * =============================================================================
 */

KvaMemoStatus WINAPI memoFlashAllLeds(MemoHandle h)
{
  int ret = 0;
  memoConf * mc = (memoConf *)h;
  int i;

  KCAN_IOCTL_LED_ACTION_I io_on;
  KCAN_IOCTL_LED_ACTION_I io_off;

  io_on.sub_command = LED_SUBCOMMAND_ALL_LEDS_ON;
  io_off.timeout = 100;
  io_off.sub_command = LED_SUBCOMMAND_ALL_LEDS_OFF;
  io_off.timeout = 100;

  for (i=0; i<15; i++) {
    ret = os_ioctl (mc->memoHandle, KCAN_IOCTL_LED_ACTION, &io_on, sizeof(KCAN_IOCTL_LED_ACTION_I));

    if (!ret) break;
    os_sleep(100);
    ret = os_ioctl (mc->memoHandle, KCAN_IOCTL_LED_ACTION, &io_off, sizeof(KCAN_IOCTL_LED_ACTION_I));

    if (!ret) break;
    os_sleep(100);
  }

  if (!ret) return MemoStatusERR_DEVICE_COMM_ERROR;
  else return MemoStatusOK;
}

//===========================================================================
KvaMemoStatus WINAPI memoConfigModeGetInterval(MemoHandle /* h */, int * /* interval */)
{
  PRINTF(("memoConfigModeGetInterval NOT IMPLEMENTED\n"));
  // Could be returned with hMemoDataMiscInfo if ever needed.
  return MemoStatusFail;
}


//===========================================================================
KvaMemoStatus WINAPI memoConfigModeGetDiskStatus(MemoHandle h, int *diskStat)
{
  int r = 0;
  hMemoDataMiscInfo *miff;
  KCANY_MEMO_INFO info;
  KvaMemoStatus    stat;

  if (!diskStat) return MemoStatusERR_PARAM;
  if (!h) return MemoStatusERR_PARAM;

  memset(&info, 0, sizeof(info));
  info.subcommand = MEMO_SUBCMD_GET_MISC_INFO;
  info.buflen = sizeof(hMemoDataMiscInfo);
  memoConf *mc = (memoConf *)h;

  r = os_ioctl (mc->memoHandle, KCANY_IOCTL_MEMO_GET_DATA,&info,sizeof(info));


  if (!r) return MemoStatusERR_DEVICE_COMM_ERROR;

  if ((stat = MemoResultToStatus(&info)) != MemoStatusOK) return stat;

  miff = (hMemoDataMiscInfo*) info.buffer;

  *diskStat = miff->diskPresent;

  return MemoStatusOK;
}


//===========================================================================
KvaMemoStatus WINAPI memoConfigModeGetMode(MemoHandle h, int *configMode)
{
  int r = 0;
  hMemoDataMiscInfo *miff;
  KCANY_MEMO_INFO info;
  KvaMemoStatus    stat;

  PRINTF(("memoConfigModeGetMode\n"));
  if (!configMode) return MemoStatusERR_PARAM;

  memset(&info, 0, sizeof(info));
  info.subcommand = MEMO_SUBCMD_GET_MISC_INFO;
  memoConf *mc = (memoConf *)h;

  r = os_ioctl (mc->memoHandle, KCANY_IOCTL_MEMO_GET_DATA, &info, sizeof(info));

  if (!r) return MemoStatusERR_DEVICE_COMM_ERROR;

  if ((stat = MemoResultToStatus(&info)) != MemoStatusOK) return stat;

  miff = (hMemoDataMiscInfo*) info.buffer;

  *configMode = miff->configMode;
  return MemoStatusOK;
}


//===========================================================================
KvaMemoStatus WINAPI memoConfigModeSetInterval(MemoHandle h, int interval )
{
  int ret = 0;
  memoConf * mc = (memoConf *)h;
  KCANY_CONFIG_MODE io;
  memset(&io, 0, sizeof(io));

    // Firmware 2.0 only accepts (abs(interval) & 0x10) or 0
  if (interval != 0) io.interval = 0x10;
  ret  = os_ioctl(mc->memoHandle, KCANY_IOCTL_MEMO_CONFIG_MODE, &io, sizeof(io));
  if (!ret) return MemoStatusERR_DEVICE_COMM_ERROR;
  else return MemoStatusOK;

}

//===========================================================================
KvaMemoStatus WINAPI memoConfigModeRefresh(MemoHandle h)
{
  int ret = 0;
  memoConf * mc = (memoConf *)h;
  KCANY_CONFIG_MODE io;

  memset(&io, 0, sizeof(io));
  // Firmware 2.0 only accepts (abs(interval) & 0x10) or 0
  io.interval = 0x10;
  ret = os_ioctl (mc->memoHandle, KCANY_IOCTL_MEMO_CONFIG_MODE, &io, sizeof(KCANY_CONFIG_MODE));

  PRINTF(("RefreshInterval KCANY_IOCTL_MEMO_CONFIG_MODE ret=%d\n", ret));

  if (!ret) return MemoStatusERR_DEVICE_COMM_ERROR;
  else return MemoStatusOK;
}



//===========================================================================
KvaMemoStatus WINAPI memoConfigModeSetAutomatic(MemoHandle /* h */ , BOOL /* onoff */)
{
  return MemoStatusERR_NOT_IMPLEMENTED;
}


/*
 * =============================================================================
 *   memoOpenDisk
 * =============================================================================
 */
KvaMemoStatus WINAPI memoOpenDisk(MemoHandle h, uint32 *fileVersion)
{
  memoConf * mc = (memoConf *)h;

  KvaMemoStatus stat;
  LioVolumeDataBlock    *lvdb = NULL;
  diskInfo dInf;
  fileSysInfo fSysInfo;
  uint8_t major, minor;
  int r = 0;
  KCAN_IOCTL_CARD_INFO_2 info2;

  if (!mc) return MemoStatusERR_PARAM;

  // The disk is already opened in firmare through lioInitialize() (which calls
  // dioGetDeviceInfo). We can request the result and check that we got
  // everything, but it's not really used for anything.
  //
  // We must request KCAN_IOCTL_GET_CARD_INFO_2 to get the USB Speed
  // settings when we are connected to a Hydra.
  if (mc->mDevice == memo_MEMO5) {
    r = os_ioctl(mc->memoHandle, KCAN_IOCTL_GET_CARD_INFO_2, &info2, sizeof(info2));

    if (!r) return MemoStatusERR_DEVICE_COMM_ERROR;
    usbSpeed = info2.usb_speed;
    // PRINTF(("USB speed=%d\n", usbSpeed));

    stat = memoReadDiskInfo(h, &dInf);
    if (stat) return stat;

    stat = memoReadFilesystemInfo(h, &fSysInfo);
    if (stat) return stat;
  }

  stat = getLioVolumeDataBlock(h);
  if (stat != MemoStatusOK) {
    return stat;
  }

  lvdb = (LioVolumeDataBlock*) lio_data_buffer;
  splitVersion (lvdb->version, &major, &minor);
  PRINTF(("\nLio Volume Data Block\n"));
  PRINTF(("---------------------\n"));
  PRINTF(("Version: %u.%u\n", major, minor));
  PRINTF(("Magic  : %X\n", lvdb->magicNumber));
  PRINTF(("Sectors: %u\n", lvdb->fileSize));
  PRINTF(("Files  : %u\n", lvdb->numOfKMFFiles));
  PRINTF(("CRC    : 0x%04x\n", lvdb->crc));
  if (LIO_MULTITRACK_VERSION == major) {
    PRINTF(("Tracks : %d\n",lvdb->tracks));
  }

  // Save LIO version and send it during extraction
  splitVersion(lvdb->version, &verMessageFD.lioMajor, &verMessageFD.lioMinor);

  *fileVersion = toVersion(verMessageFD.lioMajor, verMessageFD.lioMinor);

  return MemoStatusOK;
} // memoOpenDisk


/*
 * =============================================================================
 *   reopenDiskReadWrite
 * =============================================================================
 */
static KvaMemoStatus reopenDiskReadWrite(void)
{
  LioResult lres = lio_OK;
  uint32_t size = 0;

  if ((lres = lioOkToShutdown()) != lio_OK)
    return LioResultToStatus(lres);

  (void)lioShutdown();

  if ((lres = lioSetPath_W32(NULL, "rb+")) != lio_OK)
    return LioResultToStatus(lres);

  if ((lres = lioInitialize()) != lio_OK)
    return LioResultToStatus(lres);

   if ((lres = lioAllocDataBlock()) != lio_OK)
    return LioResultToStatus(lres);

  if ((lres = lioOpenLogfile(&size)) != lio_OK)
    return LioResultToStatus(lres);

  return MemoStatusOK;
}

/*
 * =============================================================================
 * reopenDiskReadOnly
 * =============================================================================
 */
static KvaMemoStatus reopenDiskReadOnly(void)
{
  LioResult lres = lio_OK;
  uint32_t size = 0;

  if ((lres = lioOkToShutdown()) != lio_OK)
    return LioResultToStatus(lres);

  (void)lioShutdown();

  if ((lres = lioSetPath_W32(NULL, "rb")) != lio_OK)
    return LioResultToStatus(lres);

  if ((lres = lioInitialize()) != lio_OK)
    return LioResultToStatus(lres);

  if ((lres = lioAllocDataBlock()) != lio_OK)
    return LioResultToStatus(lres);

  if ((lres = lioOpenLogfile(&size)) != lio_OK)
    return LioResultToStatus(lres);

  return MemoStatusOK;
}



/*
 * =============================================================================
 *   memoOpenDiskFile
 * =============================================================================
 */
KvaMemoStatus WINAPI memoOpenDiskFile(MemoHandle h, const char *fn,
                                      uint32 *fileVersion)
{
  LioResult lres = lio_OK;
  memoConf * mc = (memoConf *)h;
  uint32_t size;
  uint8_t major, minor;
  std::string drive, dir, base, ext, path;

  PRINTF(("memoOpenDiskFile\n"));

  if (!fileVersion || !fn) return MemoStatusERR_PARAM;

  std::string filename = std::string(fn);


  // This is a file
  mc->mDevice = memo_DISK5;

  if (!os_splitpath(filename, drive, dir, base, ext) ) {
    return MemoStatusERR_PARAM;
  }
  os_makepath(path, drive, dir, "", "");

  lres = lioSetPath_W32(path.c_str(), "rb");
  if (lres != lio_OK) {
    PRINTF(("[%s,%d] Error %i.\n", __FILE__, __LINE__, lres));
    return LioResultToStatus(lres);
  }

  try {
    lres = lioInitialize();
    if (lres != lio_OK) {
      PRINTF(("[%s,%d] Error %i.\n", __FILE__, __LINE__, lres));
      return LioResultToStatus(lres);
    }
    lres = lioAllocDataBlock();
    if (lres != lio_OK) {
      PRINTF(("Could not allocate memory in logio.\n"));
      return LioResultToStatus(lres);
    }

    lres = lioOpenLogfile(&size);
    if (lres != lio_OK) {
      PRINTF(("lioOpenLogfile -> %d\n", lres));
      return LioResultToStatus(lres);
    }

    // Can only be called after lioOpenLogfile
    lioGetVersion(&major, &minor);
    *fileVersion = toVersion(major, minor);
    // Save LIO version and send it during extraction
    verMessageFD.lioMajor = major;
    verMessageFD.lioMinor = minor;


    mc->kmfFileName = filename;
  }
  catch (int /* err*/ ) {
    return MemoStatusERR_FATAL_ERROR;
  }

  return MemoStatusOK;
} // memoOpenDiskFile


/*
 * =============================================================================
 *   memoCloseDisk
 * =============================================================================
 */
KvaMemoStatus WINAPI memoCloseDisk(MemoHandle h)
{
  LioResult lres;
  KvaMemoStatus res = MemoStatusOK;
  memoConf * mc = (memoConf *)h;

  PRINTF(("memoCloseDisk\n"));

  if ((lres = lioOkToShutdown()) != lio_OK)
    res = LioResultToStatus(lres);

  try {
    lres = lioFreeDataBlock();
    (void)lioShutdown();
    if (lres != lio_OK) {
      PRINTF(("Could not free logio memory.\n"));
    }
  }
  catch (int /*err */) {
    res = LioResultToStatus(lio_FatalError);
  }

  mc->kmfFileName.clear();
  gDiskSize = 0;

  return res;
} // memoCloseDisk


/*
 * =============================================================================
 *   memoReopenDisk
 * =============================================================================
 */
KvaMemoStatus WINAPI memoReopenDisk(MemoHandle h, uint32 *fileVersion)
{
  memoConf *mc = (memoConf *)h;
  KvaMemoStatus stat;

  if (!mc) return MemoStatusERR_PARAM;
  switch (mc->mDevice) {
    case memo_MEMO5:
    {
      stat = memoCloseDisk(h);
      if (stat == MemoStatusOK) stat = memoOpenDisk(h, fileVersion);
      break;
    }

    case memo_DISK5:
    {
      if (!mc->kmfFileName.size()) return MemoStatusERR_PARAM;
      char *tmp = (char*) malloc(mc->kmfFileName.size() + 1);
      if (!tmp) return MemoStatusERR_PARAM;
      strncpy(tmp, mc->kmfFileName.c_str(), mc->kmfFileName.size() + 1);

      stat = memoCloseDisk(h);
      if (stat == MemoStatusOK) stat = memoOpenDiskFile(h, tmp, fileVersion);
      free(tmp);
      break;
    }

    default:
      return MemoStatusERR_PARAM;
  }

  return stat;
}



/*
 * =============================================================================
 *   memoValidateDisk
 * =============================================================================
 */
KvaMemoStatus WINAPI memoValidateDisk(MemoHandle h, uint32 /* complete */, uint32 fix)
{
  KvaMemoStatus stat;
  LioVolumeControlBlock *lvcb = NULL;
  LioVolumeDataBlock    *lvdb = NULL;
  uint32_t lastKnownWPSecNum, lastKnownTriggerSecNum,
              sectorHighWaterMarkSecNum, lastKnownSectorSeqNum;
  uint8_t highWaterHasWrapped, volumeClean, VCBWriteSeqNum, win32FifoMode;
  uint8_t major, minor;

  if (!h) return MemoStatusERR_PARAM;

  // The fix variable is only set if the first validate fails
  if (fix) {
    // At the moment we can only fix differences in the Volume Control blocks
    int vcbMismatch = 0; // 0 - OK, 1 - use info in VCB 0, 2 - use info in VCB 1


    stat = getLioVolumeControlBlock0(h);
    if (stat != MemoStatusOK) {
      return stat;
    }
    lvcb = (LioVolumeControlBlock*) lio_data_buffer;
    lastKnownWPSecNum         = lvcb->lastKnownWPSecNum;
    highWaterHasWrapped       = lvcb->highWaterHasWrapped;
    lastKnownTriggerSecNum    = lvcb->lastKnownTriggerSecNum;
    sectorHighWaterMarkSecNum = lvcb->sectorHighWaterMarkSecNum;
    lastKnownSectorSeqNum     = lvcb->lastKnownSectorSeqNum;
    volumeClean               = lvcb->volumeClean;
    VCBWriteSeqNum            = lvcb->VCBWriteSeqNum;
    win32FifoMode             = lvcb->fifoMode;
    stat = getLioVolumeControlBlock1(h);
    if (stat != MemoStatusOK) {
      return stat;
    }

    // Check whether the vcb's agree and do something intelligent if not.
    // The vcb that indicates the highest high water mark is likely the most recent
    if (sectorHighWaterMarkSecNum != lvcb->sectorHighWaterMarkSecNum) {
      if (sectorHighWaterMarkSecNum < lvcb->sectorHighWaterMarkSecNum) {
        vcbMismatch = 2;
        sectorHighWaterMarkSecNum = lvcb->sectorHighWaterMarkSecNum;
      }
      else {
        vcbMismatch = 1;
      }
    }
    // Otherwise pick the second VCB if it is has exactly one higher seqnum than the first.
    if (VCBWriteSeqNum != lvcb->VCBWriteSeqNum) {
      if (!vcbMismatch) {
        if (VCBWriteSeqNum  == (uint8_t)(lvcb->VCBWriteSeqNum - 1)) {
          vcbMismatch = 2;
        }
        else {
          vcbMismatch = 1;
        }
      }
      if (vcbMismatch == 2) {
        VCBWriteSeqNum  = lvcb->VCBWriteSeqNum;
      }
    }
    if (lastKnownWPSecNum != lvcb->lastKnownWPSecNum) {
      if (vcbMismatch == 2) {
        lastKnownWPSecNum = lvcb->lastKnownWPSecNum;
      }
      else {
        vcbMismatch = 1;
      }
    }
    if (lastKnownTriggerSecNum != lvcb->lastKnownTriggerSecNum) {
      if (vcbMismatch == 2) {
        lastKnownTriggerSecNum = lvcb->lastKnownTriggerSecNum;
      }
      else {
        vcbMismatch = 1;
      }
    }
    if (lastKnownSectorSeqNum != lvcb->lastKnownSectorSeqNum) {
      if (vcbMismatch == 2) {
        lastKnownSectorSeqNum = lvcb->lastKnownSectorSeqNum;
      }
      else {
        vcbMismatch = 1;
      }
    }
    if (volumeClean != lvcb->volumeClean) {
      if (vcbMismatch == 2) {
        volumeClean = lvcb->volumeClean;
      }
      else {
        vcbMismatch = 1;
      }
    }

    if (lastKnownWPSecNum < 8) {
      lastKnownWPSecNum = 8 - 1;
    }

    win32FifoMode = lvcb->fifoMode;

    if (vcbMismatch) {
      stat = reopenDiskReadWrite();
      if (stat != MemoStatusOK && stat != MemoStatusERR_FILE_SYSTEM_CORRUPT) {
        PRINTF(("memoValidateDisk: Could not open file in read-write mode (%d).\n",stat));
        (void) reopenDiskReadOnly();
        return MemoStatusERR_WRITE_PROT;
      }

      if (vcbMismatch == 1) {
        // Copy VCB0 to curDataBlock so we can write it to VCB1.
        lvcb->lastKnownWPSecNum         = lastKnownWPSecNum;
        lvcb->lastKnownTriggerSecNum    = lastKnownTriggerSecNum;
        lvcb->sectorHighWaterMarkSecNum = sectorHighWaterMarkSecNum;
        lvcb->lastKnownSectorSeqNum     = lastKnownSectorSeqNum;
        lvcb->volumeClean               = volumeClean;
        lvcb->VCBWriteSeqNum            = VCBWriteSeqNum;
        lvcb->highWaterHasWrapped       = highWaterHasWrapped;
        lvcb->fifoMode                  = win32FifoMode;
        stat = memoWriteLioSector(h,2);
        if (stat != MemoStatusOK) {
          return stat;
        }
        PRINTF(("Fixed volumecontrol block. Copied block 1->2.\n"));
      }

      if (vcbMismatch == 2) {
        // VCB1 is the last read sector and hence already in curDataBlock.
        stat = memoWriteLioSector(h,1);
        if (stat != MemoStatusOK) {
          return stat;
        }
        PRINTF(("Fixed volumecontrol block. Copied block 2->1.\n"));
      }

      stat = reopenDiskReadOnly();
      if (stat != MemoStatusOK) {
        return stat;
      }
    }
  } // end if(fix)

  // The fix variable is only set if the first validate fails
  if (fix) {
    // At the moment we can only fix differences in the Volume Control blocks
    int vcbMismatch = 0; // 0 - OK, 1 - use info in VCB 0, 2 - use info in VCB 1


    stat = getLioVolumeControlBlock0(h);
    if (stat != MemoStatusOK) {
      return stat;
    }
    lvcb = (LioVolumeControlBlock*) lio_data_buffer;
    lastKnownWPSecNum         = lvcb->lastKnownWPSecNum;
    highWaterHasWrapped       = lvcb->highWaterHasWrapped;
    lastKnownTriggerSecNum    = lvcb->lastKnownTriggerSecNum;
    sectorHighWaterMarkSecNum = lvcb->sectorHighWaterMarkSecNum;
    lastKnownSectorSeqNum     = lvcb->lastKnownSectorSeqNum;
    volumeClean               = lvcb->volumeClean;
    VCBWriteSeqNum            = lvcb->VCBWriteSeqNum;
    win32FifoMode             = lvcb->fifoMode;
    stat = getLioVolumeControlBlock1(h);
    if (stat != MemoStatusOK) {
      return stat;
    }

    // Check whether the vcb's agree and do something intelligent if not.
    // The vcb that indicates the highest high water mark is likely the most recent
    if (sectorHighWaterMarkSecNum != lvcb->sectorHighWaterMarkSecNum) {
      if (sectorHighWaterMarkSecNum < lvcb->sectorHighWaterMarkSecNum) {
        vcbMismatch = 2;
        sectorHighWaterMarkSecNum = lvcb->sectorHighWaterMarkSecNum;
      }
      else {
        vcbMismatch = 1;
      }
    }
    // Otherwise pick the second VCB if it is has exactly one higher seqnum than the first.
    if (VCBWriteSeqNum != lvcb->VCBWriteSeqNum) {
      if (!vcbMismatch) {
        if (VCBWriteSeqNum  == (uint8_t)(lvcb->VCBWriteSeqNum - 1)) {
          vcbMismatch = 2;
        }
        else {
          vcbMismatch = 1;
        }
      }
      if (vcbMismatch == 2) {
        VCBWriteSeqNum  = lvcb->VCBWriteSeqNum;
      }
    }
    if (lastKnownWPSecNum != lvcb->lastKnownWPSecNum) {
      if (vcbMismatch == 2) {
        lastKnownWPSecNum = lvcb->lastKnownWPSecNum;
      }
      else {
        vcbMismatch = 1;
      }
    }
    if (lastKnownTriggerSecNum != lvcb->lastKnownTriggerSecNum) {
      if (vcbMismatch == 2) {
        lastKnownTriggerSecNum = lvcb->lastKnownTriggerSecNum;
      }
      else {
        vcbMismatch = 1;
      }
    }
    if (lastKnownSectorSeqNum != lvcb->lastKnownSectorSeqNum) {
      if (vcbMismatch == 2) {
        lastKnownSectorSeqNum = lvcb->lastKnownSectorSeqNum;
      }
      else {
        vcbMismatch = 1;
      }
    }
    if (volumeClean != lvcb->volumeClean) {
      if (vcbMismatch == 2) {
        volumeClean = lvcb->volumeClean;
      }
      else {
        vcbMismatch = 1;
      }
    }

    if (lastKnownWPSecNum < 8) {
      lastKnownWPSecNum = 8 - 1;
    }

    win32FifoMode = lvcb->fifoMode;

    if (vcbMismatch) {
      stat = reopenDiskReadWrite();
      if (stat != MemoStatusOK && stat != MemoStatusERR_FILE_SYSTEM_CORRUPT) {
        PRINTF(("memoValidateDisk: Could not open file in read-write mode (%d).\n",stat));
        (void) reopenDiskReadOnly();
        return MemoStatusERR_WRITE_PROT;
      }

      if (vcbMismatch == 1) {
        // Copy VCB0 to curDataBlock so we can write it to VCB1.
        lvcb->lastKnownWPSecNum         = lastKnownWPSecNum;
        lvcb->lastKnownTriggerSecNum    = lastKnownTriggerSecNum;
        lvcb->sectorHighWaterMarkSecNum = sectorHighWaterMarkSecNum;
        lvcb->lastKnownSectorSeqNum     = lastKnownSectorSeqNum;
        lvcb->volumeClean               = volumeClean;
        lvcb->VCBWriteSeqNum            = VCBWriteSeqNum;
        lvcb->highWaterHasWrapped       = highWaterHasWrapped;
        lvcb->fifoMode                  = win32FifoMode;
        stat = memoWriteLioSector(h,2);
        if (stat != MemoStatusOK) {
          return stat;
        }
        PRINTF(("Fixed volumecontrol block. Copied block 1->2.\n"));
      }

      if (vcbMismatch == 2) {
        // VCB1 is the last read sector and hence already in curDataBlock.
        stat = memoWriteLioSector(h,1);
        if (stat != MemoStatusOK) {
          return stat;
        }
        PRINTF(("Fixed volumecontrol block. Copied block 2->1.\n"));
      }

      stat = reopenDiskReadOnly();
      if (stat != MemoStatusOK) {
        return stat;
      }
    }
  } // end if(fix)

  stat = getLioVolumeDataBlock(h);
  if (stat != MemoStatusOK) {
    return MemoStatusERR_DISK_ERROR;
  }
  lvdb = (LioVolumeDataBlock*) lio_data_buffer;
  splitVersion (lvdb->version, &major, &minor);
  if (0) {
    printf("\nLio Volume Data Block\n");
    printf("---------------------\n");
    printf("Version: %u.%u\n", major, minor);
    printf("Magic  : %x\n", lvdb->magicNumber);
    printf("Sectors: %u\n", lvdb->fileSize);
    printf("Files  : %u\n", lvdb->numOfKMFFiles);
    printf("CRC    : 0x%04x\n", lvdb->crc);
  }
  if (major == LIO_MULTITRACK_VERSION) {
    PRINTF(("Tracks : %d\n",lvdb->tracks));
  }

  if (lvdb->magicNumber != CONFIG_MAGIC ||
      lvdb->version == 0 ||
      lvdb->fileSize == 0 ||
      lvdb->numOfKMFFiles == 0)
  {
    return MemoStatusERR_NOT_FORMATTED;
  }


  stat = getLioVolumeControlBlock0(h);
  if (stat != MemoStatusOK) {
    return stat;
  }
  lvcb = (LioVolumeControlBlock*) lio_data_buffer;
  lastKnownWPSecNum         = lvcb->lastKnownWPSecNum;
  highWaterHasWrapped       = lvcb->highWaterHasWrapped;
  lastKnownTriggerSecNum    = lvcb->lastKnownTriggerSecNum;
  sectorHighWaterMarkSecNum = lvcb->sectorHighWaterMarkSecNum;
  lastKnownSectorSeqNum     = lvcb->lastKnownSectorSeqNum;
  volumeClean               = lvcb->volumeClean;
  VCBWriteSeqNum            = lvcb->VCBWriteSeqNum;
  win32FifoMode             = lvcb->fifoMode;
  if (0) {
    printf("--------------------------\n");
    printf("lastKnownWPSecNum         : %u\n", lastKnownWPSecNum);
    printf("lastKnownTriggerSecNum    : %u\n", lastKnownTriggerSecNum);
    printf("lastKnownSectorSeqNum     : %u\n", lastKnownSectorSeqNum);
    printf("sectorHighWaterMarkSecNum : %u\n", sectorHighWaterMarkSecNum);
    printf("highWaterHasWrapped       : %u\n",  highWaterHasWrapped);
    printf("volumeClean               : %u\n",  volumeClean);
    printf("VCBWriteSeqNum            : %u\n",  VCBWriteSeqNum);
    printf("CRC                       : 0x%04x\n", lvcb->crc);
  }
  stat = getLioVolumeControlBlock1(h);
  if (stat != MemoStatusOK) {
    return stat;
  }
  PRINTF(("\nLio Volume Control Block 1\n"));
  PRINTF(("--------------------------\n"));
  PRINTF(("lastKnownWPSecNum         : %u\n", lvcb->lastKnownWPSecNum));
  PRINTF(("lastKnownTriggerSecNum    : %u\n", lvcb->lastKnownTriggerSecNum));
  PRINTF(("lastKnownSectorSeqNum     : %u\n", lvcb->lastKnownSectorSeqNum));
  PRINTF(("sectorHighWaterMarkSecNum : %u\n", lvcb->sectorHighWaterMarkSecNum));
  PRINTF(("highWaterHasWrapped       : %u\n", lvcb->highWaterHasWrapped));
  PRINTF(("volumeClean               : %u\n", lvcb->volumeClean));
  PRINTF(("VCBWriteSeqNum            : %u\n", lvcb->VCBWriteSeqNum));
  PRINTF(("CRC                       : 0x%04x\n", lvcb->crc));

  if ( lastKnownWPSecNum         != lvcb->lastKnownWPSecNum ||
       highWaterHasWrapped       != lvcb->highWaterHasWrapped ||
       lastKnownTriggerSecNum    != lvcb->lastKnownTriggerSecNum ||
       sectorHighWaterMarkSecNum != lvcb->sectorHighWaterMarkSecNum ||
       lastKnownSectorSeqNum     != lvcb->lastKnownSectorSeqNum ||
       volumeClean               != lvcb->volumeClean ||
       VCBWriteSeqNum            != lvcb->VCBWriteSeqNum ||
       win32FifoMode             != lvcb->fifoMode ) {
       PRINTF(("Mismatching in Lio Volume Control Blocks\n"));
       return MemoStatusERR_FILE_SYSTEM_CORRUPT;
  }

  // Check that the trigger chain is ok
  if (!triggerList.count) {
    uint32 fileCount;
    PRINTF(("Seaching for triggers."));
    stat = memoGetLogFileCountEx(h, &fileCount);
    if (stat != MemoStatusOK) {
      return stat;
    }
  }

  if (triggerList.count) {
    PRINTF(("Checking trigger chain for %d triggers.\n", triggerList.count));
    TriggerItem *triggerItem = NULL;
    Trigger *t = NULL;
    Trigger *tp = NULL;
    uint32_t prevTriggerBlockSecNum = 0;

    triggerItem = triggerList.first;
    t = &triggerItem->trigger;
    tp = t;
    prevTriggerBlockSecNum = t->prevTriggerBlockSecNum;
    triggerItem = triggerItem->next;
    while (triggerItem != NULL) {
      t = &triggerItem->trigger;
      if (t->secNum != prevTriggerBlockSecNum && t->track == tp->track) {
          PRINTF(("Trigger chain broken: %u -> %u\n",prevTriggerBlockSecNum, t->secNum));
          if (t->isHkTrigger) {
            PRINTF(("Broken by reconstructed house-keeping trigger, which is OK.\n"));
          } else {
            return MemoStatusERR_FILE_SYSTEM_CORRUPT;
          }
      }
      // PRINTF(("[%lu -> %lu]\n",prevTriggerBlockSecNum, t->secNum));
      prevTriggerBlockSecNum = t->prevTriggerBlockSecNum;
      tp = t;
      triggerItem = triggerItem->next;
     }
  }

  return MemoStatusOK;
}

/*
 * =============================================================================
 *   memoGetLogFileCount (obsolete; use memoGetLogFileCountEx instead)
 * =============================================================================
 */
KvaMemoStatus WINAPI memoGetLogFileCountEx(MemoHandle h, uint32* fileCount)
{
  KvaMemoStatus status;

  Trigger trig;
  LioVolumeControlBlock *lvcb = NULL;
  LioVolumeDataBlock    *lvdb = NULL;
  LioTriggerBlock       *ltb  = NULL;
  PRINTF(("memoGetLogFileCountEx\n"));

  if (!h) return MemoStatusERR_PARAM;
  if (!fileCount) return MemoStatusERR_PARAM;
  *fileCount = 0;

  // Invalidate cache
  gFastIo.count = 0;

  // Delete the previous list if needed
  if (triggerList.count) {
    PRINTF(("Deleting the previous trigger list.\n"));
    DeleteTriggerList(&triggerList);
  }

  // Get partitions
  status = getTracksFromLioDataBlock(h, &triggerList);
  lvdb = (LioVolumeDataBlock*) lio_data_buffer;
  gDiskSize = lvdb->fileSize;

  for (int k = 0; k < triggerList.tracks;k++) {
    // Search through the tracks in reverse order
    int idx = triggerList.tracks - (k+1);
    uint32_t newestTriggerSecNum = 0;
    uint32_t triggerBlockSecNum  = 0;
    uint32_t lastKnownWPSecNum   = 0;
    uint8_t fFifo = 0;
    uint8_t fWrap = 0;
    status = getLioDataBlock(h, triggerList.trackStart[idx]+1);
    if (status != MemoStatusOK) {
      return status;
    }
    lvcb = (LioVolumeControlBlock*) lio_data_buffer;

    // Check if the disk has wrapped around in FIFO mode.
    fWrap = lvcb->highWaterHasWrapped;

    // Get the last known written trigger block
    newestTriggerSecNum = lvcb->lastKnownTriggerSecNum;
    triggerBlockSecNum  = newestTriggerSecNum;
    lastKnownWPSecNum   = lvcb->lastKnownWPSecNum;

    if (!fWrap && newestTriggerSecNum == 0) {
      PRINTF(("This track contains no triggers.\n"));
      continue;
    }

    // Read through the chain of trigger blocks
    ltb = (LioTriggerBlock*) lio_data_buffer;
    while (status == MemoStatusOK) {
      uint32_t olderTriggerSecNum  = 0;

      status = getLioDataBlock(h, triggerBlockSecNum);
      if(ltb->magic != TRIGGER_MAGIC) {
        // Not a trigger
        PRINTF(("<not a trig>"));
        break;
      }

      // Check that the oldest pre-trigger isn't overwritten by the newest trigger
      if (fFifo && ltb->preTriggerBlockSecNum != 0 && ltb->preTriggerBlockSecNum <= lastKnownWPSecNum) {
        PRINTF(("<overlap>"));
        // Do not use house-keeping as trigger
        fWrap = 0;
        break;
      }
      //printTHinfo(ltb);

      // Copy the current trigger to the list
      trig.secNum                 = triggerBlockSecNum;
      trig.prevTriggerBlockSecNum = ltb->prevTriggerBlockSecNum;
      trig.prevWPSecNum           = ltb->prevWPSecNum;
      trig.preTriggerBlockSecNum  = ltb->preTriggerBlockSecNum;
      trig.preTriggerIsWrapped    = ltb->preTriggerIsWrapped;
      trig.preTriggerSectors      = ltb->preTriggerSectors;
      trig.isHkTrigger        = 0;
      trig.track = idx;
      trig.trackStart = triggerList.trackStart[idx];
      trig.trackEnd   = triggerList.trackEnd[idx];
      trig.trackIsWrapped    = fWrap;
      trig.lastWrittenSector = lastKnownWPSecNum;
      CopyTriggerToList(&triggerList, &trig);

      // Get the prevoius trigger
      olderTriggerSecNum = ltb->prevTriggerBlockSecNum;

      if (olderTriggerSecNum == 0 ||
          (fWrap && (olderTriggerSecNum == newestTriggerSecNum)))
      {
        // The first trigger in a file that has not wrapped around or the only
        // trigger in a wrapped file.
        PRINTF(("<zero>"));
        break;
      }
      if (fWrap && !fFifo && olderTriggerSecNum > triggerBlockSecNum) {
        // We have wrapped around and should continue to read while
        // olderTriggerSectNum > newestTriggerSecNum
        fFifo = 1;
        PRINTF(("<fifo>\n"));
      }
      // The triggers might form a seemless chain, thus the less or equal.
      if (fFifo && newestTriggerSecNum >= olderTriggerSecNum) {
          // The first block in a wrapped file
          PRINTF(("<break>"));
          break;
      }
      // Jump to next trigger
      triggerBlockSecNum = olderTriggerSecNum;
    }

    if (fWrap) {
      Trigger* pt = NULL;
      // Since we have wrapped there is most likely a trigger without a proper
      // trigger block also, hence check for house keeping blocks that doesn't
      // belong to any previously found triggers.

      PRINTF(("Searching for house keeping blocks belonging to the oldest file.\n"));

      uint32_t currentSector = lastKnownWPSecNum;
      uint32_t houseSector = LIO_HOUSEKEEPING_SECTOR * (1+currentSector / LIO_HOUSEKEEPING_SECTOR);

      if (houseSector >= triggerList.trackEnd[idx]) {
        // Wrap around
        houseSector = LIO_HOUSEKEEPING_SECTOR * (1+triggerList.trackStart[idx] / LIO_HOUSEKEEPING_SECTOR);
         PRINTF(("Wrapping housekeeping block to %u\n", houseSector ));
      }
      PRINTF(("houseSector: %u\n", houseSector));


      // Check if the housekeeping block is in the free space between the last written
      // sector and the oldest of the (if any) remaining triggers.
      if (countTriggersOnTrack(&triggerList, idx)) {
        uint32_t firstKnownWPSecNum = 0;

        pt = GetTrigger(&triggerList, 0);
        //PRINTF(("0 : %lu <-p: %lu s: %lu pre-wp: %lu\n", pt->prevTriggerBlockSecNum, pt->preTriggerBlockSecNum,
        //pt->secNum, pt->prevWPSecNum));
        firstKnownWPSecNum = pt->secNum;
        if (pt->preTriggerBlockSecNum) {
          firstKnownWPSecNum = pt->preTriggerBlockSecNum;
        }
        //PRINTF(("[%lu ? %lu ? %lu]", lastKnownWPSecNum, houseSector, firstKnownWPSecNum));
        if (lastKnownWPSecNum < firstKnownWPSecNum ) {
          if (lastKnownWPSecNum < houseSector && houseSector < firstKnownWPSecNum)
          {
            //PRINTF(("[%lu < %lu < %lu]", lastKnownWPSecNum, houseSector, firstKnownWPSecNum));
          }
          else {
            houseSector = 0;
          }
        } else {
          if ((lastKnownWPSecNum < houseSector && houseSector < triggerList.trackEnd[idx]) ||
              (triggerList.trackStart[idx] <  houseSector && houseSector < firstKnownWPSecNum))
            {
            //PRINTF(("[%lu : %lu : %lu]", lastKnownWPSecNum, houseSector, firstKnownWPSecNum));
          }
          else {
            houseSector = 0;
          }
        }
      }



      if (houseSector >= triggerList.trackEnd[idx] || houseSector <= triggerList.trackStart[idx]) {
        PRINTF(("<housekeeping block outside track>"));
        houseSector = 0;
      }
      if (houseSector) {
        status = getLioDataBlock(h, houseSector);
        if(status == MemoStatusOK && ltb->magic == HOUSE_MAGIC) {
          PRINTF(("Found a housekeeping trigger in sector %u.\n",houseSector));
          // Pretend that this house keeping block is a trigger. Note that a
          // pretrigger that didn't trigger might have overwritten all sectors
          // before the trigger. 
          trig.secNum                 = houseSector;
          trig.prevTriggerBlockSecNum = ltb->prevTriggerBlockSecNum;
          trig.prevWPSecNum           = ltb->prevWPSecNum;
          trig.preTriggerBlockSecNum  = 0; // Use only sectors after the trigger
          trig.preTriggerIsWrapped    = 0;
          trig.preTriggerSectors      = 0;
          trig.isHkTrigger            = 1;
          trig.track = idx;
          trig.trackStart = triggerList.trackStart[idx];
          trig.trackEnd   = triggerList.trackEnd[idx];
          trig.trackIsWrapped    = fWrap;
          trig.lastWrittenSector = lastKnownWPSecNum;
          CopyTriggerToList(&triggerList, &trig);
        }
      }
    }
  } // End for loop over partitions

  *fileCount = triggerList.count;

  // For debug
  PrintReverseTriggerList(&triggerList);
  PRINTF(("<fileCount=%u>", *fileCount));

  return MemoStatusOK;
} // memoGetLogFileCountEx

/*
 * =============================================================================
 *   openLogFile
 * =============================================================================
 */
static KvaMemoStatus openLogFile(MemoHandle h,
                                 uint32 fileNo,
                                 uint64 *eventCount)
{

  memoConf * mc = (memoConf *)h;
  KvaMemoStatus stat;
  uint32_t sectors = 0;
  uint64_t numberOfEvents = 0;
  int i;


  Trigger *pTrig = NULL;
  LioTriggerBlock *ltb        = NULL;

  PRINTF(("OF %u\n", fileNo));

  if (!h) return MemoStatusERR_PARAM;
  if (mc->handleType != MemoHandleMemorator) return MemoStatusERR_PARAM;

  pTrig = GetTrigger(&triggerList, fileNo);
  if (pTrig == NULL) {
    PRINTF(("Could not open trigger %u.\n", fileNo));
    return MemoStatusFail;
  }

  mc->trigger = pTrig;

  // Calculate approx. number of events
  sectors = getNoSectors(&triggerList, fileNo);
  numberOfEvents = (uint64_t)((uint64_t)sectors * (uint64_t)20);
  if (eventCount) {
    *eventCount = numberOfEvents;
  }

  // Read the triggerBlock

  stat = getLioDataBlock(h, pTrig->secNum);
  if (stat != MemoStatusOK ) {
    return stat;
  }

  ltb = (LioTriggerBlock*) lio_data_buffer;
  if (!(ltb->magic == TRIGGER_MAGIC || ltb->magic == HOUSE_MAGIC)) {
    PRINTF(("Trigger %u does not point to a trigger block (%u).\n", fileNo,
            pTrig->secNum));
    return MemoStatusFail;
  }

  mc->verMajor = ltb->fwver_major;
  mc->verMinor = ltb->fwver_minor;
  for (i = 0; i < MAX_NO_CHANNELS; i++) {
    mc->transType[i] = ltb->channel[i].transType;
    mc->bitrate[i] = lioComputeBitrate(&ltb->channel[i]);
  }
  mc->channels = ltb->channels;

  mc->hiresTimerFqMHz = ltb->hiresTimerFqMHz;
  mc->trigger->serialNumber = ltb->serialNumber;
  mc->trigger->eanHi = ltb->eanhi;
  mc->trigger->eanLo = ltb->eanlo;

  // Save version information
  mc->sendVersion = true;
  verMessageFD.fwMajor = ltb->fwver_major;
  verMessageFD.fwMinor = ltb->fwver_minor;
  verMessageFD.fwBuild = ltb->fwver_build;
  verMessageFD.eanHi   = ltb->eanhi;
  verMessageFD.eanLo   = ltb->eanlo;
  verMessageFD.serialNumber = ltb->serialNumber;

  // Get start time:
  if (mc->trigger->isHkTrigger) {
    PRINTF(("Housekeeping trigger."));
    stat = loadLioDataSector(h);
    if (stat) {
      PRINTF(("Could not load data sector for initial time use.\n"));
    }

    mc->trigger->isHkTrigger = 1;

    mc->startTime = ((LioDataBlock*) mc->ld)->unixTime;
  }
  else {
    mc->startTime = ltb->unixTime;
  }

  // Get last sector for reading end time:
  //   Initialise endTime to start time, so we get something even if it fails later:
  mc->endTime = mc->startTime;
  LioDataBlock *ldb = NULL;
  uint32_t endSector = 0;
  endSector = getEndSector(&triggerList, fileNo);
  stat = getLioDataBlock(h, endSector);

  if (stat == MemoStatusOK ) {
    if (!(ltb->magic == TRIGGER_MAGIC ||
          ltb->magic == HOUSE_MAGIC   ||
          ltb->magic == PRETRIGGER_MAGIC) ) {
      // If none of these magics, then it is datablock:
      ldb = (LioDataBlock*) lio_data_buffer;
      mc->endTime = ldb->unixTime;
    } else {
      // A file can end with a housekeeping block
      mc->endTime = ltb->unixTime;
    }
  } else {
    PRINTF(("Could not read endTime, set it to startTime.\n"));
  }

  mc->ldPos = 0;
  mc->currentSector = 0;
  mc->currentSequence = 0;

#if defined(DEBUG)
  unsigned int yy, mm,dd, ho, m, s;
  PRINTF(("StartTime = unix time -> %ld\n", mc->startTime));
  unix_time (mc->startTime, &yy, &mm, &dd, &ho, &m, &s);
  PRINTF(("            %d-%02d-%02d %02d:%02d.%02d", yy, mm, dd, ho, m, s));

  PRINTF(("EndTime   = unix time -> %ld\n", mc->endTime));
  unix_time (mc->endTime, &yy, &mm, &dd, &ho, &m, &s);
  PRINTF(("            %d-%02d-%02d %02d:%02d.%02d", yy, mm, dd, ho, m, s));
#endif
  return MemoStatusOK;
}

/*
 * =============================================================================
 *   debugprintDate
 * =============================================================================
 */
static void debugprintDate (const uint64_t wallclock)
{
#if defined(DEBUG)
  unsigned int yy, mm, dd, ho, m, s;
  unix_time ((uint32_t) (wallclock / ONE_BILLION), &yy, &mm, &dd, &ho, &m, &s);
  PRINTF(("Date: %d-%02d-%02d %02d:%02d.%02d", yy, mm, dd, ho, m, s));
#else
  if (wallclock) {
    // Unused variable
  }
#endif
}

/*
 * =============================================================================
 *   createInitialRTC
 * =============================================================================
 */
static KvaMemoStatus createInitialRTC(MemoHandle h, hydraHostCmdLogFD **e)
{
  memoConf * mc = (memoConf *)h;
  KvaMemoStatus stat = MemoStatusOK;
  LioTriggerBlock *ltb = NULL;
  uint64_t wallclock = 0;    // RTC in nano seconds
  uint64_t timestamp = 0;    // Timestamp in clock cycles
  uint64_t minTimestamp = 0; // Earliest timestamp in first data sector
  *e = NULL;

  stat = getLioDataBlock(h, (mc->trigger)->secNum);
  if (stat == MemoStatusOK) {
    ltb = (LioTriggerBlock*)  lio_data_buffer;

    wallclock = (uint64_t)ltb->unixTime * ONE_BILLION + (uint64_t) ltb->unixTimeSubsecond;

    if (mc->trigger->isHkTrigger) {
      // Create a new trigger from this housekeeping block on next call
      // by saving timestamp in global variable rtcMessageFD.
      mc->createTrigger = true;
      timestamp = ltb->usTime;
    } else {
      // Save the timestamp that created the trigger block
      timestamp = ltb->timestamp;

      // Correct for the (small) time difference between the CAN event that
      // created the trigger and trigger block creation that sets unixTime.
      wallclock -= mc->toNs(ltb->usTime - timestamp);
    }
    debugprintDate(wallclock);
  }

  // Load the actual data sector
  stat = loadLioDataSector(h);
  if (MemoStatusOK != stat) {
    PRINTF(("Could not load data sector."));
    return stat;
  }

  // Correct the RTC time by finding the earliest timestamp in the first sector
  minTimestamp = timestamp;
  while (mc->ldPos < LIO_DATA_PAYLOAD_SIZE) {
    hcmdLogHead *msg = NULL;
    msg = (hcmdLogHead*) &((mc->ld)->data[mc->ldPos]);
    if (msg->cmdLen == 0) {
      break;
    }
    else {
      uint64_t tmp = 0;
      mc->ldPos += msg->cmdLen;
      switch (msg->cmdNo) {
        case CMD_LOG_RTC_TIME_FD:
          memcpy(&tmp, ((hcmdLogRtcTimeFD*) msg)->time, sizeof(tmp));
          break;

        case CMD_RX_MESSAGE_FD:
          memcpy(&tmp, ((hcmdLogMessageFD*) msg)->time, sizeof(tmp));
          break;

        case CMD_LOG_TRIG_FD:
          memcpy(&tmp, ((hcmdLogTrigFD*) msg)->time, sizeof(tmp));
          break;
      }
      if (tmp < minTimestamp) minTimestamp = tmp;
    }
  }

  // Correct for the (large) time difference between first CAN event in the
  // pre-trigger and trigger block creation that sets unixTime or the
  // (very small) difference depending on which channel FPGA read first
  wallclock -= mc->toNs(timestamp - minTimestamp);
  timestamp = minTimestamp;
  debugprintDate(wallclock);

  rtcMessageFD.unixTime = (uint32_t) (wallclock / ONE_BILLION);
  rtcMessageFD.unixTimeSubsecond = (uint32_t) (wallclock % ONE_BILLION);
  memcpy(rtcMessageFD.time, &timestamp, sizeof(rtcMessageFD.time));
  *e = (hydraHostCmdLogFD*) &rtcMessageFD;

  // Reset position
  mc->ldPos = 0;
  return stat;
}

/*
 * =============================================================================
 *   readLogFile
 * =============================================================================
 */
static KvaMemoStatus readLogFile(MemoHandle h, hydraHostCmdLogFD **e)
{
  memoConf * mc = (memoConf *)h;
  KvaMemoStatus stat = MemoStatusOK;

  *e = NULL;
  if (!mc->trigger) return MemoStatusERR_LOGFILEOPEN;

  // Create and send a version message and then a RTC message
  if (mc->currentSector == 0) {
    if (mc->sendVersion) {
      *e = (hydraHostCmdLogFD*) &verMessageFD;
      mc->sendVersion = false;
    } else {
      stat = createInitialRTC(h, e);
    }
    return stat;
  }

  if (mc->createTrigger) {
    PRINTF(("Sending housekeeping trigger."));
    // Use time from previous (global) RTC message
    trgMessageFD.time[0] = rtcMessageFD.time[0];
    trgMessageFD.time[1] = rtcMessageFD.time[1];
    *e = (hydraHostCmdLogFD*) &trgMessageFD;
    mc->createTrigger = false;
    return MemoStatusOK;
  }

  while (stat == MemoStatusOK) {
    hcmdLogHead *msg = NULL;

    // Load a new data sector if needed
    if (mc->currentSector == 0 || mc->ldPos >= LIO_DATA_PAYLOAD_SIZE) {
      mc->ldPos = 0;
      stat = loadLioDataSector(h);
      if (stat != MemoStatusOK) {
        mc->currentSector   = 0;
        mc->currentSequence = 0;
        mc->ld = NULL;
        return stat;
      }
    }

    // Read one message from the sector
    msg = (hcmdLogHead*) &((mc->ld)->data[mc->ldPos]);

    if (msg->cmdLen == 0) {
      // Try next sector
      mc->ldPos = LIO_DATA_PAYLOAD_SIZE;
    }
    else {
      // Found a valid message
      mc->ldPos += msg->cmdLen;
      *e = (hydraHostCmdLogFD*) msg;
      break;
    }
  }

  return MemoStatusOK;
}

/*
 * =============================================================================
 *   closeLogFile
 * =============================================================================
 */
static void closeLogFile(MemoHandle h)
{
  memoConf * mc = (memoConf *)h;
  PRINTF(("CF"));
  if (mc->trigger) {
    if (mc->trigger->isHkTrigger) {
      mc->trigger->isHkTrigger = 1;
    }
  }
} // closeLogFile


/*
 * =============================================================================
 *   updateTimeStamp
 * =============================================================================
 */
static void updateTimeStamp(memoConfBase &mc, hydraHostCmdLogFD *e)
{
  uint64_t x;
  if (!mc.hiresTimerFqMHz) return;

  switch (((hcmdLogHead*)e)->cmdNo) {
    case CMD_RX_MESSAGE_FD:
    {
      memcpy(&x, e->logMessageFD.time, sizeof(x));
      x = mc.toNs(x);
      mc.updateTime(x);
      break;
    }

    case CMD_LOG_TRIG_FD:
    {
      memcpy(&x, e->logTrigFD.time, sizeof(x));
      x = mc.toNs(x);
      mc.updateTime(x);
      break;
    }

    case CMD_LOG_RTC_TIME_FD:
    {
      memcpy(&x, e->logRtcTimeFD.time, sizeof(x));
      x = mc.toNs(x);
      mc.updateTime(x);
      break;
    }

    default:
      // Timestamp nonexistent or deliberately ignored.
      break;
  }
} // updateTimeStamp


/*
 * =============================================================================
 *   memoDiskReadMultipleSectors
 * =============================================================================
 */
KvaMemoStatus WINAPI memoDiskReadMultipleSectors(MemoHandle h, uint32 sectorNo, uint16 count,
                                                void* buf, size_t bufsize)
{
  int ret = 0;
  memoConf * mc = (memoConf *)h;
  KCANY_MEMO_DISK_IO_FAST io;
  memset(&io, 0, sizeof(io));

  io.subcommand   = KCANY_MEMO_DISK_IO_FASTREAD_LOGICAL_SECTOR;
  io.first_sector = sectorNo;
  io.count        = count;

  ret = os_ioctl(mc->memoHandle, KCANY_IOCTL_MEMO_DISK_IO_FAST, &io, sizeof(io));

  if (!ret) {
    return MemoStatusFail;
  }

  if (io.dio_status >= dio_Pending) {
    return DioResultToStatus((DioResult)io.dio_status);
  } else {
    memcpy(buf, &(io.buffer[0][0]), bufsize);
  }
  return MemoStatusOK;
} // memoDiskReadPhysicalSector


/*
 * =============================================================================
 *   memoDiskReadPhysicalSector
 * =============================================================================
 */
KvaMemoStatus WINAPI memoDiskReadPhysicalSector(MemoHandle h, uint32 sectorNo,
                                                void* buf, size_t bufsize)
{
  int ret = 0;
  KCANY_MEMO_DISK_IO io;
  memoConf * mc = (memoConf *)h;

  if (bufsize != SD_DISK_SECTOR_SIZE) return MemoStatusERR_PARAM;

  memset(&io, 0, sizeof(io));
  io.subcommand = KCANY_MEMO_DISK_IO_READ_PHYSICAL_SECTOR;
  io.first_sector = sectorNo;
  io.count = 1;

  ret = os_ioctl(mc->memoHandle,KCANY_IOCTL_MEMO_DISK_IO, &io, sizeof(io));

  if (!ret) return MemoStatusERR_DEVICE_COMM_ERROR;

  memcpy(buf, &io.buffer, bufsize);
  if (io.dio_status >= dio_Pending) {
    PRINTF(("memoDiskReadPhysicalSector ERROR, dio=%d", io.dio_status));
    return DioResultToStatus((DioResult)io.dio_status);
  }

  return MemoStatusOK;
} // memoDiskReadPhysicalSector

/*
 * =============================================================================
 *   memoDiskReadLogicalSector
 * =============================================================================
 */
KvaMemoStatus WINAPI memoDiskReadLogicalSector(MemoHandle h, uint32 sectorNo,
                                               void* buf, size_t bufsize)
{
  int ret = 0;
  KCANY_MEMO_DISK_IO io;
  memoConf * mc = (memoConf *)h;

  if (bufsize != SD_DISK_SECTOR_SIZE) return MemoStatusERR_PARAM;

  memset(&io, 0, sizeof(io));
  io.subcommand = KCANY_MEMO_DISK_IO_READ_LOGICAL_SECTOR;
  io.first_sector = sectorNo;
  io.count = 1;

  ret = os_ioctl(mc->memoHandle, KCANY_IOCTL_MEMO_DISK_IO, &io, sizeof(io));

  if (!ret) return MemoStatusERR_DEVICE_COMM_ERROR;

  memcpy(buf, &io.buffer, bufsize);
  if (io.dio_status >= dio_Pending)
    return DioResultToStatus((DioResult)io.dio_status);
  return MemoStatusOK;
} // memoDiskReadLogicalSector

/*
 * =============================================================================
 *   localDiskReadLogicalSector
 * =============================================================================
 */
 // We can't use functions in the API for some reason (linking fails, since the
 // dll is not built yet) so this is a quick 'n' dirty fix for now.
static KvaMemoStatus localDiskReadLogicalSector(MemoHandle h, unsigned int sectorNo,
                                               void* buf, size_t bufsize)
{
  int ret = 0;
  KCANY_MEMO_DISK_IO io;
  memoConf * mc = (memoConf *)h;

  if (bufsize != SD_DISK_SECTOR_SIZE) return MemoStatusERR_PARAM;

  memset(&io, 0, sizeof(io));
  io.subcommand = KCANY_MEMO_DISK_IO_READ_LOGICAL_SECTOR;
  io.first_sector = sectorNo;
  io.count = 1;

  ret = os_ioctl (mc->memoHandle, KCANY_IOCTL_MEMO_DISK_IO, &io, sizeof(io));
  if (!ret) return MemoStatusERR_DEVICE_COMM_ERROR;

  memcpy(buf, &io.buffer, bufsize);
  if (io.dio_status >= dio_Pending)
    return DioResultToStatus((DioResult)io.dio_status);
  return MemoStatusOK;
} // localDiskReadLogicalSector

/*
 * =============================================================================
 *   memoDiskReadLogicalSectorFast
 * =============================================================================
 */
static KvaMemoStatus WINAPI memoDiskReadLogicalSectorFast(MemoHandle h, unsigned int sectorNo)
{
  int ret = 0;
  memoConf * mc = (memoConf *)h;


  // Check if the desired sector is already in memory
  if ((gFastIo.count != 0 && gFastIo.first_sector <= sectorNo) &&
        (sectorNo <= gFastIo.first_sector + gFastIo.count - 1)) {
      // The selected sector is already loaded
      //PRINTF(("<cache: %d in [%d %d]>", mc->currentSector, gFastIo.first_sector, gFastIo.first_sector + gFastIo.count - 1));
      memcpy(lio_data_buffer, &(gFastIo.buffer[sectorNo - gFastIo.first_sector][0]),
      LIO_SECTOR_SIZE);
      return MemoStatusOK;
  }

  gFastIo.subcommand = KCANY_MEMO_DISK_IO_FASTREAD_LOGICAL_SECTOR;
  gFastIo.first_sector = sectorNo;
  gFastIo.count = 8;

  // Do not try to read outside the disk
  if (gFastIo.first_sector + gFastIo.count > gDiskSize - 1) {
    gFastIo.count = gDiskSize - gFastIo.first_sector;
  }
  ret = os_ioctl(mc->memoHandle, KCANY_IOCTL_MEMO_DISK_IO_FAST, &gFastIo, sizeof(gFastIo));

  if (!ret) {
    gFastIo.first_sector = 0;
    gFastIo.count = 0;
    return MemoStatusFail;
  }

  if (gFastIo.dio_status >= dio_Pending) {
    gFastIo.first_sector = 0;
    gFastIo.count = 0;
    return DioResultToStatus((DioResult)gFastIo.dio_status);
  } else {
    memcpy(lio_data_buffer, &(gFastIo.buffer[0][0]), LIO_SECTOR_SIZE);
  }
  return MemoStatusOK;
} // memoDiskReadLogicalSector


// Functions to read sectors from the disk
KvaMemoStatus getLioVolumeDataBlock(MemoHandle h) {
  return memoReadLioSector(h, 0);
}
KvaMemoStatus getLioVolumeControlBlock0(MemoHandle h) {
  return memoReadLioSector(h, (unsigned int) 1);
}
KvaMemoStatus getLioVolumeControlBlock1(MemoHandle h) {
  return memoReadLioSector(h, (unsigned int) 2);
}
KvaMemoStatus getLioDataBlock(MemoHandle h, unsigned int sector) {
  return memoReadLioSector(h, sector);
}
KvaMemoStatus memoReadLioSector(MemoHandle h, uint32_t sector) {
  KvaMemoStatus stat = MemoStatusFail;
  LioResult lres = {};
  memoConf * mc = (memoConf *)h;
  LioBlock *lb = NULL;


  switch (mc->mDevice) {
    case memo_MEMO5:
      if (usbSpeed != KCAN_USBSPEED_HISPEED) {
        //PRINTF(("<Slow USB>"));
        stat = localDiskReadLogicalSector(h,sector, lio_data_buffer, LIO_SECTOR_SIZE);
      } else {
        //PRINTF(("<Fast USB>"));
        stat = memoDiskReadLogicalSectorFast(h,sector);
      }
      break;

    case memo_DISK5:
        getCurDataBlock(&lb);
        lres = lioReadSector(sector);
        memcpy(lio_data_buffer,lb,LIO_SECTOR_SIZE);
        stat = LioResultToStatus(lres);
      break;
    default:
      PRINTF(("memoReadLioSector: Unknown connection %d\n", mc->mDevice));
  }
  if (stat != MemoStatusOK) {
    if (!mc->trigger) {
          PRINTF(("memoReadLioSector : lres: %d stat %d", lres, stat));
    }
    else if (mc->trigger->lastWrittenSector != 0 && sector >= mc->trigger->lastWrittenSector && stat == MemoStatusERR_CRC_ERROR) {
      // This might be outside the logged data
      stat = MemoStatusEOF;
    } else {
      PRINTF(("memoReadLioSector : lres: %d stat %d", lres, stat));
    }
  }
  return stat;
}

/*
 * =============================================================================
 *   loadLioDataSector
 * =============================================================================
 */
KvaMemoStatus loadLioDataSector(MemoHandle h) {

  memoConf * mc = (memoConf *)h;
  KvaMemoStatus stat = MemoStatusOK;
  LioDataBlock* ldb = NULL;

  // Get the first sector and allocate memory at the first call
  if (mc->currentSector == 0) {
    mc->currentSector = GetFirstDataSector(mc->trigger);
    // Reset the global fast io buffer
    memset(&gFastIo,0, sizeof(gFastIo));
    if (mc->currentSector == 0) {
      PRINTF(("Couldn't get the first data sector for the selected trigger %d.",mc->currentFileIndx));
      return MemoStatusFail;
    }
    mc->ld = (LioBlock*) lio_data_buffer;
  }
  else
  {
    mc->currentSector = GetNextDataSector(mc->trigger, mc->currentSector);
  }
  // PRINTF(("[S:%lu]", mc->currentSector));
  // Read sectors from the disk until a proper data sector is found
  while (stat == MemoStatusOK) {
    uint32_t magicNo = 0;

    // Read the sector from disk and check it
    stat = getLioDataBlock(h, mc->currentSector);
    if (stat != MemoStatusOK ) {
      return stat;
    }

    ldb = (LioDataBlock*) mc->ld;

    // Check the magic number in the sector
    magicNo = *(uint32_t *)mc->ld;
    if (magicNo == PRETRIGGER_MAGIC ||  magicNo == TRIGGER_MAGIC) {
      // Found next trigger or the last written sector, thus end of measurement.
      PRINTF(("[T]"));
      return MemoStatusEOF;
    }
    // Read a wrapped disk using a housekeeping block
    if (mc->currentSector == mc->trigger->lastWrittenSector + 1) {
      if (!(mc->trigger->isHkTrigger == 1)) {
        PRINTF(("[LAST]"));
        return MemoStatusEOF;
      } else {
        mc->trigger->isHkTrigger = 2;
      }
    }


    // Skip housekeeping blocks
    if (magicNo == HOUSE_MAGIC) {
      if (getEndSector(&triggerList, mc->currentFileIndx) == mc->currentSector) {
        // The log ends with a trigger block
        PRINTF(("[HOUSE][EOF]"));
        return MemoStatusEOF;
      }
      PRINTF(("[HOUSE][%u]",mc->currentSector));
      mc->currentSector = GetNextDataSector(mc->trigger, mc->currentSector);
      continue;
    }

    // Check the sector sequence number. It is incremented for each new sector
    // after the trigger but kept constant in the pre-trigger data.
    if (mc->currentSequence == 0) {
      mc->currentSequence = ldb->sectorSeqNum;
    }
    else
    {
      if (isPretriggerData(mc->trigger, mc->currentSector)) {
        // Pre-trigger data
        if (mc->trigger->isHkTrigger) {
          // The housekeeping trigger only pretends to have a pretrigger
          mc->currentSequence++;
        }
      }
      else
      {
        mc->currentSequence++;
      }
    }

    if (mc->currentSequence != ldb->sectorSeqNum) {
      if (mc->currentSector == mc->trigger->lastWrittenSector + 1) {
        // Wrong sector sequence number, stop reading here.
        PRINTF(("[EOF: %u<->%u]",ldb->sectorSeqNum, mc->currentSequence));
        return MemoStatusEOF;
      }
      else {
        PRINTF(("MemoStatusERR_FILE_ERROR [%u<->%u][%u]",ldb->sectorSeqNum, mc->currentSequence, mc->currentSector));
        return MemoStatusERR_FILE_ERROR;
      }
    }
    // All are ok!
    break;
  }
  return MemoStatusOK;
}


/*
 * =============================================================================
 *   memoDiskWritePhysicalSector
 * =============================================================================
 */
KvaMemoStatus WINAPI memoDiskWritePhysicalSector(MemoHandle h, uint32 sectorNo, void* buf, size_t bufsize)
{
  int ret = 0;
  KCANY_MEMO_DISK_IO io;
  memoConf * mc = (memoConf *)h;

  if (bufsize != SD_DISK_SECTOR_SIZE) return MemoStatusERR_PARAM;

  memset(&io, 0, sizeof(io));
  io.subcommand = KCANY_MEMO_DISK_IO_WRITE_PHYSICAL_SECTOR;
  io.first_sector = sectorNo;
  io.count = 1;
  memcpy(&io.buffer, buf, bufsize);

  ret = os_ioctl(mc->memoHandle, KCANY_IOCTL_MEMO_DISK_IO, &io, sizeof(io));

  if (!ret) return MemoStatusERR_DEVICE_COMM_ERROR;

  if (io.dio_status >= dio_Pending)
    return DioResultToStatus((DioResult)io.dio_status);
  return MemoStatusOK;
} // memoDiskWritePhysicalSector


/*
 * =============================================================================
 *   memoDiskReadDBaseSector
 * =============================================================================
 */
static KvaMemoStatus WINAPI memoDiskReadDBaseSector(MemoHandle h, uint32_t sectorNo,
  void* buf, size_t bufsize)
{
  size_t res = 0;
  memoConf * mc = (memoConf *)h;
  KvaMemoStatus status = MemoStatusFail;

  switch (mc->mDevice) {
    case memo_MEMO5:
      status = memoDiskReadPhysicalSector(h, sectorNo, buf, bufsize);
      break;
    case memo_DISK5:
      if (sectorNo != gDBFileIO.currentSector) {
        if(os_fseek(gDBFileIO.fileHandle,sectorNo*SD_DISK_SECTOR_SIZE,SEEK_SET)) {
          PRINTF(("memoDiskReadDBaseSector: Couldn't move file pointer to sector %u.\n", sectorNo));
          return MemoStatusERR_FILE_ERROR;
        }
      }

      res = fread(buf, 1, bufsize, gDBFileIO.fileHandle);
      if (res != bufsize) {
        PRINTF(("memoDiskReadDBaseSector: Read failed %Ilu\n",res));
        return MemoStatusERR_FILE_ERROR;
      }
      if (res == SD_DISK_SECTOR_SIZE) {
        gDBFileIO.currentSector++;
      }
      status = MemoStatusOK;
      break;
    default:
      PRINTF(("memoDiskReadDBaseSector: Unknown connection %d\n", mc->mDevice));
  }

  return status;
}

/*
 * =============================================================================
 *   memoDiskWriteDBaseSector
 * =============================================================================
 */
static KvaMemoStatus WINAPI memoDiskWriteDBaseSector(MemoHandle h, uint32_t sectorNo,
  void* buf, size_t bufsize)
{
  size_t res = 0;
  memoConf * mc = (memoConf *)h;
  KvaMemoStatus status = MemoStatusFail;

  switch (mc->mDevice) {
    case memo_MEMO5:
      status = memoDiskWritePhysicalSector(h, sectorNo, buf, bufsize);
      break;
    case memo_DISK5:
      if (sectorNo != gDBFileIO.currentSector) {
        if(os_fseek(gDBFileIO.fileHandle,sectorNo*SD_DISK_SECTOR_SIZE,SEEK_SET)) {
          PRINTF(("memoDiskWriteDBaseSector: Couldn't move file pointer to sector %u.\n", sectorNo));
          return MemoStatusERR_FILE_ERROR;
        }
      }

      res = fwrite(buf, 1, bufsize, gDBFileIO.fileHandle);
      if (res != bufsize) {
        PRINTF(("memoDiskWriteDBaseSector: Write failed %Ilu.\n",res));
        return MemoStatusERR_FILE_ERROR;
      }
      if (res == SD_DISK_SECTOR_SIZE) {
        gDBFileIO.currentSector++;
      }
      status = MemoStatusOK;
      break;
    default:
      PRINTF(("memoDiskWriteDBaseSector: Unknown connection %d\n", mc->mDevice));
  }

  return status;
}

static void resetgDBFileIO(void) {

    if (gDBFileIO.fileName) {
      // PRINTF(("Warning: Closing file %s.\n", gDBFileIO.fileName));
      free(gDBFileIO.fileName);
    }

    if (gDBFileIO.fileHandle != NULL) {
      // PRINTF(("Warning: Closing database file.\n"));
      fclose(gDBFileIO.fileHandle);
    }
    memset(&gDBFileIO,0, sizeof(gDBFileIO));
  }

/*
 * =============================================================================
 *   memoCloseDBaseFile
 * =============================================================================
 */
static KvaMemoStatus WINAPI memoOpenDBaseFile(MemoHandle h, uint32_t *startSector, uint32_t *endSector) {
  memoConf * mc = (memoConf *)h;

   switch (mc->mDevice) {
    case memo_MEMO5:
    {
      KvaMemoStatus    stat;
      int           res = 0;
      hMemoDataFsInfoB  *mfsb;
      KCANY_MEMO_INFO  info;

      // Get the database area
      memset(&info, 0, sizeof(info));
      info.subcommand = MEMO_SUBCMD_GET_FS_INFO_B;


      res = os_ioctl(mc->memoHandle, KCANY_IOCTL_MEMO_GET_DATA, &info, sizeof(info));
      if (!res) {
        PRINTF(("memoOpenDBaseFile: DEVICE_COMM_ERROR"));
        return MemoStatusERR_DEVICE_COMM_ERROR;
      }

      if ((stat = MemoResultToStatus(&info)) != MemoStatusOK) return stat;

      mfsb = (hMemoDataFsInfoB*) info.buffer;
      *startSector = mfsb->first_dbase_sector;
      *endSector = mfsb->last_dbase_sector;
      if (*startSector == 0 || *endSector == 0) {
        PRINTF(("memoOpenDBaseFile: No database area found.\n"));
        return MemoStatusERR_NOT_FORMATTED;
      }
    }
    break;
    case memo_DISK5:
    {
      uint64_t fileSize = 0;
      std::string drive, dir, path, base, ext;

      if(!mc->kmfFileName.size()) {
        PRINTF(("memoOpenDBaseFile: Not connected to a SD reader."));
        return MemoStatusERR_DEVICE_COMM_ERROR;
      }

      if ( !os_splitpath(mc->kmfFileName, drive, dir, base, ext)) {
        return MemoStatusERR_PARAM;
      }
      os_makepath(path, drive, dir, DATABASE_FILE_NAME, "");

      // Open the file for binary read/write
      resetgDBFileIO();
      gDBFileIO.fileHandle = fopen(path.c_str(), "rb+");
      if (gDBFileIO.fileHandle == NULL) {
          PRINTF(("memoOpenDBaseFile: Could not open file %s.\\n", path.c_str()));
        return MemoStatusERR_FILE_ERROR;
      }

      // Save the filename
      gDBFileIO.fileName = (char*) malloc(path.size()+1);
      gDBFileIO.fileName = strcpy(gDBFileIO.fileName,path.c_str());

      if (!os_get_file_size(gDBFileIO.fileHandle, &fileSize)) {
        PRINTF(("Could not get the size of the file.\n"));
        return MemoStatusERR_FILE_ERROR;
      }

      gDBFileIO.startSector = 0;
      gDBFileIO.endSector = (uint32_t) (fileSize / SD_DISK_SECTOR_SIZE +1);
      gDBFileIO.currentSector = gDBFileIO.startSector;

      *startSector = gDBFileIO.startSector;
      *endSector   = gDBFileIO.endSector;
    }
    break;

    default:
      PRINTF(("memoOpenDBaseFile: Unknown connection %d\n", mc->mDevice));
  }

  return MemoStatusOK;
}
/*
 * =============================================================================
 *   memoCloseDBaseFile
 * =============================================================================
 */
static KvaMemoStatus WINAPI memoCloseDBaseFile(MemoHandle h) {
  memoConf * mc = (memoConf *)h;

  switch (mc->mDevice) {
    case memo_MEMO5:
      // We don't need to do anything here.
      break;
    case memo_DISK5:
      // Clean up the global structure;
      if (gDBFileIO.fileName) {
        PRINTF(("Closing file %s\n", gDBFileIO.fileName));
      }
      resetgDBFileIO();
      break;
    default:
      PRINTF(("memoCloseDBaseFile: Unknown connection %d\n", mc->mDevice));
   }
   return MemoStatusOK;
}


/*
 * =============================================================================
 *   memoDiskWriteLogicalSector
 * =============================================================================
 */
KvaMemoStatus WINAPI memoDiskWriteLogicalSector(MemoHandle h, uint32 sectorNo,
                                                void* buf, size_t bufsize)
{
  int ret = 0;
  KCANY_MEMO_DISK_IO io;
  memoConf * mc = (memoConf *)h;

  if (bufsize != SD_DISK_SECTOR_SIZE) return MemoStatusERR_PARAM;

  memset(&io, 0, sizeof(io));
  io.subcommand = KCANY_MEMO_DISK_IO_WRITE_LOGICAL_SECTOR;
  io.first_sector = sectorNo;
  io.count = 1;
  memcpy(&io.buffer, buf, bufsize);

  ret = os_ioctl(mc->memoHandle, KCANY_IOCTL_MEMO_DISK_IO, &io, sizeof(io));

  if (!ret) return MemoStatusERR_DEVICE_COMM_ERROR;

  if (io.dio_status >= dio_Pending)
    return DioResultToStatus((DioResult)io.dio_status);
  return MemoStatusOK;
} // memoDiskWriteLogicalSector

/*
 * =============================================================================
 *   memoWriteLioSector
 * =============================================================================
 */
KvaMemoStatus memoWriteLioSector(MemoHandle h, uint32_t sector) {
  KvaMemoStatus stat = MemoStatusFail;
  LioResult lres = lio_OK;
  memoConf * mc = (memoConf *)h;
  LioBlock *lb = NULL;

  switch (mc->mDevice) {
    case memo_MEMO5:
      // We only write a few sectors at the moment, so speed is not an issue here yet
      stat = memoDiskWriteLogicalSector(h, (unsigned int) sector, (void*) lio_data_buffer, (size_t) LIO_SECTOR_SIZE);
      // Cache is no longer valid
      gFastIo.count = 0;
      break;
    case memo_DISK5:
        getCurDataBlock(&lb);
        memcpy(lb,lio_data_buffer,LIO_SECTOR_SIZE);
        lres = lioWriteSector(sector);
        stat = LioResultToStatus(lres);
      break;
    default:
      PRINTF(("memoWriteLioSector: Unknown connection %d\n", mc->mDevice));
  }
  if (stat != MemoStatusOK) {
      PRINTF(("memoWriteLioSector : lres: %d stat %d", lres, stat));
  }

  return stat;
}

/*
 * =============================================================================
 *   memoDiskErasePhysicalSector
 * =============================================================================
 */
KvaMemoStatus WINAPI memoDiskErasePhysicalSector(MemoHandle h, uint32 sectorNo,
                                                 uint32 count)
{
  int ret = 0;
  KCANY_MEMO_DISK_IO io;
  memoConf * mc = (memoConf *)h;

  memset(&io, 0, sizeof(io));
  io.subcommand = KCANY_MEMO_DISK_IO_ERASE_PHYSICAL_SECTOR;
  io.first_sector = sectorNo;
  io.count = count;

  ret = os_ioctl(mc->memoHandle, KCANY_IOCTL_MEMO_DISK_IO, &io, sizeof(io));

  if (!ret) return MemoStatusERR_DEVICE_COMM_ERROR;

  if (io.dio_status >= dio_Pending)
    return DioResultToStatus((DioResult)io.dio_status);
  return MemoStatusOK;
} // memoDiskErasePhysicalSector

/*
 * =============================================================================
 *   memoDiskEraseLogicalSector
 * =============================================================================
 */
KvaMemoStatus WINAPI memoDiskEraseLogicalSector(MemoHandle h, uint32 sectorNo,
                                                uint32 count)
{
  int ret = 0;
  KCANY_MEMO_DISK_IO io;
  memoConf * mc = (memoConf *)h;

  memset(&io, 0, sizeof(io));
  io.subcommand = KCANY_MEMO_DISK_IO_ERASE_LOGICAL_SECTOR;
  io.first_sector = sectorNo;
  io.count = count;

  ret = os_ioctl(mc->memoHandle, KCANY_IOCTL_MEMO_DISK_IO, &io, sizeof(io));

  if (!ret) return MemoStatusERR_DEVICE_COMM_ERROR;

  if (io.dio_status >= dio_Pending)
    return DioResultToStatus((DioResult)io.dio_status);
  return MemoStatusOK;
} // memoDiskEraseLogicalSector

/*
 * =============================================================================
 *   memoSetCallback
 * =============================================================================
 */
KvaMemoStatus WINAPI memoSetCallback(MemoHandle /* h */,  memolib_callback_type /* f */)
{
  // Obsolete; use memoSetCallbackEx instead.
  return MemoStatusOK;
}

KvaMemoStatus WINAPI memoSetCallbackEx(MemoHandle /* h */, MemoCallback f)
{
  guiCallback = f;
  return MemoStatusOK;
} // memoSetCallbackEx

/*
 * =============================================================================
 *   memoGetSerialNumber
 * =============================================================================
 */
KvaMemoStatus WINAPI memoGetSerialNumber(MemoHandle h, unsigned int *serial)
{

  int ret = 0;
  memoConf * mc = (memoConf *)h;
  VCAN_IOCTL_CARD_INFO ioctl_card_info;

  ret = os_ioctl(mc->memoHandle, VCAN_IOCTL_GET_CARD_INFO, &ioctl_card_info, sizeof(ioctl_card_info));

  if (!ret) return MemoStatusERR_DEVICE_COMM_ERROR;

  if (serial) *serial = ioctl_card_info.serial_number;

  return MemoStatusOK;
}



/*
 * =============================================================================
 *   memoGetSoftwareVersionInfo
 * =============================================================================
 */
KvaMemoStatus WINAPI memoGetSoftwareVersionInfo(MemoHandle h,
                                                memoVersionInfo itemCode,
                                                unsigned int *major,
                                                unsigned int *minor,
                                                unsigned int *build,
                                                unsigned int *flags)
{
  int ret = 0;

  memoConf * mc = (memoConf *)h;
  VCAN_IOCTL_CARD_INFO ioctl_card_info;

  ret = os_ioctl(mc->memoHandle, VCAN_IOCTL_GET_CARD_INFO, &ioctl_card_info, sizeof(ioctl_card_info));

  switch (itemCode) {
    case memo_SWINFO_KVAMEMOLIB:
      if (major) *major = CANLIB_MAJOR_VERSION;
      if (minor) *minor = CANLIB_MINOR_VERSION;
      if (build) *build = 0;
#ifdef DEBUG
      if (flags) *flags = MEMOVERSION_FLAG_BETA;
#else
      if (flags) *flags = 0;
#endif
      break;

    case memo_SWINFO_DRIVER:
      if (!ret) return MemoStatusERR_DEVICE_COMM_ERROR;
      if (major) *major = ioctl_card_info.driver_version_major;
      if (minor) *minor = ioctl_card_info.driver_version_minor;
      if (build) *build = ioctl_card_info.driver_version_build;
      if (flags) *flags = 0;
      break;

    case memo_SWINFO_FIRMWARE:
      if (!ret) return MemoStatusERR_DEVICE_COMM_ERROR;
      if (major) *major = ioctl_card_info.firmware_version_major;
      if (minor) *minor = ioctl_card_info.firmware_version_minor;
      if (build) *build = ioctl_card_info.firmware_version_build;
      if (flags) {
        *flags = 0;
        if (ioctl_card_info.firmware_version_build == 0) *flags = MEMOVERSION_FLAG_BETA;
      }
      break;

    case memo_SWINFO_DRIVER_PRODUCT:
      if (!ret) return MemoStatusERR_DEVICE_COMM_ERROR;
      if (major) *major = ioctl_card_info.product_version_major;
      if (minor) *minor = ioctl_card_info.product_version_minor;
      if (build) *build = 0;
      if (flags) {
        *flags = 0;
        if (ioctl_card_info.product_version_major == 0 && ioctl_card_info.product_version_minor == 0)
          *flags = MEMOVERSION_FLAG_BETA;
      }
      break;

    case memo_SWINFO_CONFIG_VERSION_NEEDED:
    {
      // Return the version of param.lif that the connected device expects
      // NOTE: This is FAKE DATA - there is no way to get this from firmware.
      uint8_t lioMajor, lioMinor;
      uint8_t configMajor, configMinor;

      if (!ret) return MemoStatusERR_DEVICE_COMM_ERROR;
      fwVerToConfigVer(ioctl_card_info.firmware_version_major,
                       ioctl_card_info.firmware_version_minor,
                       &lioMajor, &lioMinor,
                       &configMajor, &configMinor);
      if (major) *major = configMajor;
      if (minor) *minor = configMinor;
      if (build) *build = 0;
      if (flags) *flags = 0;
      break;
    }
    case memo_SWINFO_CPLD_VERSION:
    {
      KvaMemoStatus stat;
      int r = 0;
      hMemoDataMiscInfo *miff;
      KCANY_MEMO_INFO info;

      memset(&info, 0, sizeof(info));
      info.subcommand = MEMO_SUBCMD_GET_MISC_INFO;
      memoConf *mc = (memoConf *)h;

      r = os_ioctl(mc->memoHandle, KCANY_IOCTL_MEMO_GET_DATA, &info, sizeof(info));

      if (!r) return MemoStatusERR_DEVICE_COMM_ERROR;

      if ((stat = MemoResultToStatus(&info)) != MemoStatusOK) return stat;

      miff = (hMemoDataMiscInfo*) info.buffer;
      if (major) *major = miff->cpldVersion;
      if (minor) *minor = 0;
      if (build) *build = 0;
      if (flags) *flags = 0;
      break;
    }

    default:
      return MemoStatusERR_PARAM;
  }
  return MemoStatusOK;

} // memoGetSoftwareVersionInfo




/*
 * =============================================================================
 *   translateLogEvent - Translates a LogFileEvent read directly from a
 *   Memorator to a memoLogEvent that is used in memoLib's API.
 * =============================================================================
 */
static bool translateLogEvent(memoConfBase &h,
                              hydraHostCmdLogFD *e,
                              memoLogEventEx *me)
{
  bool status = false;
  memoConf * mc = (memoConf *)&h;
  switch (e->logHead.cmdNo) {

    case CMD_LOG_RTC_TIME_FD:
    {
      me->type = MEMOLOG_TYPE_CLOCK;
      me->x.rtc.calendarTime = e->logRtcTimeFD.unixTime;
      me->x.rtc.timeStamp = (int64)mc->currentTime64();
      status = true;
      break;
    }

    case CMD_RX_MESSAGE_FD:
    {
      int dataLen = 0;
      me->type = MEMOLOG_TYPE_MSG;

      me->x.msg.id          = e->logMessageFD.id & 0x1FFFFFFF;
      me->x.msg.timeStamp   = (int64)mc->currentTime64();
      me->x.msg.channel     = e->logMessageFD.channel;
      me->x.msg.flags       = 0;

      me->x.msg.flags |= (e->logMessageFD.id & 0x80000000)? canMSG_EXT : canMSG_STD;

      if (e->logMessageFD.flags & MSGFLAG_OVERRUN)
        me->x.msg.flags |= canMSGERR_OVERRUN;

      if (e->logMessageFD.flags & MSGFLAG_NERR)
        me->x.msg.flags |= canMSG_NERR;

      if (e->logMessageFD.flags & MSGFLAG_WAKEUP)
        me->x.msg.flags |= canMSG_WAKEUP;

      if (e->logMessageFD.flags & MSGFLAG_ERROR_FRAME)
        me->x.msg.flags |= canMSG_ERROR_FRAME;

      if (e->logMessageFD.flags & MSGFLAG_TX)
        me->x.msg.flags |= canMSG_TXACK;

      if (e->logMessageFD.flags & MSGFLAG_BRS)
        me->x.msg.flags |= canFDMSG_BRS;

      if (e->logMessageFD.flags & MSGFLAG_ESI)
        me->x.msg.flags |= canFDMSG_ESI;

      if (e->logMessageFD.flags & MSGFLAG_FDF) {
        me->x.msg.flags |= canFDMSG_FDF;
        me->x.msg.dlc = e->logMessageFD.dlc;
        dataLen = dlcToBytesFD(e->logMessageFD.dlc);
      }
      else {
        me->x.msg.dlc = e->logMessageFD.dlc & 0xf;
        dataLen = MIN(e->logMessageFD.dlc, MAX_LOGGED_CAN_DATABYTES);
      }

      if (e->logMessageFD.flags & MSGFLAG_REMOTE_FRAME) {
        me->x.msg.flags |= canMSG_RTR;
        // Remote frames have dlc, but don't contain any data
        dataLen = 0;
        memset(me->x.msg.data, 0, sizeof(me->x.msg.data));
      }

      memcpy(me->x.msg.data, e->logMessageFD.data, dataLen);
      status = true;
      break;
    }

    case CMD_LOG_TRIG_FD:
    {
      me->type                = MEMOLOG_TYPE_TRIGGER;
      me->x.trig.type         = e->logTrigFD.type;
      me->x.trig.preTrigger   = e->logTrigFD.preTrigger;
      me->x.trig.postTrigger  = e->logTrigFD.postTrigger;
      me->x.trig.trigNo       = e->logTrigFD.trigNo;
      me->x.trig.timeStamp    = (int64)mc->currentTime64();
      status = true;
      break;
    }

    case CMD_LOG_VERSION_FD:
    {
      me->type               = MEMOLOG_TYPE_VERSION;
      me->x.ver.lioMajor     = e->logVerFD.lioMajor;
      me->x.ver.lioMinor     = e->logVerFD.lioMinor;
      me->x.ver.fwMajor      = e->logVerFD.fwMajor;
      me->x.ver.fwMinor      = e->logVerFD.fwMinor;
      me->x.ver.fwBuild      = e->logVerFD.fwBuild;
      me->x.ver.serialNumber = e->logVerFD.serialNumber;
      me->x.ver.eanHi        = e->logVerFD.eanHi;
      me->x.ver.eanLo        = e->logVerFD.eanLo;
      PRINTF(("LIO: %u.%u FW: %u.%u.%u EAN:%u %u S:%u",
      me->x.ver.lioMajor, me->x.ver.lioMinor, me->x.ver.fwMajor,
      me->x.ver.fwMinor, me->x.ver.fwBuild,  me->x.ver.eanHi,
      me->x.ver.eanLo, me->x.ver.serialNumber));
      status = true;
      break;
    }

    default:
      PRINTF(("translateLogEvent: Unknown event e->logHead.cmdNo %d ",
        e->logHead.cmdNo));
      break;
  }
  return status;
}


/*
 * =============================================================================
 *   memoLogOpenFile
 * =============================================================================
 */
KvaMemoStatus WINAPI memoLogOpenFile(MemoHandle h,
                                     uint32 fileIndx,
                                     uint64 *eventCount)
{
  memoConf * mc = (memoConf *)h;
  KvaMemoStatus stat;

  mc->resetCurrentTime();
  msgBufReset();

  stat = openLogFile(h, fileIndx, eventCount);

  if (stat != MemoStatusOK) {
    return stat;
  }

  mc->currentFileIndx = fileIndx;
  return MemoStatusOK;

} // memoLogFileOpen


/*
 * =============================================================================
 *   memoLogGetStartTime
 *   Returns the calendar start time of the open log file. This can be used
 *   as a sort of creation date for the file.
 * =============================================================================
 */
KvaMemoStatus WINAPI memoLogGetStartTime(MemoHandle h, uint32 *start_time)
{
  memoConf * mc = (memoConf *)h;

  if (!start_time) return MemoStatusERR_PARAM;
  if (mc->startTime != 0) {
    *start_time = (uint32) mc->startTime;
    return MemoStatusOK;
  } else {
    return MemoStatusFail;
  }
} // memoLogFileGetStartTime

/*
 * =============================================================================
 *   memoLogGetEndTime
 *   Returns the calendar end time of the open log file. This can be used
 *   together with start time to display the time interval coverd by the log.
 * =============================================================================
 */
KvaMemoStatus WINAPI memoLogGetEndTime(MemoHandle h, uint32 *end_time)
{
  memoConf * mc = (memoConf *)h;

  if (!end_time) return MemoStatusERR_PARAM;
  if (mc->endTime != 0) {
    *end_time = (uint32) mc->endTime;
    return MemoStatusOK;
  } else {
    return MemoStatusFail;
  }
} // memoLogFileGetEndTime

/*
 * =============================================================================
 *   memoLogGetSerial
 *   Returns the serial number of the open log file. This can be used
 *   with SD cards to find out which device has created the log
 * =============================================================================
 */
KvaMemoStatus WINAPI memoLogGetSerial(MemoHandle h, uint32 *serialNumber)
{
  memoConf * mc = (memoConf *)h;

  if (!serialNumber) return MemoStatusERR_PARAM;
  if (!mc->trigger) return MemoStatusERR_LOGFILEOPEN;

  if (mc->trigger->serialNumber != 0) {
    *serialNumber =  mc->trigger->serialNumber;
    return MemoStatusOK;
  } else {
    return MemoStatusFail;
  }
} // memoLogGetSerial

/*
 * =============================================================================
 *   memoLogGetEan
 *   Returns the EAN number of the open log file. This can be used
 *   with SD cards to find out what type of device has created the log
 * =============================================================================
 */
KvaMemoStatus WINAPI memoLogGetEan(MemoHandle h, uint32 *eanHi, uint32 *eanLo)
{
  memoConf * mc = (memoConf *)h;

  if (!eanHi || !eanLo) return MemoStatusERR_PARAM;
  if (!mc->trigger) return MemoStatusERR_LOGFILEOPEN;

  if (mc->trigger->serialNumber != 0) {
    *eanHi =  mc->trigger->eanHi;
    *eanLo =  mc->trigger->eanLo;
    return MemoStatusOK;
  } else {
    return MemoStatusFail;
  }
} // memoLogGetEan

/*
 * =============================================================================
 *   Helper functions to sort the messages from multiple channels
 * =============================================================================
 */

// =============================================================================
KvaMemoStatus msgBufReset (void)
{
  KvaMemoStatus status = MemoStatusOK;
  int ch;
  for (ch = 0; ch < MAX_NO_CHANNELS + 1; ch++) {
    wp[ch] = 0;
    rp[ch] = 0;
  }
  endOfLogFile = 0;
  return status;
}

// =============================================================================
KvaMemoStatus msgBufCheckChannel(int ch)
{
  KvaMemoStatus status = MemoStatusOK;
  if (ch < 0 || ch >= MAX_NO_CHANNELS + 1) {
    PRINTF(("msgBufCheckChannel: Channel %d is not implemented.\n", ch));
    status = MemoStatusFail;
  }
  return status;
}

// =============================================================================
int msgBufferAreFull (void)
{
  int ch           = 0;
  int full         = 0;

  for (ch = 0; ch < MAX_NO_CHANNELS +1; ch++) {
    unsigned int nwp = 0;
    INC_MSG_BUFFER_PTR(wp[ch], nwp);
    if (rp[ch] == nwp) {
      full = 1;
      break;
    }
  }
  return full;
}

 // =============================================================================
int msgBufIsNotEmpty (int ch)
{
  if (msgBufCheckChannel(ch)) return 0;
  return (wp[ch] != rp[ch]);
}

// =============================================================================
void msgBufPut (hydraHostCmdLogFD *e, int ch)
{
  if (!e) return;
  if (msgBufCheckChannel(ch)) return;
  memcpy(msgbuffers[ch]+wp[ch], e, sizeof(hydraHostCmdLogFD));
  INC_MSG_BUFFER_PTR(wp[ch], wp[ch]);
}

// =============================================================================
void msgBufGet (hydraHostCmdLogFD **e, int ch)
{
  if (msgBufCheckChannel(ch)) return;

  *e = msgbuffers[ch] + rp[ch];
  INC_MSG_BUFFER_PTR(rp[ch], rp[ch]);
}

// =============================================================================
void msgBufInsert (hydraHostCmdLogFD *e)
{
  int channel = GLOBAL_CHAN_IDX;

  if (e->logHead.cmdNo == CMD_RX_MESSAGE_FD) {
    channel = e->logMessageFD.channel;
  }

  if (e->logHead.cmdNo == CMD_LOG_TRIG_FD) {
    if (GLOBAL_CHAN != e->logTrigFD.channel) {
      channel = e->logTrigFD.channel;
    }
  }
  //PRINTF(("Insert %d in channel %d", e->logHead.cmdNo, channel));
  msgBufPut(e, channel);
}

// =============================================================================
uint64_t msgBufGetTime (int ch)
{
  uint64_t time;
  hydraHostCmdLogFD *tmp = NULL;
  time = 0;

  if (msgBufCheckChannel(ch)) return time;

  tmp = msgbuffers[ch] + rp[ch];
  if (tmp->logHead.cmdNo == CMD_RX_MESSAGE_FD) {
    memcpy(&time, tmp->logMessageFD.time, sizeof(time));
  } else if (tmp->logHead.cmdNo == CMD_LOG_TRIG_FD) {
    memcpy(&time, tmp->logTrigFD.time, sizeof(time));
  } else if (tmp->logHead.cmdNo == CMD_LOG_RTC_TIME_FD) {
    memcpy(&time, tmp->logRtcTimeFD.time, sizeof(time));
  }
  return time;
}


// =============================================================================
KvaMemoStatus msgBufGetNext (hydraHostCmdLogFD **e)
{
  uint64_t min_time = 0;
  uint64_t time;
  int k, min_k = -1;
  // Compare the time-stamp for messages from all channels and make
  // sure that global event comes first

  // Check for global events first
  if (msgBufIsNotEmpty(GLOBAL_CHAN_IDX)) {
      min_time = msgBufGetTime(GLOBAL_CHAN_IDX);
      min_k    = GLOBAL_CHAN_IDX;
  }

  for (k=0; k < MAX_NO_CHANNELS; k++) {
    if (msgBufIsNotEmpty(k)) {
      time = msgBufGetTime(k);
      if (min_k < 0 || time < min_time) {
        min_time = time;
        min_k    = k;
      }
    }
  }

  if (min_k >= 0) {
    msgBufGet(e, min_k);
    return MemoStatusOK;
  }
  else {
    return MemoStatusEOF;
  }
}

/*
 * =============================================================================
 *   memoLogReadEventEx
 *   Reads one event from a log file.
 * =============================================================================
 */
KvaMemoStatus WINAPI memoLogReadEventEx(MemoHandle h, memoLogEventEx *e)
{
  memoConf * mc = (memoConf *)h;
  bool gotOne = false;
  KvaMemoStatus stat = MemoStatusOK;
  hydraHostCmdLogFD *f = NULL;

  while ((stat == MemoStatusOK) && (!endOfLogFile)) {
    if (msgBufferAreFull()) {
      break;
    }
    stat = readLogFile(h, &f);
    if (stat == MemoStatusOK) {
      msgBufInsert(f);
    }
    else if (stat == MemoStatusEOF) {
      endOfLogFile = 1;
    }
    else {
      return stat;
    }
  }

  while (!gotOne) {
    stat = msgBufGetNext(&f);
    if (stat == MemoStatusEOF) {
      PRINTF(("MemoStatusERR_NOLOGMSG"));
      return MemoStatusERR_NOLOGMSG;
    }
    updateTimeStamp(*mc, f);
    gotOne = translateLogEvent(*mc, f, e);
  }

  return stat;

} // memoLogReadEventEx


/*
 * =============================================================================
 *   memoLogCloseFile
 * =============================================================================
 */
void WINAPI memoLogCloseFile (MemoHandle h)
{
  memoConf * mc = (memoConf *)h;

  mc->resetCurrentTime();
  mc->currentFileIndx = -1;
  mc->startTime = 0;
  if (mc->trigger) {
    mc->trigger->serialNumber = 0;
  }
  msgBufReset();
  closeLogFile (h);
} // MemoLogFileClose


/*
 * =============================================================================
 *   writeCleanTrack
 * =============================================================================
 */
static KvaMemoStatus writeCleanTrack(MemoHandle h, TriggerList *list, int track, uint8_t lioMajor, uint8_t lioMinor) {
  KvaMemoStatus          status;
  LioVolumeDataBlock*    lvdb = NULL;
  LioVolumeControlBlock* lvcb = NULL;



  PRINTF(("writeCleanTrack: %u.%u", lioMajor, lioMinor));
  // LioVolumeDataBlock
  lvdb = (LioVolumeDataBlock*) lio_data_buffer;
  memset(lvdb, 0, sizeof(lio_data_buffer));
  lvdb->version       = toVersion(lioMajor, lioMinor);
  lvdb->magicNumber   = CONFIG_MAGIC;
  lvdb->fileSize      = list->fileSize;
  lvdb->numOfKMFFiles = list->numOfKMFFiles;
  if (LIO_MULTITRACK_VERSION == lioMajor) {
    lvdb->trackVersion = LIO_MULTITRACK_VERSION;
    uint32_t *buf = (uint32_t *) lvdb->trackInfo;
    for (int n=0; n < list->tracks; n++) {
      memcpy(buf + 2*n,       &list->trackStart[n], sizeof(uint32_t));
      memcpy(buf + (2*n + 1), &list->trackEnd[n],   sizeof(uint32_t));
      PRINTF(("%d [%u %u]", n, list->trackStart[n], list->trackEnd[n]));
    }
    lvdb->tracks = list->tracks;
  }

  // Do a sanity check before writing anything to the disk
  if (lvdb->fileSize == 0 || lvdb->numOfKMFFiles == 0) {
    PRINTF(("Corrupt LioDataBlock information. Aborting.\n"));
    return MemoStatusERR_FILE_SYSTEM_CORRUPT;
  }

  status = memoWriteLioSector(h,list->trackStart[track]);
  if (MemoStatusOK != status) {
    PRINTF(("memoWriteLioSector failed."));
    return status;
  }

  // LioVolumeControlBlocks
  lvcb = (LioVolumeControlBlock*) lio_data_buffer;
  memset(lvcb, 0, sizeof(lio_data_buffer));
  lvcb->lastKnownWPSecNum         = list->trackStart[track];
  lvcb->sectorHighWaterMarkSecNum = list->trackStart[track] + LIO_FIRST_AVAILABLE_SECTOR;
  status = memoWriteLioSector(h,list->trackStart[track]+1);
  if (MemoStatusOK != status) return status;
  status = memoWriteLioSector(h,list->trackStart[track]+2);
  if (MemoStatusOK != status) return status;

  return MemoStatusOK;
}

/*
 * =============================================================================
 *   createTracks
 * =============================================================================
 */
static KvaMemoStatus createTracks(TriggerList *list, int tracks) {

  PRINTF(("createTracks"));

  if (!list)  return MemoStatusERR_PARAM;

  if (list->trackStart) free(list->trackStart);
  if (list->trackEnd)   free(list->trackEnd);

  list->trackStart = (uint32_t*) malloc(tracks * sizeof(uint32_t));
  list->trackEnd   = (uint32_t*) malloc(tracks * sizeof(uint32_t));
  if (!list->trackStart || !list->trackEnd) {
    PRINTF(("Out of memory."));
    return MemoStatusERR_FATAL_ERROR;
  }

  // Calculate tracks
  list->trackStart[0] = 0;
  list->trackEnd[0] = list->fileSize / tracks;
  for (int m=1; m < tracks - 1; m++) {
    list->trackStart[m] = list->trackEnd[m-1] + 1;
    list->trackEnd[m]   = list->trackStart[m] + list->fileSize / tracks;
  }
  list->trackEnd[tracks-1]   = list->fileSize;
  if (tracks > 1) list->trackStart[tracks-1] = list->trackEnd[tracks-2]+1;
  list->tracks = tracks;

  for (int k=0;k < list->tracks;k++) {
    PRINTF(("%d [%u %u]", k, list->trackStart[k], list->trackEnd[k]));
  }

  return MemoStatusOK;
}
/*
 * =============================================================================
 *   memoPartitionDisk
 * =============================================================================
 */
KvaMemoStatus WINAPI memoPartitionDisk(MemoHandle h, char * /* path */, int tracks)
{
  KvaMemoStatus status = MemoStatusOK;
  uint8_t lioMajor = LIO_VERSION_FD;
  uint8_t lioMinor = 0;
  memoConf *mc = NULL;

  PRINTF(("memoPartitionDisk"));
  if (!h)  return MemoStatusERR_PARAM;
  if (tracks < 1 || tracks > 50) return MemoStatusERR_PARAM;

  if (tracks > 1) {
    lioMajor = LIO_MULTITRACK_VERSION;
  }

   mc = (memoConf *)h;

  if (gDiskSize == 0) {
    fileSysUsage fus;
    status = memoGetFileSystemUsage(h, &fus);
  }

  /*  if (0 != triggerList.tracks) {
    status = getTracksFromLioDataBlock(h, &triggerList);
    if (status != MemoStatusOK ) return status;
  }
  // Erase the old system blocks to avoid future confusion
  for (int k=0; k < triggerList.tracks; k++) {
    memset(&lio_data_buffer, 0, sizeof(lio_data_buffer));
    for (int n=0;n < 5; n++) {
      memoWriteLioSector(h,triggerList.trackStart[k]+n);
      if (status != MemoStatusOK ) return status;
    }
  }
*/
  // Create new tracks
  status =  createTracks(&triggerList, tracks);

  // Change to read/write if we are using a file/disk
  if (mc->mDevice == memo_DISK5) {
    status = reopenDiskReadWrite();
    if (status != MemoStatusOK) {
      (void) reopenDiskReadOnly();
      PRINTF(("memoFormatDiskEx: File is write-protected.\n"));
      return MemoStatusERR_WRITE_PROT;
    }
  }

  // Write clean tracks to the disk
  for (int m = 0; m < tracks; m++) {
    status = writeCleanTrack(h, &triggerList, m, lioMajor, lioMinor);
    if (status != MemoStatusOK ) return status;
  }

  if (mc->mDevice == memo_DISK5) {
    status = reopenDiskReadOnly();
    if (status != MemoStatusOK) return status;
  }

  // Delete triggers, update track list and verify
  DeleteTriggerList(&triggerList);
  status = getTracksFromLioDataBlock(h, &triggerList);

  return status;
}


/*
 * =============================================================================
 *   memoGetTrackCount
 * =============================================================================
 */
KvaMemoStatus WINAPI  memoGetTrackCount(MemoHandle h, int *tracks)
{
  KvaMemoStatus status = MemoStatusOK;
  PRINTF(("memoGetTrackCount"));
  status = getTracksFromLioDataBlock(h, &triggerList);
  *tracks = triggerList.tracks;
  return status;
}

/*
 * =============================================================================
 *   memoGetFileTrackUsage
 * =============================================================================
 */
KvaMemoStatus WINAPI memoGetFileTrackUsage(MemoHandle h, int track, fileSysUsage *fus)
{
  KvaMemoStatus stat          = MemoStatusOK;
  LioVolumeControlBlock *lvcb = NULL;
  LioVolumeDataBlock    *lvdb = NULL;

  if (!h) return MemoStatusERR_PARAM;
  if (!fus) return MemoStatusERR_PARAM;
  memset(fus, 0, sizeof(*fus));
  memset(&gFastIo, 0, sizeof(gFastIo));

  // Read the first LioVolumeDataBlock
  stat = getTracksFromLioDataBlock(h, &triggerList);
  if (stat != MemoStatusOK) {
    PRINTF(("getTracksFromLioDataBlock failed!"));
    return stat;
  }

  if ( track < 0 || triggerList.tracks - 1 < track) {
    PRINTF(("Track failure: %d %d",triggerList.tracks, track));
    return MemoStatusERR_PARAM;
  }

  lvdb = (LioVolumeDataBlock*) lio_data_buffer;
  fus->maxFileCount = 10000;
  fus->fileCount    = lvdb->numOfKMFFiles;
  fus->diskSize     = triggerList.trackEnd[track] - triggerList.trackStart[track] - LIO_FIRST_AVAILABLE_SECTOR;
  gDiskSize         = lvdb->fileSize;

  // Load the LioVolumeControlBlock for this partition
  stat = getLioDataBlock(h, triggerList.trackStart[track]+1);
  if (stat != MemoStatusOK) {
    PRINTF(("getLioDataBlock(%u)", triggerList.trackStart[track]+1));
    return stat;
  }

  lvcb = (LioVolumeControlBlock*) lio_data_buffer;
  if (lvcb->highWaterHasWrapped) {
    fus->usedDiskSize = fus->diskSize;
  } else {
    if (lvcb->sectorHighWaterMarkSecNum > triggerList.trackStart[track]+LIO_FIRST_AVAILABLE_SECTOR) {
      fus->usedDiskSize = lvcb->sectorHighWaterMarkSecNum - (triggerList.trackStart[track]+LIO_FIRST_AVAILABLE_SECTOR) + 1;
    } else {
      fus->usedDiskSize = 0;
    }
  }
  PRINTF(("Done."));
  return MemoStatusOK;
}

/*
 * =============================================================================
 *   memoClearDataTrack
 * =============================================================================
 */
KvaMemoStatus WINAPI memoClearDataTrack(MemoHandle h, int track)
{
  KvaMemoStatus status = MemoStatusOK;
  LioVolumeDataBlock    *lvdb = NULL;
  uint8_t major, minor;
  uint8_t lioMajor = LIO_VERSION_FD;
  uint8_t lioMinor = 0;
  memoConf *mc = (memoConf *)h;

  PRINTF(("memoClearDataTrack %d\n", track));
  if (!h) return MemoStatusERR_PARAM;

  status = getTracksFromLioDataBlock(h, &triggerList);
  if (status != MemoStatusOK ) return status;

  if ( track < 0 || triggerList.tracks - 1 < track) return MemoStatusERR_PARAM;

  // Get version
  status = getLioVolumeDataBlock(h);
  if (status != MemoStatusOK) return status;

  lvdb = (LioVolumeDataBlock*) lio_data_buffer;
  if (!lvdb) return MemoStatusERR_FILE_ERROR;

  splitVersion(lvdb->version, &major, &minor);

  if (LIO_MULTITRACK_VERSION == major) lioMajor = LIO_MULTITRACK_VERSION;

  // Change to read/write if we are using a file/disk
  if (mc->mDevice == memo_DISK5) {
    status = reopenDiskReadWrite();
    if (status != MemoStatusOK) {
      (void) reopenDiskReadOnly();
      PRINTF(("memoFormatDiskEx: File is write-protected.\n"));
      return MemoStatusERR_WRITE_PROT;
    }
  }

  // Write clean tracks to the disk
  status = writeCleanTrack(h, &triggerList, track, lioMajor, lioMinor);
  if (status != MemoStatusOK ) {
    if (mc->mDevice == memo_DISK5) reopenDiskReadOnly();
    return status;
  }

  if (mc->mDevice == memo_DISK5) {
    status = reopenDiskReadOnly();
    if (status != MemoStatusOK) return status;
  }

  return MemoStatusOK;
}

/*
 * =============================================================================
 *   memoGetLogFileCountTrack
 * =============================================================================
 */
KvaMemoStatus WINAPI memoGetLogFileCountTrack(MemoHandle h, int track, uint32* fileCount)
{
  KvaMemoStatus status = MemoStatusOK;
  PRINTF(("memoGetLogFileCountTrack"));
  if (!fileCount) return MemoStatusERR_PARAM;
  if (!h)         return MemoStatusERR_PARAM;

  if (0 == triggerList.tracks || 0 == triggerList.count) {
    // Create the trigger list
    status = memoGetLogFileCountEx(h, fileCount);
    if (status != MemoStatusOK ) return status;
  }

  if ( track < 0 || triggerList.tracks - 1 < track) return MemoStatusERR_PARAM;

  *fileCount = countTriggersOnTrack(&triggerList, track);
  PRINTF(("fileCount: %d",*fileCount));
  return status;
}

/*
 * =============================================================================
 *   verifyTrackLayout
 * =============================================================================
 */
static KvaMemoStatus verifyTrackLayout(TriggerList *list, uint32_t endSector)
{
  PRINTF(("verifyTrackLayout: Check"));
  if (!list) return MemoStatusERR_PARAM;
  if (!list->trackStart) return MemoStatusERR_PARAM;
  if (!list->trackEnd)   return MemoStatusERR_PARAM;

  PRINTF(("tracks: %d trackStart: %u trackEnd: %u",
   list->tracks, list->trackStart[0], list->trackEnd[0]));

  // Every disk contains at least one track
  if (0 == list->tracks) return MemoStatusERR_FILE_SYSTEM_CORRUPT;

  // Every disk must contain at least 10 sectors 8 system: + 2 data
  for(int k=0; k < list->tracks;k++) {
    if (list->trackEnd[k] <= list->trackStart[k]) return MemoStatusERR_FILE_SYSTEM_CORRUPT;
    if (list->trackEnd[k] - list->trackStart[k] <= 10) return MemoStatusERR_FILE_SYSTEM_CORRUPT;
  }

  // Partiontions must not overlap
  if (list->tracks > 1) {
    for(int k=0; k < list->tracks - 1;k++) {
      if (list->trackEnd[k] >= list->trackStart[k+1]) return MemoStatusERR_FILE_SYSTEM_CORRUPT;
    }
  }

  if (list->trackEnd[list->tracks-1] > endSector) return MemoStatusERR_FILE_SYSTEM_CORRUPT;
  PRINTF(("verifyTrackLayout: OK"));
  return MemoStatusOK;
}

/*
 * =============================================================================
 *   memoGetLogFileCountTrack
 * =============================================================================
 */
static KvaMemoStatus getTracksFromLioDataBlock(MemoHandle h, TriggerList *list)
{
  KvaMemoStatus status = MemoStatusOK;
  LioVolumeDataBlock    *lvdb = NULL;
  unsigned int size =   sizeof(uint32_t);
  uint8_t major, minor;
  PRINTF(("getTracksFromLioDataBlock"));

  if (!list) return MemoStatusERR_PARAM;
  if (!h)    return MemoStatusERR_PARAM;

  // Read the sysinfo sectors from the Memorator
  status = getLioVolumeDataBlock(h);
  if (status != MemoStatusOK) {
    return status;
  }

  if (list->trackStart) free(list->trackStart);
  if (list->trackEnd)   free(list->trackEnd);

  lvdb = (LioVolumeDataBlock*) lio_data_buffer;
  if (!lvdb) return MemoStatusERR_FILE_ERROR;
  splitVersion(lvdb->version, &major, &minor);
  if (LIO_MULTITRACK_VERSION == major) {
    list->tracks = lvdb->tracks;
  } else {
    list->tracks = 1;
  }

  list->trackStart = (uint32_t*) malloc(list->tracks * sizeof(uint32_t));
  list->trackEnd   = (uint32_t*) malloc(list->tracks * sizeof(uint32_t));
  if (!list->trackStart || !list->trackEnd) {
    PRINTF(("Out of memory."));
    return MemoStatusERR_FATAL_ERROR;
  }

  if (LIO_MULTITRACK_VERSION == major) {
    for(int k = 0; k < list->tracks; k++) {
      memcpy(&list->trackStart[k], lvdb->trackInfo + 2*k*size, size);
      memcpy(&list->trackEnd[k],   lvdb->trackInfo + (2*k + 1)*size, size);
      PRINTF(("Track: %d(%d) [%u %u]\n",k,list->tracks,list->trackStart[k],list->trackEnd[k]));
    }
  } else {
    list->trackStart[0] = 0;
    list->trackEnd[0]   = lvdb->fileSize;
  }

  list->numOfKMFFiles = lvdb->numOfKMFFiles;
  list->fileSize      = lvdb->fileSize;

  return verifyTrackLayout(list, lvdb->fileSize);
}



/*
 * =============================================================================
 *   memoReadConfigDisk
 * =============================================================================
 */
static KvaMemoStatus WINAPI memoReadConfigDisk(const MemoHandle h, uint8_t *const buf,
                                        const size_t buflen, size_t *const actual_len)

{
  KvaMemoStatus status = MemoStatusOK;

  PRINTF(("memoReadConfigDisk\n"));

  if (!buf || !h ) {
    return MemoStatusERR_PARAM;
  }

  if (actual_len) {
    *actual_len = 0;
  }
  memoConf *mc = (memoConf *)h;

  if(mc->kmfFileName.empty()) {
    PRINTF(("memoReadConfigDisk: Not connected to a SD reader.\n"));
    return MemoStatusERR_DEVICE_COMM_ERROR;
  }
  // Recast the path to the kvm log file into something a path to the config file.
  std::string drive, dir, path, base, ext;
  if ( !os_splitpath(mc->kmfFileName, drive, dir, base, ext)) {
    return MemoStatusERR_PARAM;
  }
  os_makepath(path, drive, dir, CONFIG_FILE_NAME, "");

  // Open the configuration file on SD card.
  PRINTF(("memoReadConfigDisk: Opening file '%s'\n", path.c_str()));
  FILE* configFile = fopen(path.c_str(), "rb");
  if (!configFile) {
    // The configuration file is missing which indicates that the SD card needs to be formatted.
    return MemoStatusERR_NOT_FORMATTED;
  }

  size_t bytesRead = fread(buf, 1, buflen, configFile);
  fclose(configFile);

  // Find actual length of config data.
  if (bytesRead) {
    uint8_t    *endbuf = (uint8_t *)buf + bytesRead;
    size_t     L;
    BlockHead  *bh;
    uint8_t    *rp;

    rp = (uint8_t *)buf;
    bh = (BlockHead*) buf;
    L = 0;
    status = MemoStatusERR_CONFIG_ERROR;
    while (rp + sizeof(BlockHead) <= endbuf) {

      if (bh->id == 0) {
        PRINTF(("BLOCK_ID_ZERO\n"));
        if (L == 0) {
          PRINTF(("Config file is empty.\n"));
          status = MemoStatusOK;
        } else {
          status = MemoStatusERR_CONFIG_ERROR;
        }
        break;
      }

      if (bh->len == 0) {
        PRINTF(("Error: Config block length is zero.\n"));
        status = MemoStatusERR_CONFIG_ERROR;
        break;
      }

      if (bh->id == BLOCK_ID_END) {
        PRINTF(("BLOCK_ID_END\n"));
        L += sizeof(BlockHead);
        status = MemoStatusOK;
        break;
      }

      PRINTF(("BLOCK_ID: %02d (%u)\n", bh->id, bh->len));
      rp += bh->len;
      L += bh->len;
      bh = (BlockHead*) rp;
    }
    if (actual_len) {
      *actual_len = L;
    }
  }
  else {
    status = MemoStatusERR_NOT_FORMATTED;
  }

  return status;
} // memoReadConfigDisk




