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
**   Library for accessing Eagle; consisting of wrapped calls to kvamemolib.
** -----------------------------------------------------------------------------
*/

#define NOMINMAX
#include <algorithm>
#include <string>
#include <cstring>

#include "kvmlib.h"
#include "canlib_version.h"
#include "filehandler.h"
#include "versionhandler.h"
#include "util.h"
#include "kvdebug.h"
#include "kvaMemolib.h"

// Device selection and detection
#define KVM_USE_DISK   1
#define KVM_USE_DEVICE 2
typedef struct {
  kvmHandle h;
  int device;
} kvmHandleDevice;
#define DEVICE(x) (((kvmHandleDevice*)(x))->device)

#define DOS_FILENAME_SIZE 12

// ===========================================================================
static int is_handle_valid(kvmHandle h)
{
  int stat = 1;

  // Naive check that should be replaced by full handle validation
  if (h == INVALID_HANDLE_VALUE) {
    stat = 0;
  }
  return stat;
}

// ===========================================================================
void WINAPI kvmInitialize(void) {
  kvmInitializeLibrary();
}

// ===========================================================================
kvmStatus WINAPI kvmGetVersion(int *major, int *minor, int *build) {

  if (!major || !minor || !build) {
    return kvmERR_PARAM;
  }
  *major = CANLIB_MAJOR_VERSION;
  *minor = CANLIB_MINOR_VERSION;
  *build = 0;
  return kvmOK;
}

// ===========================================================================
static kvmStatus kvmGetMemoType (int32 deviceType,
                                     uint32 connType,
                                     memoDevice *mDevice)
{
  kvmStatus stat = kvmOK;

  switch (deviceType) {

  case kvmDEVICE_MHYDRA:
    if (connType == KVM_USE_DISK) {
      *mDevice = memo_HDISK;
    } else {
      *mDevice = memo_HYDRA;
    }
    break;

  case kvmDEVICE_MHYDRA_EXT:
    if (connType == KVM_USE_DISK) {
      *mDevice = memo_DISK5;
    } else {
      *mDevice = memo_MEMO5;
    }
    break;

  default:
    stat = kvmERR_PARAM;
  }
  return stat;
}

// ---------------------------------------------------------------------------
static FileType toFileType (int32 type)
{
  switch (type) {
    case kvmFILE_KME24: return KME24;
    case kvmFILE_KME25: return KME25;
    case kvmFILE_KME40: return KME40;
    case kvmFILE_KME50: return KME50;
    case kvmFILE_KME60: return KME60;

    default:
      PRINTF(("Unknown kvmFileType: %d.\n", type));
      return UNKNOWN_FILE_TYPE;
  }
}

// ---------------------------------------------------------------------------
static kvmStatus tokvmStatus (FileStatus status)
{
  switch (status) {
    case FileStatusOK:                  return kvmOK;
    case FileStatusFail:                return kvmFail;
    case FileStatusERR_PARAM:           return kvmERR_PARAM;
    case FileStatusERR_NOLOGMSG:        return kvmERR_NOLOGMSG;
    case FileStatusERR_QUEUE_FULL:      return kvmERR_QUEUE_FULL;
    case FileStatusERR_FILE_ERROR:      return kvmERR_FILE_ERROR;
    case FileStatusERR_NOT_IMPLEMENTED: return kvmERR_NOT_IMPLEMENTED;
    case FileStatusERR_ILLEGAL_REQUEST: return kvmERR_ILLEGAL_REQUEST;
    case FileStatusERR_FILE_NOT_FOUND:  return kvmERR_FILE_NOT_FOUND;
    case FileStatusERR_WRITE_PROT:      return kvmERR_WRITE_PROT;
    case FileStatusERR_INVALID_HANDLE:  return kvmERR_PARAM;
    case FileStatusERR_NO_FREE_HANDLE:  return kvmERR_OCCUPIED;

    default:
      PRINTF(("Unmapped FileStatus: %d. Using kvmFail.\n", status));
      return kvmFail;
  }
}

// ===========================================================================
kvmHandle    WINAPI kvmDeviceOpen(int32 cardNr,
                                     kvmStatus *status,
                                     int32 deviceType)
{
  kvmHandle h = INVALID_HANDLE_VALUE;
  memoDevice mDevice;

  // Get the device type
  *status = kvmGetMemoType(deviceType, KVM_USE_DEVICE, &mDevice);
  if ( kvmOK != *status) return h;

  // Connect to the device
  h = memoOpen((KvaMemoStatus*)status, cardNr, 0, mDevice);
  if (kvmOK != *status) return h;

  *status = (kvmStatus)memoConfigModeSetInterval(h, -1);
  if (kvmOK != *status) return h;
  memoConfigModeRefresh(h);

  return h;
}

// ===========================================================================
kvmStatus WINAPI kvmDeviceMountKmf(kvmHandle h)
{
  uint32 fileVersion;

  if (is_handle_valid(h) == 0) {
    return kvmERR_PARAM;
  }

  return (kvmStatus)memoOpenDisk(h, &fileVersion);
}

// ===========================================================================
kvmStatus WINAPI kvmDeviceMountKmfEx(kvmHandle h, int *ldfMajor, int *ldfMinor)
{
  uint32 fileVersion = 0;
  kvmStatus status;

  if (is_handle_valid(h) == 0) {
    return kvmERR_PARAM;
  }

  if (!ldfMajor || !ldfMinor) return kvmERR_PARAM;

  status = (kvmStatus)memoOpenDisk(h, &fileVersion);
  if (kvmOK == status) {
    *ldfMajor = (int)((fileVersion & 0x0000ff00) >> 8);
    *ldfMinor = (int) (fileVersion & 0x000000ff);
    // Fix for old major LIO version in first byte
    if (!*ldfMajor) {
      *ldfMajor = *ldfMinor;
      *ldfMinor = 0;
    }
  }
  return status;
}

// ===========================================================================
kvmHandle    WINAPI kvmKmfOpen  (const char *filename,
                                   kvmStatus *status,
                                   int32 deviceType)
{
  kvmHandle h = INVALID_HANDLE_VALUE;
  uint32 fileVersion;
  memoDevice mDevice;

  // Get the device type
  *status = kvmGetMemoType(deviceType, KVM_USE_DISK, &mDevice);
  if (kvmOK  != *status) return h;


  // Connect to a file
  h = memoOpen((KvaMemoStatus*)status, 0, 0, mDevice);
  if (*status != kvmOK ) return h;

  *status = (kvmStatus)memoOpenDiskFile(h, filename, &fileVersion);
  if (kvmOK != *status) {
    // memoOpen succeded so we need to close the handle
    memoClose(h);
    h = INVALID_HANDLE_VALUE;
  }
  return h;
}

// ===========================================================================
kvmHandle    WINAPI kvmKmfOpenEx  (const char *filename,
                                   kvmStatus *status,
                                   int32 deviceType,
                                   int *ldfMajor,
                                   int *ldfMinor)
{
  kvmHandle h = INVALID_HANDLE_VALUE;
  uint32 fileVersion;
  memoDevice mDevice;

  if (!status) {
    return h;
  }

  if (!ldfMajor || !ldfMinor) {
    *status = kvmERR_PARAM;
    return h;
  }

  // Get the device type
  *status = kvmGetMemoType(deviceType, KVM_USE_DISK, &mDevice);
  if (kvmOK  != *status) return h;


  // Connect to a file
  h = memoOpen((KvaMemoStatus*)status, 0, 0, mDevice);
  if (*status != kvmOK ) return h;

  *status = (kvmStatus)memoOpenDiskFile(h, filename, &fileVersion);
  if (kvmOK == *status) {
    *ldfMajor = (int)((fileVersion & 0x0000ff00) >> 8);
    *ldfMinor = (int) (fileVersion & 0x000000ff);
    // Fix for old major LIO version in first byte
    if (!*ldfMajor) {
      *ldfMajor = *ldfMinor;
      *ldfMinor = 0;
    }
  } else {
    // memoOpen succeded so we need to close the handle
    memoClose(h);
    h = INVALID_HANDLE_VALUE;
  }

  return h;
}

// ===========================================================================
kvmStatus WINAPI kvmClose(kvmHandle h)
{
  kvmStatus status;

  if (is_handle_valid(h) == 0) {
    return kvmERR_PARAM;
  }

  status = (kvmStatus)memoCloseDisk(h);
  if (kvmOK != status) {
    // Close anyway; stops configuration mode
    memoClose(h);
    return status;
  }
  return (kvmStatus)memoClose(h);
}

// ===========================================================================
kvmStatus WINAPI kvmKmfValidate(kvmHandle h)
{
  if (is_handle_valid(h) == 0) {
    return kvmERR_PARAM;
  }

  return (kvmStatus)memoValidateDisk(h, 0, 0);
}

// ===========================================================================
kvmStatus WINAPI kvmDeviceFormatDisk(kvmHandle h,
                                   int fat32,
                                   uint32 reserve_space,
                                   uint32 dbase_space)
{
  uint32 flags = MEMOLIB_FORMAT_RECREATE;
  uint32 format_stat = 0;
  kvmStatus status;

  if (is_handle_valid(h) == 0) {
    return kvmERR_PARAM;
  }

  PRINTF(("kvmDeviceFormatDisk fat32=%d reserve_space=%d dbase_space=%d", fat32, reserve_space, dbase_space));
  if (fat32) flags |= MEMOLIB_FORMAT_FAT32;

  switch (DEVICE(h)) {
    case memo_MEMO3:
    case memo_HYDRA:
    case memo_MEMO5:
      status = (kvmStatus)memoFormatDiskEx(h, NULL, flags, reserve_space, 0, dbase_space, &format_stat);
      break;

    default:
      status = kvmERR_PARAM;
  }
  return status;
}

// ===========================================================================
kvmStatus WINAPI kvmLogFileGetCount(kvmHandle h, uint32 *fileCount)
{
  kvmStatus stat;
  uint32    n_file = 0;
  uint32    n_track;
  
  if (is_handle_valid(h) == 0) {
    return kvmERR_PARAM;
  }
  
  stat = (kvmStatus)memoGetTrackCount(h, (int*)&n_track);
  if (stat != kvmOK) {
    return stat;
  }
  
  for (uint32 track = 0; track < n_track; track++) {
    stat = (kvmStatus)memoGetLogFileCountTrack(h, (int)track, fileCount);
    if (stat == kvmOK) {
      n_file += *fileCount;
    } else {
      return stat;
    }
  }
  
  *fileCount = n_file;
  return stat;
}

// ===========================================================================
kvmStatus WINAPI kvmLogFileGetType(kvmHandle h, uint32 fileIndx, int32 *filetype)
{
  kvmStatus stat;
  uint32    n_file;
  uint32    n_track;
  
  if (is_handle_valid(h) == 0) {
    return kvmERR_PARAM;
  }

  stat = kvmLogFileGetCount (h, &n_file);
  if (stat != kvmOK) {
    return stat;
  }

  if (fileIndx >= n_file) {
    return kvmERR_PARAM;
  }

  stat = (kvmStatus)memoGetTrackCount(h, (int*)&n_track);
  if (stat != kvmOK) {
    return stat;
  }

  //if we have 1 track,  all files is "kvmLogFileType_ALL files"
  //if we have 2 tracks, all files on track 0 is "kvmLogFileType_ERR files"
  if (n_track == 1) {
    *filetype = kvmLogFileType_ALL;
  } else if (n_track == 2) {
    stat = (kvmStatus)memoGetLogFileCountTrack(h, 0, &n_file);
    if (stat == kvmOK) {
      if (fileIndx < n_file) {
	*filetype = kvmLogFileType_ERR;
      } else {
	*filetype = kvmLogFileType_ALL;
      }
    } else {
      return stat;
    }
  } else {
    return kvmERR_DISK_ERROR;
  }

  return stat;
}

// ===========================================================================
kvmStatus WINAPI kvmLogFileMount(kvmHandle h, uint32 fileIndx,  uint32 *eventCount)
{
  uint64    tmp;
  kvmStatus stat;

  if (is_handle_valid(h) == 0) {
    return kvmERR_PARAM;
  }

  stat = (kvmStatus)memoLogOpenFile(h, fileIndx, &tmp);

  if (stat != kvmOK) {
    return stat;
  }

  *eventCount = (uint32)tmp;

  if (tmp != (uint64)*eventCount) {
    return kvmERR_RESULT_TOO_BIG;
  }

  return stat;
}

// ===========================================================================
kvmStatus WINAPI kvmLogFileMountEx(kvmHandle h, uint32 fileIndx,  int64 *eventCount)
{
  if (is_handle_valid(h) == 0) {
    return kvmERR_PARAM;
  }

  return (kvmStatus)memoLogOpenFile(h, fileIndx, (uint64*)eventCount);
}

// ===========================================================================
kvmStatus WINAPI kvmLogFileDismount(kvmHandle h)
{
  if (is_handle_valid(h) == 0) {
    return kvmERR_PARAM;
  }

  memoLogCloseFile(h);
  return kvmOK;
}

// ===========================================================================
kvmStatus WINAPI kvmLogFileGetStartTime(kvmHandle h, uint32 *start_time)
{
  if (is_handle_valid(h) == 0) {
    return kvmERR_PARAM;
  }

  return (kvmStatus)memoLogGetStartTime(h, start_time);
}

// ===========================================================================
kvmStatus WINAPI kvmLogFileGetEndTime(kvmHandle h, uint32 *end_time)
{
  if (is_handle_valid(h) == 0) {
    return kvmERR_PARAM;
  }

  return (kvmStatus)memoLogGetEndTime(h, end_time);
}

// ===========================================================================
kvmStatus WINAPI kvmLogFileGetCreatorSerial(kvmHandle h, uint32 *serialNumber)
{
  kvmStatus status = kvmOK;

  if (is_handle_valid(h) == 0) {
    return kvmERR_PARAM;
  }

  switch (DEVICE(h)) {
    case memo_HYDRA:
    case memo_HDISK:
    case memo_MEMO5:
    case memo_DISK5:
      status = (kvmStatus)memoLogGetSerial(h, serialNumber);
      break;

    default:
      status = kvmERR_NOT_IMPLEMENTED;
  }
  return status;
}

// ===========================================================================
kvmStatus WINAPI kvmLogFileReadEvent(kvmHandle h, kvmLogEventEx *e)
{
  if (is_handle_valid(h) == 0) {
    return kvmERR_PARAM;
  }

  kvmStatus status = (kvmStatus)memoLogReadEventEx(h, (memoLogEventEx*)e);
  if (!status && (e->type == kvmLOG_TYPE_MSG)) {
    uint32 dlc = dlcToBytesFD(e->eventUnion.msg.dlc);
    e->eventUnion.msg.dlc = dlc;
  }
  return status;
}

// ===========================================================================
kvmStatus WINAPI kvmLogFileDeleteAll(kvmHandle h) {
  if (is_handle_valid(h) == 0) {
    return kvmERR_PARAM;
  }
  
  {
    kvmStatus stat;
    uint32    n_track;
    
    stat = (kvmStatus)memoGetTrackCount(h, (int*)&n_track);
    if (stat != kvmOK) {
      return stat;
    }
    
    for (uint32 track = 0; track < n_track; track++) {
      stat = (kvmStatus)memoClearDataTrack(h, (int)track);
      if (stat != kvmOK) {
	return stat;
      }
    }
    return stat;
  }
}

// ===========================================================================
kvmStatus WINAPI kvmDeviceDiskStatus(kvmHandle h, int *present)
{
  if (is_handle_valid(h) == 0) {
    return kvmERR_PARAM;
  }

  return (kvmStatus)memoConfigModeGetDiskStatus(h, present);
}

// ===========================================================================
kvmStatus WINAPI kvmKmfGetUsage(kvmHandle h, uint32 *totalSectorCount, uint32 *usedSectorCount)
{
  fileSysUsage sysinfo;
  kvmStatus status;

  if (is_handle_valid(h) == 0) {
    return kvmERR_PARAM;
  }

  status = (kvmStatus)memoGetFileSystemUsage(h, &sysinfo);
  *totalSectorCount = sysinfo.diskSize;
  *usedSectorCount = sysinfo.usedDiskSize;
  return status;
}

// ===========================================================================
kvmStatus WINAPI kvmDeviceDiskSize(kvmHandle h, uint32 *diskSize)
{
  if (is_handle_valid(h) == 0) {
    return kvmERR_PARAM;
  }

  diskInfo diskinfo;
  kvmStatus status;
  status = (kvmStatus)memoReadDiskInfo(h, &diskinfo);
  *diskSize = diskinfo.data_size;
  return status;
}

// ===========================================================================
kvmStatus WINAPI kvmDeviceGetSerialNumber(kvmHandle h, unsigned int *serial)
{
  if (is_handle_valid(h) == 0) {
    return kvmERR_PARAM;
  }
  return (kvmStatus)memoGetSerialNumber(h, serial);
}

// ===========================================================================
kvmStatus WINAPI kvmDeviceGetSoftwareInfo(kvmHandle h,
                                               int32 itemCode,
                                               unsigned int *major,
                                               unsigned int *minor,
                                               unsigned int *build,
                                               unsigned int *flags)
{
  if (is_handle_valid(h) == 0) {
    return kvmERR_PARAM;
  }

  return (kvmStatus)memoGetSoftwareVersionInfo(h, (memoVersionInfo)itemCode, major, minor, build, flags);
}

// ===========================================================================
kvmStatus WINAPI kvmDeviceFlashLeds(kvmHandle h)
{
  if (is_handle_valid(h) == 0) {
    return kvmERR_PARAM;
  }

  return (kvmStatus)memoFlashAllLeds(h);
}

// ===========================================================================
kvmStatus WINAPI kvmDeviceGetRTC(kvmHandle h, uint32 *t)
{
  if (is_handle_valid(h) == 0) {
    return kvmERR_PARAM;
  }

  return (kvmStatus)memoGetRTC(h, t) ;
}
// ===========================================================================
kvmStatus WINAPI kvmDeviceSetRTC(kvmHandle h, uint32 t)
{
  if (is_handle_valid(h) == 0) {
    return kvmERR_PARAM;
  }

  return (kvmStatus)memoSetRTC(h, t);
}

// ===========================================================================
kvmStatus WINAPI kvmKmfReadConfig(kvmHandle h, void *buf,
                                    size_t buflen, size_t *actual_len)
{
  if (is_handle_valid(h) == 0) {
    return kvmERR_PARAM;
  }

  return (kvmStatus)memoReadConfig(h, buf, buflen, actual_len);
}

// ===========================================================================
kvmStatus WINAPI kvmKmfWriteConfig(kvmHandle h, void *buf, size_t buflen)
{
  if (is_handle_valid(h) == 0) {
    return kvmERR_PARAM;
  }

  return (kvmStatus)memoWriteConfig(h, buf, buflen);
}

// ===========================================================================
kvmStatus WINAPI kvmKmfGetDbaseFile(kvmHandle h, char *path, char *filenamebuf, size_t buflen)
{
  if (buflen < DOS_FILENAME_SIZE + 1) return kvmERR_PARAM;

  if (is_handle_valid(h) == 0) {
    return kvmERR_PARAM;
  }

  if ( (DEVICE(h) == memo_HYDRA) || (DEVICE(h) == memo_HDISK) ||
       (DEVICE(h) == memo_MEMO5) || (DEVICE(h) == memo_DISK5) )
  {
    return (kvmStatus)memoGetDbaseFile(h, 0, path, filenamebuf);
  }

  return kvmERR_PARAM;
}

// ===========================================================================
kvmStatus WINAPI kvmKmfPutDbaseFile(kvmHandle h, char *filename)
{
  if (is_handle_valid(h) == 0) {
    return kvmERR_PARAM;
  }

  if ( (DEVICE(h) == memo_HYDRA) || (DEVICE(h) == memo_HDISK) ||
       (DEVICE(h) == memo_MEMO5) || (DEVICE(h) == memo_DISK5) )
  {
    return (kvmStatus)memoPutDbaseFile(h, 0, filename);
  }
  return kvmERR_PARAM;
}

// ===========================================================================
kvmStatus WINAPI kvmKmfEraseDbaseFile(kvmHandle h)
{
  if (is_handle_valid(h) == 0) {
    return kvmERR_PARAM;
  }

  if ( (DEVICE(h) == memo_HYDRA) || (DEVICE(h) == memo_HDISK) ||
       (DEVICE(h) == memo_MEMO5) || (DEVICE(h) == memo_DISK5) )
  {
    return (kvmStatus)memoEraseDbaseFile(h, 0);
  }
  return kvmERR_PARAM;
}

// ===========================================================================
kmeFileHandle WINAPI kvmKmeOpenFile (const char *filename,
                                 kvmStatus *status,
                                 int32 fileType)
{
  FileStatus fStatus;
  FileHandle fHandle;

  if (!status) {
     return INVALID_HANDLE_VALUE;
  }

  if (!filename) {
    *status = kvmERR_PARAM;
    return INVALID_HANDLE_VALUE;
  }

  fHandle = openFile(filename, &fStatus, toFileType(fileType));
  *status = tokvmStatus(fStatus);
  return (kmeFileHandle) fHandle;

}

// ===========================================================================
kmeFileHandle WINAPI kvmKmeCreateFile (const char *filename,
                                   kvmStatus *status,
                                   int32 fileType)
{
  FileStatus fStatus;
  FileHandle fHandle;

  if (!status) {
     return INVALID_HANDLE_VALUE;
  }

  if (!filename) {
    *status = kvmERR_PARAM;
    return INVALID_HANDLE_VALUE;
  }

  fHandle = createFile(filename, &fStatus, toFileType(fileType));
  *status = tokvmStatus(fStatus);
  return (kmeFileHandle) fHandle;
}

// ===========================================================================
kvmStatus WINAPI kvmKmeReadEvent (kmeFileHandle h, kvmLogEventEx *e)
{
  if (INVALID_HANDLE_VALUE == h || 0 == h || !e) {
    return kvmERR_PARAM;
  }
  return tokvmStatus(readEvent((FileHandle)h, e));
}

// ===========================================================================
kvmStatus WINAPI kvmKmeWriteEvent (kmeFileHandle h, kvmLogEventEx *e)
{
  if (INVALID_HANDLE_VALUE == h || 0 == h || !e) {
    return kvmERR_PARAM;
  }
  return tokvmStatus(writeEvent((FileHandle)h, e));
}

// ===========================================================================
kvmStatus WINAPI kvmKmeCountEvents(kmeFileHandle h,  uint32 *eventCount)
{
 uint64   tmp;
 kvmStatus stat;

  if (INVALID_HANDLE_VALUE == h || 0 == h || !eventCount) {
    return kvmERR_PARAM;
  }

  stat = tokvmStatus(countEvents((FileHandle)h, &tmp));

  if (stat != kvmOK) {
    return stat;
  }

  *eventCount = (uint32)tmp;

  if (tmp != (uint64)*eventCount) {
    stat = kvmERR_RESULT_TOO_BIG;
  }

  return stat;
}

// ===========================================================================
kvmStatus WINAPI kvmKmeCountEventsEx(kmeFileHandle h,  int64 *eventCount)
{
  if (INVALID_HANDLE_VALUE == h || 0 == h || !eventCount) {
    return kvmERR_PARAM;
  }
  kvmStatus stat = tokvmStatus(countEvents((FileHandle)h, (uint64 *)eventCount));
  return stat;
}

// ===========================================================================
kvmStatus WINAPI kvmKmeCloseFile (kmeFileHandle h)
{
  if (INVALID_HANDLE_VALUE == h || 0 == h) {
    return kvmERR_PARAM;
  }
  return tokvmStatus(closeFile((FileHandle)h));
}

// ===========================================================================
kvmStatus    WINAPI kvmKmeScanFileType (const char *filename,
                                        int32 *fileType)
{
  return kmeScanFileType(filename, fileType);
}

// ---------------------------------------------------------------------------
// Error texts
// ---------------------------------------------------------------------------

typedef struct {
    int errcode;
    const char* msg;
} errmsg_t;

static errmsg_t errmsg_table[] = {
  {kvmOK,                       "OK!"},
  {kvmFail,                     "Generic error."},
  {kvmERR_PARAM,                "Error in supplied parameters."},
  {kvmERR_LOGFILEOPEN,          "Can't find/open log file."},
  {kvmERR_NOSTARTTIME,          "Start time not found."},
  {kvmERR_NOLOGMSG,             "No log message found."},
  {kvmERR_LOGFILEWRITE,         "Error writing log file."},
  {kvmEOF,                      "End of file found."},
  {kvmERR_NO_DISK,              "No disk found."},
  {kvmERR_LOGFILEREAD,          "Error while reading log file."},
  {kvmERR_QUEUE_FULL,           "Queue is full."},
  {kvmERR_CRC_ERROR,            "CRC check failed."},
  {kvmERR_SECTOR_ERASED,        "Sector unexpectadly erased."},
  {kvmERR_FILE_ERROR,           "File I/O error."},
  {kvmERR_DISK_ERROR,           "General disk error."},
  {kvmERR_DISKFULL_DIR,         "Disk full (directory)."},
  {kvmERR_DISKFULL_DATA,        "Disk full (data)."},
  {kvmERR_SEQ_ERROR,            "Unexpected sequence."},
  {kvmERR_FILE_SYSTEM_CORRUPT,  "File system corrupt."},
  {kvmERR_UNSUPPORTED_VERSION,  "Unsupported version."},
  {kvmERR_NOT_IMPLEMENTED,      "Not implemented."},
  {kvmERR_FATAL_ERROR,          "Fatal error."},
  {kvmERR_ILLEGAL_REQUEST,      "Illegal request."},
  {kvmERR_FILE_NOT_FOUND,       "File not found."},
  {kvmERR_NOT_FORMATTED,        "Disk not formatted."},
  {kvmERR_WRONG_DISK_TYPE,      "Wrong disk type."},
  {kvmERR_TIMEOUT,              "Timeout."},
  {kvmERR_DEVICE_COMM_ERROR,    "Device communication error."},
  {kvmERR_OCCUPIED,             "Device occupied."},
  {kvmERR_USER_CANCEL,          "User abort."},
  {kvmERR_FIRMWARE,             "Firmware error."},
  {kvmERR_CONFIG_ERROR,         "Configuration error."},
  {kvmERR_WRITE_PROT,           "Disk is write protected."},
  {kvmERR_RESULT_TOO_BIG,       "Result is too big for an out-parameter."},
};

#define ERRMSG_TABLE_LEN  sizeof(errmsg_table)/sizeof(errmsg_table[0])

static bool get_error_text(int code, const char* &msg)
{
  msg = NULL;
  for (unsigned int i=0; i<ERRMSG_TABLE_LEN; i++) {
    if (errmsg_table[i].errcode == code) {
      msg = errmsg_table[i].msg;
      return true;
    }
  }
  return false;
}

// ===========================================================================
kvmStatus WINAPI kvmGetErrorText(kvmStatus error, char *buf, size_t len)
{
  if (error > 0 || !buf || len < 1) return kvmERR_PARAM;

  const char* msg;
  if ( get_error_text(error, msg) ) {
    strncpy(buf, msg, len);
   }
   else {
    strncpy(buf, "Unknown error code.", len);
    buf[len-1] = '\0';
   }
   return kvmOK;
}




