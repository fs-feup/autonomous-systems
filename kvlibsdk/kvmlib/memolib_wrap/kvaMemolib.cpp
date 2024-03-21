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
 **   Library for accessing Eagle.
 **
 **   Scenarios
 **
 **   1: users continue to use existing apps with existing
 **   kvamemolib.dll. They can then use existing memorators (f/w 2.5
 **   and on) as long as we don't change anything vital in the KMF
 **   code.
 **
 **   2. They install the new kvamemolib.dll plus the associated
 **   kvamemolibXXXX.dll (all of them.) The application is unchanged.
 **   Works as (1). They will use the old API and the default DLL will
 **   be loaded; no difference.
 **
 **   3) Install new kvamemolib.dll and only the corresponding
 **   kvamemolibXXXX for memo2 (but not for memo1) They will then use
 **   the old API but will access the memo2 instead.
 **
 **   4) Install new DLLs and modify the app to call the new init
 **   function instead. Should work as before, but with both memo1 and
 **   memo2. The old API can be used if they want.
 **
 **   5) New applications should use the new API.
 ** ---------------------------------------------------------------------------
 */

#include <stdio.h>
#include <time.h>
#include <sys/stat.h>
#include <stdlib.h>
#include <cstring>

#include "kvaMemolib.h"
#include "canlib_version.h"
#include "jumptable_25.h"
#include "os_util.h"
#include "kvdebug.h"

#ifdef DEBUG
#include <stdarg.h>
#endif

static const std::string KVAMEMOLIB0600 = "kvamemolib0600";
static const std::string KVAMEMOLIB0700 = "kvamemolib0700";

//
// The jump table. Contains version information and a union that
// contains an instance of the right type of jump table (there's
// currently only one type but we might get more in the future)
//
typedef struct {
  unsigned int version;
  InstanceHandle hInst;
  union {
    Jumptable_25 j25;
  } u;
} Jumptable;

typedef struct {
  MemoHandle h;
  int device;
} MemoWrapHandle;

//
// Preprocessor magic.
//
// First, a macro to convert the handle from our caller to the handle
// used by the subordinate DLL. Our handle is simply a pointer to the
// subordinate DLL's handle. "Every problem can be solved by adding
// another level of indirection."
//
// #define HND(x) ((MemoHandle)(*(MemoHandle*)x))

#define HND(x) (((MemoWrapHandle*)(x))->h)

//
//
// And a macro which gets the address of a function from the
// subordinate DLL
#define J25_DEFINE(x) j->u.j25.x = (x##_t) GPA(j->hInst, #x )


// Global jumptable.
// NOTE: This means we can currently only handle a single connection.
Jumptable *jp = NULL;

// DLL initialisation status.
bool initDone = false;
bool oldInitDone = false;
bool newInitDone = false;
bool dllLoadFailed = false;


// Wrapper for Win32 GetProcAddress().
static InstanceAdress GPA(InstanceHandle h, const char *proc)
{
  InstanceAdress f;
  f = os_get_symbol_adress(h, proc);
  PRINTF(("%s -> %p\n", proc, f));
  if (!f) dllLoadFailed = true;
  return f;
}


static int loadMemoLib(unsigned int version, Jumptable **jump)
{
  Jumptable *j;

  dllLoadFailed = false;
  j = (Jumptable*)calloc(1, sizeof(Jumptable));
  j->version = version;

  switch (version) {
    case 0x0600:
    {
      PRINTF(("loadMemoLib: loading kvamemolib0600\n"));
      j->hInst = os_load_library(KVAMEMOLIB0600);
      if (!j->hInst) dllLoadFailed = true;
      J25_DEFINE(memoGetDbaseFile);
      J25_DEFINE(memoPutDbaseFile);
      J25_DEFINE(memoEraseDbaseFile);
      J25_DEFINE(memoPartitionDisk);
      J25_DEFINE(memoGetTrackCount);
      J25_DEFINE(memoGetFileTrackUsage);
      J25_DEFINE(memoClearDataTrack);
      J25_DEFINE(memoGetLogFileCountTrack);
      J25_DEFINE(memoInitializeLibrary);
      J25_DEFINE(memoOpen);
      J25_DEFINE(memoClose);
      J25_DEFINE(memoProbeDeviceVersion);
      J25_DEFINE(memoProbeFileVersion);
      J25_DEFINE(memoReadFilesystemInfo);
      J25_DEFINE(memoReadDiskInfo);
      J25_DEFINE(memoFormatDisk);
      J25_DEFINE(memoFormatDiskEx);
      J25_DEFINE(memoClearDataDisk);
      J25_DEFINE(memoGetRTC);
      J25_DEFINE(memoSetRTC);
      J25_DEFINE(memoConfigModeRefresh);
      J25_DEFINE(memoConfigModeSetInterval);
      J25_DEFINE(memoConfigModeGetInterval);
      J25_DEFINE(memoConfigModeSetAutomatic);
      J25_DEFINE(memoConfigModeGetDiskStatus);
      J25_DEFINE(memoConfigModeGetMode);
      J25_DEFINE(memoFlashAllLeds);
      J25_DEFINE(memoOpenDisk);
      J25_DEFINE(memoOpenDiskFile);
      J25_DEFINE(memoCloseDisk);
      J25_DEFINE(memoReopenDisk);
      J25_DEFINE(memoValidateDisk);
      J25_DEFINE(memoGetLogFileCountEx);
      J25_DEFINE(memoLogOpenFile);
      J25_DEFINE(memoLogCloseFile);
      J25_DEFINE(memoLogGetStartTime);
      J25_DEFINE(memoLogGetEndTime);
      J25_DEFINE(memoLogReadEventEx);
      J25_DEFINE(memoDiskReadPhysicalSector);
      J25_DEFINE(memoDiskReadLogicalSector);
      J25_DEFINE(memoDiskWritePhysicalSector);
      J25_DEFINE(memoDiskWriteLogicalSector);
      J25_DEFINE(memoDiskErasePhysicalSector);
      J25_DEFINE(memoDiskEraseLogicalSector);
      J25_DEFINE(memoSetCallback);
      J25_DEFINE(memoSetCallbackEx);
      J25_DEFINE(memoGetSoftwareVersionInfo);
      J25_DEFINE(memoGetSerialNumber);
      J25_DEFINE(memoGetFileSystemUsage);
      J25_DEFINE(memoReadConfig);
      J25_DEFINE(memoWriteConfig);
      J25_DEFINE(memoGetHardwareInfo);
      J25_DEFINE(memoLogGetSerial);
      J25_DEFINE(memoLogGetEan);
      break;
    }

    case 0x0700:
    {
      PRINTF(("loadMemoLib: loading kvamemolib0700\n"));
      j->hInst = os_load_library(KVAMEMOLIB0700);
      if (!j->hInst) dllLoadFailed = true;
      J25_DEFINE(memoGetDbaseFile);
      J25_DEFINE(memoPutDbaseFile);
      J25_DEFINE(memoEraseDbaseFile);
      J25_DEFINE(memoPartitionDisk);
      J25_DEFINE(memoGetTrackCount);
      J25_DEFINE(memoGetFileTrackUsage);
      J25_DEFINE(memoClearDataTrack);
      J25_DEFINE(memoGetLogFileCountTrack);
      J25_DEFINE(memoInitializeLibrary);
      J25_DEFINE(memoOpen);
      J25_DEFINE(memoClose);
      J25_DEFINE(memoProbeDeviceVersion);
      J25_DEFINE(memoProbeFileVersion);
      J25_DEFINE(memoReadFilesystemInfo);
      J25_DEFINE(memoReadDiskInfo);
      J25_DEFINE(memoFormatDisk);
      J25_DEFINE(memoFormatDiskEx);
      J25_DEFINE(memoClearDataDisk);
      J25_DEFINE(memoGetRTC);
      J25_DEFINE(memoSetRTC);
      J25_DEFINE(memoConfigModeRefresh);
      J25_DEFINE(memoConfigModeSetInterval);
      J25_DEFINE(memoConfigModeGetInterval);
      J25_DEFINE(memoConfigModeSetAutomatic);
      J25_DEFINE(memoConfigModeGetDiskStatus);
      J25_DEFINE(memoConfigModeGetMode);
      J25_DEFINE(memoFlashAllLeds);
      J25_DEFINE(memoOpenDisk);
      J25_DEFINE(memoOpenDiskFile);
      J25_DEFINE(memoCloseDisk);
      J25_DEFINE(memoReopenDisk);
      J25_DEFINE(memoValidateDisk);
      J25_DEFINE(memoGetLogFileCountEx);
      J25_DEFINE(memoLogOpenFile);
      J25_DEFINE(memoLogCloseFile);
      J25_DEFINE(memoLogGetStartTime);
      J25_DEFINE(memoLogGetEndTime);
      J25_DEFINE(memoLogReadEventEx);
      J25_DEFINE(memoDiskReadPhysicalSector);
      J25_DEFINE(memoDiskReadLogicalSector);
      J25_DEFINE(memoDiskWritePhysicalSector);
      J25_DEFINE(memoDiskWriteLogicalSector);
      J25_DEFINE(memoDiskErasePhysicalSector);
      J25_DEFINE(memoDiskEraseLogicalSector);
      J25_DEFINE(memoSetCallback);
      J25_DEFINE(memoSetCallbackEx);
      J25_DEFINE(memoGetSoftwareVersionInfo);
      J25_DEFINE(memoGetSerialNumber);
      J25_DEFINE(memoGetFileSystemUsage);
      J25_DEFINE(memoReadConfig);
      J25_DEFINE(memoWriteConfig);
      J25_DEFINE(memoGetHardwareInfo);
      J25_DEFINE(memoLogGetSerial);
      J25_DEFINE(memoLogGetEan);
      break;
    }
    default:
      PRINTF(("ERROR: Could not load DLL 0x%x\n", version));
      dllLoadFailed = false;

      break;
  }
  *jump = j;

  if (dllLoadFailed) {
    initDone = false;
    oldInitDone = false;
    newInitDone = false;
  }
  return 0;
}

void unloadMemoLib(Jumptable *jp)
{
  os_unload_library(jp->hInst);
  memset(jp, 0, sizeof(Jumptable));
  free(jp);
}


//===========================================================================
//
// Old API init function.
//
void WINAPI memoInitializeLibrary(void)
{
  if (!initDone) {
    loadMemoLib(0x0600, &jp);
    if (dllLoadFailed) {
      loadMemoLib(0x0700, &jp);
    }

    if (dllLoadFailed) return;

    jp->u.j25.memoInitializeLibrary();
    initDone = true;
    oldInitDone = true;
    newInitDone = false;
  }
}

//===========================================================================
//
// New API init function.
//
void WINAPI kvmInitializeLibrary(void)
{
  if (!initDone) {
    initDone = newInitDone = true;
    oldInitDone = false;
  }
}

#define MEMOHANDLE_VERSION_NOT_PROBED 1
//===========================================================================
//
// Old API functions come here. Most functions just call the
// corresponding function in the subordinate DLL via the jump table.
//
// A few functions try to obtain the firmware version of the target
// (which can be either a device or a KMF file) and loads the
// appropriate subordinate DLL when this is done.
//
MemoHandle WINAPI memoOpen(KvaMemoStatus *stat, int32 memoNr,
                           int32 flags, memoDevice mDevice)
{
  MemoHandle r = 0;

  *stat = MemoStatusFail;

  if (initDone) {
    if (oldInitDone) {
      //
      // Old application is calling us
      //
      PRINTF(("memoOpen: called by old app\n"));
      if (!jp) r = 0;
      else r = (MemoHandle)jp->u.j25.memoOpen(stat, memoNr, flags, mDevice);
    } else {
      //
      // New application is calling us using old API
      //
      if (!jp) {
        switch (mDevice) {
          case memo_HYDRA:
          case memo_HDISK:
          {
            PRINTF(("memoOpen: memo_HYDRA/memo_HDISK\n"));
            // Load DLL 0600 and probe version
            loadMemoLib(0x0600, &jp);
            if (!dllLoadFailed) {
              jp->u.j25.memoInitializeLibrary();
              PRINTF(("memoOpen: DLL 0600\n"));
              r = (MemoHandle)jp->u.j25.memoOpen(stat, memoNr, flags, mDevice);
              if (*stat != MemoStatusOK) {
                PRINTF(("memOpen Wrap: Unloading DLL 0600\n"));
                unloadMemoLib(jp);
                jp = NULL;
              }
            }
            break;
          }

          case memo_MEMO5:
          case memo_DISK5:
          {
            PRINTF(("memoOpen: memo_MEMO5/memo_DISK5\n"));
            // Load DLL 0700 and probe version
            loadMemoLib(0x0700, &jp);
            if (!dllLoadFailed) {
              jp->u.j25.memoInitializeLibrary();
              PRINTF(("memoOpen: DLL 0700\n"));
              r = (MemoHandle)jp->u.j25.memoOpen(stat, memoNr, flags, mDevice);
              if (*stat != MemoStatusOK) {
                PRINTF(("memOpen Wrap: Unloading DLL 0700\n"));
                unloadMemoLib(jp);
                jp = NULL;
              }
            }
            break;
          }

          default:
            // Unknown device.
            r = (MemoHandle)0;
            *stat = MemoStatusOK;
            break;
        }

      } else {
        // DLL is loaded already.
        r = (MemoHandle)jp->u.j25.memoOpen(stat, memoNr, flags, mDevice);
      }
    }
  }

  if (r != 0) {
    MemoWrapHandle *x = (MemoWrapHandle*)malloc(sizeof(MemoWrapHandle));
    x->h = r;
    x->device = mDevice;
    r = (MemoHandle) x;
  }

  return r;
}




//===========================================================================
// Open the memorator using the drivername (for example kcanl1a)
// This function will call memoOpen
// this is the only implementation of this function i.e. it only
// exists in the memolib_wrapper
//
MemoHandle WINAPI memoOpenDrv(KvaMemoStatus *stat, char *drvName,
                              int32 flags, memoDevice mDevice)
{
  int32 memoNr = 0;
  int   tmp;
  char  c;
  char  d;

  switch (mDevice) {
    case memo_HYDRA:
      if (sscanf(drvName, "kcany%d%c%c", &tmp, &c, &d) == 2) break;
      *stat = MemoStatusERR_PARAM;
      PRINTF(("memoOpenDrv ERROR: driver name didn't match memo_HYDRA\n"));
      return 0;
      break;

    case memo_MEMO5:
      if (sscanf(drvName, "kcany%d%c%c", &tmp, &c, &d) == 2) break;
      *stat = MemoStatusERR_PARAM;
      PRINTF(("memoOpenDrv ERROR: driver name didn't match memo_MEMO5\n"));
      return 0;
      break;

    default:
      // by default open device 0
      PRINTF(("Warning: Opening device 0 by default. \n"));
      return memoOpen(stat, 0, flags, mDevice);
  }

  PRINTF(("memoOpenDrv: drvName = %s -> ", drvName));
  if (!initDone) return 0;

  memoNr = tmp;
  *stat = MemoStatusOK;
  PRINTF(("memoOpenDrv: memonumber = %d\n", memoNr));
  return memoOpen(stat, memoNr, flags, mDevice);
}


//===========================================================================
// Close the connection, unload the subordinate DLL and deallocate the
// handle.
KvaMemoStatus WINAPI memoClose(MemoHandle h)
{
  KvaMemoStatus stat;

  PRINTF(("memoClose wrap\n"));

  if (!initDone) return MemoStatusFail;
  if (!h) return MemoStatusERR_PARAM;

  stat = (KvaMemoStatus)jp->u.j25.memoClose(HND(h));
  unloadMemoLib(jp);
  jp = NULL;
  free((void *)h);
  return stat;
}


KvaMemoStatus WINAPI memoWriteConfig(MemoHandle h, void *buf, size_t buflen)
{
  if (!initDone) return MemoStatusFail;
  return (KvaMemoStatus)jp->u.j25.memoWriteConfig(HND(h), buf, buflen);
}


KvaMemoStatus WINAPI memoReadConfig(MemoHandle h, void *buf, size_t buflen, size_t *actual_len)
{
  if (!initDone) return MemoStatusFail;
  return (KvaMemoStatus)jp->u.j25.memoReadConfig(HND(h), buf, buflen, actual_len);
}

KvaMemoStatus WINAPI memoReadFilesystemInfo(MemoHandle h, fileSysInfo *fSysInfo)
{
  if (!initDone) return MemoStatusFail;
  return (KvaMemoStatus)jp->u.j25.memoReadFilesystemInfo(HND(h), fSysInfo);
}

KvaMemoStatus WINAPI memoGetFileSystemUsage(MemoHandle h, fileSysUsage *fus)
{
  if (!initDone) return MemoStatusFail;
  return (KvaMemoStatus)jp->u.j25.memoGetFileSystemUsage(HND(h), fus);
}

KvaMemoStatus WINAPI memoReadDiskInfo(MemoHandle h, diskInfo *dInf)
{
  if (!initDone) return MemoStatusFail;
  return (KvaMemoStatus)jp->u.j25.memoReadDiskInfo(HND(h), dInf);
}

KvaMemoStatus WINAPI memoFormatDisk(MemoHandle h, uint32 *format_stat)
{
  if (!initDone) return MemoStatusFail;
  return (KvaMemoStatus)jp->u.j25.memoFormatDisk(HND(h), format_stat);
}

KvaMemoStatus WINAPI memoFormatDiskEx(MemoHandle h, char *path,
                                      uint32 flags, uint32 reserve_space,
                                      uint32 max_space, uint32 max_files,
                                      uint32 *format_stat)
{
  if (!initDone) return MemoStatusFail;
  return (KvaMemoStatus)jp->u.j25.memoFormatDiskEx(HND(h), path,
    flags, reserve_space, max_space, max_files, format_stat);
}

KvaMemoStatus WINAPI memoClearDataDisk(MemoHandle h, uint32 *stat)
{
  if (!initDone) return MemoStatusFail;
  return (KvaMemoStatus)jp->u.j25.memoClearDataDisk(HND(h), stat);
}

KvaMemoStatus WINAPI memoGetRTC(MemoHandle h, uint32 *t)
{
  if (!initDone) return MemoStatusFail;
  return (KvaMemoStatus)jp->u.j25.memoGetRTC(HND(h), t);
}

KvaMemoStatus WINAPI memoSetRTC(MemoHandle h, uint32 t)
{
  if (!initDone) return MemoStatusFail;
  return (KvaMemoStatus)jp->u.j25.memoSetRTC(HND(h), t);
}

KvaMemoStatus WINAPI memoFlashAllLeds(MemoHandle h)
{
  if (!initDone) return MemoStatusFail;
  return (KvaMemoStatus)jp->u.j25.memoFlashAllLeds(HND(h));
}

KvaMemoStatus WINAPI memoConfigModeGetInterval(MemoHandle h, int *interval)
{
  if (!initDone) return MemoStatusFail;
  return (KvaMemoStatus)jp->u.j25.memoConfigModeGetInterval(HND(h), interval);
}

KvaMemoStatus WINAPI memoConfigModeGetDiskStatus(MemoHandle h, int *diskStat)
{
  if (!initDone) return MemoStatusFail;
  return (KvaMemoStatus)jp->u.j25.memoConfigModeGetDiskStatus(HND(h), diskStat);
}

KvaMemoStatus WINAPI memoConfigModeGetMode(MemoHandle h, int *configMode)
{
  if (!initDone) return MemoStatusFail;
  return (KvaMemoStatus)jp->u.j25.memoConfigModeGetMode(HND(h), configMode);
}

KvaMemoStatus WINAPI memoConfigModeSetInterval(MemoHandle h, int interval)
{
  if (!initDone) return MemoStatusFail;
  return (KvaMemoStatus)jp->u.j25.memoConfigModeSetInterval(HND(h), interval);
}

KvaMemoStatus WINAPI memoConfigModeRefresh(MemoHandle h)
{
  if (!initDone) return MemoStatusFail;
  return (KvaMemoStatus)jp->u.j25.memoConfigModeRefresh(HND(h));
}

KvaMemoStatus WINAPI memoConfigModeSetAutomatic(MemoHandle h, BOOL onoff)
{
  if (!initDone) return MemoStatusFail;
  return (KvaMemoStatus)jp->u.j25.memoConfigModeSetAutomatic(HND(h), onoff);
}

// "Open" the disk in a connected memorator.
KvaMemoStatus WINAPI memoOpenDisk(MemoHandle h, uint32 *fileVersion)
{
  if (!initDone) return MemoStatusFail;
  return (KvaMemoStatus)jp->u.j25.memoOpenDisk(HND(h), fileVersion);
}

// Open a KMF file.
KvaMemoStatus WINAPI memoOpenDiskFile(MemoHandle h, const char *fn, uint32 *fileVersion)
{
  if (!initDone) return MemoStatusFail;
  return (KvaMemoStatus) jp->u.j25.memoOpenDiskFile(HND(h), fn, fileVersion);
}

KvaMemoStatus WINAPI memoCloseDisk(MemoHandle h)
{
  if (!initDone) return MemoStatusFail;
  return (KvaMemoStatus)jp->u.j25.memoCloseDisk(HND(h));
}

KvaMemoStatus WINAPI memoReopenDisk(MemoHandle h, uint32 *appVersion)
{
  if (!initDone) return MemoStatusFail;
  return (KvaMemoStatus)jp->u.j25.memoReopenDisk(HND(h), appVersion);
}

KvaMemoStatus WINAPI memoValidateDisk(MemoHandle h, uint32 complete, uint32 fix)
{
  if (!initDone) return MemoStatusFail;
  return (KvaMemoStatus)jp->u.j25.memoValidateDisk(HND(h), complete, fix);
}

KvaMemoStatus WINAPI memoGetLogFileCountEx(MemoHandle h, uint32* fileCount)
{
  if (!initDone) return MemoStatusFail;
  return (KvaMemoStatus)jp->u.j25.memoGetLogFileCountEx(HND(h), fileCount);
}

KvaMemoStatus WINAPI memoDiskReadPhysicalSector(MemoHandle h, uint32 sectorNo,
                                                void* buf, size_t bufsize)
{
  if (!initDone) return MemoStatusFail;
  return (KvaMemoStatus)jp->u.j25.memoDiskReadPhysicalSector(HND(h), sectorNo, buf, bufsize);
}

KvaMemoStatus WINAPI memoDiskReadLogicalSector(MemoHandle h, unsigned int sectorNo,
                                               void* buf, size_t bufsize)
{
  if (!initDone) return MemoStatusFail;
  return (KvaMemoStatus)jp->u.j25.memoDiskReadLogicalSector(HND(h), sectorNo, buf, bufsize);
}

KvaMemoStatus WINAPI memoDiskWritePhysicalSector(MemoHandle h, uint32 sectorNo,
  void* buf, size_t bufsize)
{
  if (!initDone) return MemoStatusFail;
  return (KvaMemoStatus)jp->u.j25.memoDiskWritePhysicalSector(HND(h), sectorNo, buf, bufsize);
}

KvaMemoStatus WINAPI memoDiskWriteLogicalSector(MemoHandle h, unsigned int sectorNo,
                                                void* buf, size_t bufsize)
{
  if (!initDone) return MemoStatusFail;
  return (KvaMemoStatus)jp->u.j25.memoDiskWriteLogicalSector(HND(h), sectorNo, buf, bufsize);
}

KvaMemoStatus WINAPI memoDiskErasePhysicalSector(MemoHandle h, unsigned int sectorNo,
  unsigned int count)
{
  if (!initDone) return MemoStatusFail;
  return (KvaMemoStatus)jp->u.j25.memoDiskErasePhysicalSector(HND(h), sectorNo, count);
}

KvaMemoStatus WINAPI memoDiskEraseLogicalSector(MemoHandle h, unsigned int sectorNo,
                                                unsigned int count)
{
  if (!initDone) return MemoStatusFail;
  return (KvaMemoStatus)jp->u.j25.memoDiskEraseLogicalSector(HND(h), sectorNo, count);
}

KvaMemoStatus WINAPI memoSetCallback(MemoHandle h, memolib_callback_type f)
{
  if (!initDone) return MemoStatusFail;
  return (KvaMemoStatus)jp->u.j25.memoSetCallback(HND(h), f);
}

KvaMemoStatus WINAPI memoSetCallbackEx(MemoHandle h, MemoCallback f)
{
  if (!initDone) return MemoStatusFail;
  return (KvaMemoStatus)jp->u.j25.memoSetCallbackEx(HND(h), f);
}

KvaMemoStatus WINAPI memoGetSerialNumber(MemoHandle h, unsigned int *serial)
{
  if (!initDone) return MemoStatusFail;
  return (KvaMemoStatus)jp->u.j25.memoGetSerialNumber(HND(h), serial);
}

KvaMemoStatus WINAPI memoGetSoftwareVersionInfo(MemoHandle h,
                                                memoVersionInfo itemCode,
                                                unsigned int *major,
                                                unsigned int *minor,
                                                unsigned int *build,
                                                unsigned int *flags)
{
  if (!initDone) return MemoStatusFail;
  return (KvaMemoStatus)jp->u.j25.memoGetSoftwareVersionInfo(HND(h), itemCode, major, minor, build, flags);
}

KvaMemoStatus WINAPI memoLogOpenFile(MemoHandle h,
                                     uint32 fileIndx,
                                     uint64 *eventCount)
{
  if (!initDone) return MemoStatusFail;
  return (KvaMemoStatus)jp->u.j25.memoLogOpenFile(HND(h), fileIndx, eventCount);
}

KvaMemoStatus WINAPI memoLogGetStartTime(MemoHandle h, uint32 *start_time)
{
  if (!initDone) return MemoStatusFail;
  return (KvaMemoStatus)jp->u.j25.memoLogGetStartTime(HND(h), start_time);
}

KvaMemoStatus WINAPI memoLogGetEndTime(MemoHandle h, uint32 *end_time)
{
  if (!initDone) return MemoStatusFail;
  return (KvaMemoStatus)jp->u.j25.memoLogGetEndTime(HND(h), end_time);
}

KvaMemoStatus WINAPI memoLogReadEventEx(MemoHandle h, memoLogEventEx *e)
{
  if (!initDone) return MemoStatusFail;
  return (KvaMemoStatus)jp->u.j25.memoLogReadEventEx(HND(h), e);
}

void WINAPI memoLogCloseFile (MemoHandle h)
{
  if (!initDone) return;
  jp->u.j25.memoLogCloseFile(HND(h));
}

KvaMemoStatus WINAPI memoGetHardwareInfo(MemoHandle h, MemoHardwareInfo *hinfo)
{
  if (!initDone) return MemoStatusFail;
  return (KvaMemoStatus)jp->u.j25.memoGetHardwareInfo(HND(h), hinfo);
}

KvaMemoStatus WINAPI memoGetDbaseFile(MemoHandle h, int filenumber, char *path, char *dbname)
{
  if (!initDone) return MemoStatusFail;
  return (KvaMemoStatus)jp->u.j25.memoGetDbaseFile(HND(h), filenumber, path, dbname);
}

KvaMemoStatus WINAPI memoPutDbaseFile(MemoHandle h, int filenumber, char *filename)
{
  if (!initDone) return MemoStatusFail;
  return (KvaMemoStatus)jp->u.j25.memoPutDbaseFile(HND(h), filenumber, filename);
}

KvaMemoStatus WINAPI memoEraseDbaseFile(MemoHandle h, int filenumber)
{
  if (!initDone) return MemoStatusFail;
  return (KvaMemoStatus)jp->u.j25.memoEraseDbaseFile(HND(h), filenumber);
}

KvaMemoStatus WINAPI memoPartitionDisk(MemoHandle h, char *path, int tracks)
{
  if (!initDone) return MemoStatusFail;
  return (KvaMemoStatus)jp->u.j25.memoPartitionDisk(HND(h), path, tracks);
}

KvaMemoStatus WINAPI  memoGetTrackCount(MemoHandle h, int* tracks)
{
  if (!initDone) return MemoStatusFail;
  return (KvaMemoStatus)jp->u.j25.memoGetTrackCount(HND(h), tracks);
}

KvaMemoStatus WINAPI memoGetFileTrackUsage(MemoHandle h, int track, fileSysUsage *fus)
{
  if (!initDone) return MemoStatusFail;
  return (KvaMemoStatus)jp->u.j25.memoGetFileTrackUsage(HND(h), track, fus);
}

KvaMemoStatus WINAPI memoClearDataTrack(MemoHandle h, int track)
{
  if (!initDone) return MemoStatusFail;
  return (KvaMemoStatus)jp->u.j25.memoClearDataTrack(HND(h), track);
}

KvaMemoStatus WINAPI memoGetLogFileCountTrack(MemoHandle h, int track, uint32* fileCount)
{
  if (!initDone) return MemoStatusFail;
  return (KvaMemoStatus)jp->u.j25.memoGetLogFileCountTrack(HND(h), track, fileCount);
}

KvaMemoStatus WINAPI memoLogGetSerial(MemoHandle h, uint32 *serialNumber)
{
  if (!initDone) return MemoStatusFail;
  return (KvaMemoStatus)jp->u.j25.memoLogGetSerial(HND(h), serialNumber);
}

KvaMemoStatus WINAPI memoLogGetEan(MemoHandle h, uint32 *eanHi, uint32 *eanLo)
{
  if (!initDone) return MemoStatusFail;
  return (KvaMemoStatus)jp->u.j25.memoLogGetEan(HND(h), eanHi, eanLo);
}

//===========================================================================
// #if defined(_DEBUG) || defined(DEBUG)
// void dbg_printf(char* msg, ...)
// {
//   char      tmp_buffer[1024];
//   va_list   ap;

//   va_start(ap, msg);
//   vsprintf(tmp_buffer, msg, ap);
//   va_end(ap);

//   OutputDebugString(tmp_buffer);
// }
// #endif
