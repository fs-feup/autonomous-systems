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

#ifndef JUMPTABLE_25_H_
#define JUMPTABLE_25_H_

#include "kvaMemolib.h"

//
// Entry points in the subordinate DLL
//
typedef void (WINAPI *memoInitializeLibrary_t)(void);
typedef MemoHandle (WINAPI *memoOpen_t)(KvaMemoStatus *stat, int32 memoNr, int32 flags, memoDevice mDevice);
typedef KvaMemoStatus (WINAPI *memoProbeDeviceVersion_t)(memoDevice mDevice, int32 memoNr, uint32 *firmwareVersion, uint32 *fileVersion, uint32 *appVersion);
typedef KvaMemoStatus (WINAPI *memoProbeFileVersion_t)(const char *fn, uint32 *fileVersion, uint32 *appVersion);
typedef KvaMemoStatus (WINAPI *memoClose_t)(MemoHandle h);
typedef KvaMemoStatus (WINAPI *memoReadFilesystemInfo_t)(MemoHandle h, fileSysInfo * fSysInfo);
typedef KvaMemoStatus (WINAPI *memoGetFileSystemUsage_t)(MemoHandle h, fileSysUsage *fus);
typedef KvaMemoStatus (WINAPI *memoReadDiskInfo_t)(MemoHandle h, diskInfo * disk_info);
typedef KvaMemoStatus (WINAPI *memoFormatDisk_t)(MemoHandle h, uint32 *format_stat);
typedef KvaMemoStatus (WINAPI *memoFormatDiskEx_t)(MemoHandle h, char *path, uint32 flags, uint32 reserve_space, uint32 max_space, uint32 max_files, uint32 *format_stat);
typedef KvaMemoStatus (WINAPI *memoClearDataDisk_t)(MemoHandle h, uint32 *stat);
typedef KvaMemoStatus (WINAPI *memoGetRTC_t)(MemoHandle h, uint32 *t);
typedef KvaMemoStatus (WINAPI *memoSetRTC_t)(MemoHandle h, uint32 t);
typedef KvaMemoStatus (WINAPI *memoConfigModeSetInterval_t)(MemoHandle h, int interval);
typedef KvaMemoStatus (WINAPI *memoConfigModeGetInterval_t)(MemoHandle h, int * interval);
typedef KvaMemoStatus (WINAPI *memoConfigModeRefresh_t)(MemoHandle h);
typedef KvaMemoStatus (WINAPI *memoConfigModeSetAutomatic_t)(MemoHandle h, int onoff);
typedef KvaMemoStatus (WINAPI *memoConfigModeGetDiskStatus_t)(MemoHandle h, int *diskStat);
typedef KvaMemoStatus (WINAPI *memoConfigModeGetMode_t)(MemoHandle h, int *configMode);
typedef KvaMemoStatus (WINAPI *memoFlashAllLeds_t)(MemoHandle h);
typedef KvaMemoStatus (WINAPI *memoOpenDisk_t)(MemoHandle h, uint32 *appVersion);
typedef KvaMemoStatus (WINAPI *memoOpenDiskFile_t)(MemoHandle h, const char *fn, uint32 *appVersion);
typedef KvaMemoStatus (WINAPI *memoCloseDisk_t)(MemoHandle h);
typedef KvaMemoStatus (WINAPI *memoReopenDisk_t)(MemoHandle h, uint32 *appVersion);
typedef KvaMemoStatus (WINAPI *memoValidateDisk_t)(MemoHandle h, uint32 complete, uint32 fix);
typedef KvaMemoStatus (WINAPI *memoGetLogFileCountEx_t)(MemoHandle h, uint32* fileCount);
typedef KvaMemoStatus (WINAPI *memoDiskReadPhysicalSector_t)(MemoHandle h, uint32 sectorNo, void* buf, size_t bufsize);
typedef KvaMemoStatus (WINAPI *memoDiskReadLogicalSector_t)(MemoHandle h, uint32 sectorNo, void* buf, size_t bufsize);
typedef KvaMemoStatus (WINAPI *memoDiskWritePhysicalSector_t)(MemoHandle h, uint32 sectorNo, void* buf, size_t bufsize);
typedef KvaMemoStatus (WINAPI *memoDiskWriteLogicalSector_t)(MemoHandle h, uint32 sectorNo, void* buf, size_t bufsize);
typedef KvaMemoStatus (WINAPI *memoDiskErasePhysicalSector_t)(MemoHandle h, uint32 sectorNo, uint32 count);
typedef KvaMemoStatus (WINAPI *memoDiskEraseLogicalSector_t)(MemoHandle h, uint32 sectorNo, uint32 count);
typedef KvaMemoStatus (WINAPI *memoSetCallback_t)(MemoHandle h, memolib_callback_type f);
typedef KvaMemoStatus (WINAPI *memoSetCallbackEx_t)(MemoHandle h, MemoCallback f);
typedef KvaMemoStatus (WINAPI *memoGetSoftwareVersionInfo_t)(MemoHandle h, memoVersionInfo itemCode, unsigned int *major, unsigned int *minor, unsigned int *build, unsigned int *flags);
typedef KvaMemoStatus (WINAPI *memoGetSerialNumber_t)(MemoHandle h, unsigned int *serial);
typedef KvaMemoStatus (WINAPI *memoLogOpenFile_t)(MemoHandle h, uint32 fileIndx, uint64 *eventCount);
typedef void (WINAPI *memoLogCloseFile_t)(MemoHandle h);
typedef KvaMemoStatus (WINAPI *memoLogGetStartTime_t)(MemoHandle h, uint32 *start_time);
typedef KvaMemoStatus (WINAPI *memoLogGetEndTime_t)(MemoHandle h, uint32 *end_time);
typedef KvaMemoStatus (WINAPI *memoLogReadEvent_t)(MemoHandle h, memoLogEvent *e);
typedef KvaMemoStatus (WINAPI *memoLogReadEventEx_t)(MemoHandle h, memoLogEventEx *e);
typedef KvaMemoStatus (WINAPI *memoReadConfig_t)(MemoHandle h, void *buf, size_t buflen, size_t *actual_len);
typedef KvaMemoStatus (WINAPI *memoWriteConfig_t)(MemoHandle h, void *buf, size_t buflen);
typedef KvaMemoStatus (WINAPI *memoGetHardwareInfo_t)(MemoHandle h, MemoHardwareInfo *hinfo);
typedef KvaMemoStatus (WINAPI *memoGetDbaseFile_t)(MemoHandle h, int filenumber, char *path, char *dbname);
typedef KvaMemoStatus (WINAPI *memoPutDbaseFile_t)(MemoHandle h, int filenumber,  char *filename);
typedef KvaMemoStatus (WINAPI *memoEraseDbaseFile_t)(MemoHandle h, int filenumber);
typedef KvaMemoStatus (WINAPI *memoPartitionDisk_t)(MemoHandle h, char *path, int tracks);
typedef KvaMemoStatus (WINAPI *memoGetTrackCount_t)(MemoHandle h, int* tracks);
typedef KvaMemoStatus (WINAPI *memoGetFileTrackUsage_t)(MemoHandle h, int track, fileSysUsage *fus);
typedef KvaMemoStatus (WINAPI *memoClearDataTrack_t)(MemoHandle h, int track);
typedef KvaMemoStatus (WINAPI *memoGetLogFileCountTrack_t)(MemoHandle h, int track, uint32* fileCount);
typedef KvaMemoStatus (WINAPI *memoLogGetSerial_t)(MemoHandle h, uint32 *serialNumber);
typedef KvaMemoStatus (WINAPI *memoLogGetEan_t)(MemoHandle h, uint32 *eanHi, uint32 *eanLo);

// A macro to save sometyping when defining the jump table
#define DECLARE(x) x##_t x

//
// The jump table for a DLL like the one used in Memorator I, firmware
// 2.5 and similar. Currently this is the only type of jump table we
// use.
//
typedef struct {
  DECLARE(memoInitializeLibrary);
  DECLARE(memoOpen);
  DECLARE(memoClose);
  DECLARE(memoProbeDeviceVersion);
  DECLARE(memoProbeFileVersion);
  DECLARE(memoReadFilesystemInfo);
  DECLARE(memoReadDiskInfo);
  DECLARE(memoFormatDisk);
  DECLARE(memoFormatDiskEx);
  DECLARE(memoClearDataDisk);
  DECLARE(memoGetRTC);
  DECLARE(memoSetRTC);
  DECLARE(memoConfigModeRefresh);
  DECLARE(memoConfigModeSetInterval);
  DECLARE(memoConfigModeGetInterval);
  DECLARE(memoConfigModeSetAutomatic);
  DECLARE(memoConfigModeGetDiskStatus);
  DECLARE(memoConfigModeGetMode);
  DECLARE(memoFlashAllLeds);
  DECLARE(memoOpenDisk);
  DECLARE(memoOpenDiskFile);
  DECLARE(memoCloseDisk);
  DECLARE(memoReopenDisk);
  DECLARE(memoValidateDisk);
  DECLARE(memoGetLogFileCountEx);
  DECLARE(memoLogOpenFile);
  DECLARE(memoLogCloseFile);
  DECLARE(memoLogGetStartTime);
  DECLARE(memoLogGetEndTime);
  DECLARE(memoLogReadEvent);
  DECLARE(memoLogReadEventEx);
  DECLARE(memoDiskReadPhysicalSector);
  DECLARE(memoDiskReadLogicalSector);
  DECLARE(memoDiskWritePhysicalSector);
  DECLARE(memoDiskWriteLogicalSector);
  DECLARE(memoDiskErasePhysicalSector);
  DECLARE(memoDiskEraseLogicalSector);
  DECLARE(memoSetCallback);
  DECLARE(memoSetCallbackEx);
  DECLARE(memoGetSoftwareVersionInfo);
  DECLARE(memoGetSerialNumber);
  DECLARE(memoGetFileSystemUsage);
  DECLARE(memoReadConfig);
  DECLARE(memoWriteConfig);
  DECLARE(memoGetHardwareInfo);
  DECLARE(memoGetDbaseFile);
  DECLARE(memoPutDbaseFile);
  DECLARE(memoEraseDbaseFile);
  DECLARE(memoPartitionDisk);
  DECLARE(memoGetTrackCount);
  DECLARE(memoGetFileTrackUsage);
  DECLARE(memoClearDataTrack);
  DECLARE(memoGetLogFileCountTrack);
  DECLARE(memoLogGetSerial);
  DECLARE(memoLogGetEan);
} Jumptable_25;

#undef DECLARE

#endif
