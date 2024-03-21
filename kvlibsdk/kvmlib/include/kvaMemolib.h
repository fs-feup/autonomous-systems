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
**   Library for accessing Kvaser Memorator (2nd generation)
**
** -----------------------------------------------------------------------------
*/

#ifndef KVAMEMOLIB_H
#define KVAMEMOLIB_H

//#define LOG_PERFORMANCE_TEST 1

#define ERROR_MESSAGE_LENGTH 2048

#include <inttypes.h>
#define WINAPI
typedef int         BOOL;
typedef void* MemoHandle;
typedef int8_t      int8;
typedef uint8_t    uint8;
typedef int16_t    int16;
typedef uint16_t  uint16;
typedef int32_t    int32;
typedef uint32_t  uint32;
typedef int64_t    int64;
typedef uint64_t   uint64;
#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef uint16 (WINAPI *MemoCallback)(int type, char *s);

typedef enum {
    MemoStatusOK                =  0,
    MemoStatusFail              = -1, // Generic error
    MemoStatusERR_PARAM         = -3,
    MemoStatusERR_LOGFILEOPEN   = -8, // can't find/open log file.
    MemoStatusERR_NOSTARTTIME   = -9,
    MemoStatusERR_NOLOGMSG      = -10,
    MemoStatusERR_LOGFILEWRITE  = -11,
    MemoStatusEOF               = -12,
    MemoStatusERR_NO_DISK       = -13,
    MemoStatusERR_LOGFILEREAD   = -14,

    MemoStatusERR_QUEUE_FULL          = -20,
    MemoStatusERR_CRC_ERROR           = -21,
    MemoStatusERR_SECTOR_ERASED       = -22,
    MemoStatusERR_FILE_ERROR          = -23,
    MemoStatusERR_DISK_ERROR          = -24,
    MemoStatusERR_DISKFULL_DIR        = -25,
    MemoStatusERR_DISKFULL_DATA       = -26,
    MemoStatusERR_SEQ_ERROR           = -27,
    MemoStatusERR_FILE_SYSTEM_CORRUPT = -28,
    MemoStatusERR_UNSUPPORTED_VERSION = -29,
    MemoStatusERR_NOT_IMPLEMENTED     = -30,
    MemoStatusERR_FATAL_ERROR         = -31,
    MemoStatusERR_ILLEGAL_REQUEST     = -32,
    MemoStatusERR_FILE_NOT_FOUND      = -33,
    MemoStatusERR_NOT_FORMATTED       = -34,
    MemoStatusERR_WRONG_DISK_TYPE     = -35,
    MemoStatusERR_TIMEOUT             = -36,
    MemoStatusERR_DEVICE_COMM_ERROR   = -37,
    MemoStatusERR_OCCUPIED            = -38,
    MemoStatusERR_USER_CANCEL         = -39,
    MemoStatusERR_FIRMWARE            = -40,
    MemoStatusERR_CONFIG_ERROR        = -41,
    MemoStatusERR_WRITE_PROT          = -42,

} KvaMemoStatus;

typedef enum {
    memoLOG_TYPE_INVALID    = -1,
    memoLOG_TYPE_CLOCK      =  0,
    memoLOG_TYPE_MSG        =  1,
    memoLOG_TYPE_TRIGGER    =  2
} MemoLogType;

// For memoLogEventEx
#define MEMOLOG_TYPE_INVALID    0
#define MEMOLOG_TYPE_CLOCK      1
#define MEMOLOG_TYPE_MSG        2
#define MEMOLOG_TYPE_TRIGGER    3
#define MEMOLOG_TYPE_VERSION    4



// Device type passed to memoOpen
typedef enum {
  memo_DISK = 0,  // Kvaser Memorator KMF file on a disk reader or local disk
  memo_MEMO = 1,  // Kvaser Memorator device
  memo_MEMO2 = 2, // Kvaser Memorator Professional device
  memo_DISK2 = 3, // Kvaser Memorator Professional KMF file on a disk reader or local disk
  memo_MEMO3 = 4, // Kvaser Memorator Professional device LOGIO3
  memo_DISK3 = 5, // Kvaser Memorator Professional KMF file on a disk reader or local disk LOGIO3
  memo_HYDRA = 6, // Kvaser MiniHydra
  memo_HDISK = 7, // Kvaser MiniHydra KMF file on a disk reader or local disk
  memo_MEMO5 = 8, // Kvaser Extended KMF format for Memorator 5
  memo_DISK5 = 9, // Kvaser Extended KMF file on a disk reader or local disk
} memoDevice;

typedef enum {
    memo_SWINFO_KVAMEMOLIB     = 1,
    memo_SWINFO_DRIVER         = 2,
    memo_SWINFO_FIRMWARE       = 3,
    memo_SWINFO_DRIVER_PRODUCT = 4,
    memo_SWINFO_CONFIG_VERSION_NEEDED = 5,
    memo_SWINFO_CPLD_VERSION   = 6,     // Kvaser Memorator Professional only
} memoVersionInfo;

// For memoFormatDiskEx()
#define MEMOLIB_FORMAT_PATH_DISKREADER    0x01    // Path is to a disk reader
#define MEMOLIB_FORMAT_PATH_DISKFILE      0x02    // Path is to a KMF file
#define MEMOLIB_FORMAT_FAT16              0x04    // Format with FAT16
#define MEMOLIB_FORMAT_FAT32              0x08    // Format with FAT32
#define MEMOLIB_FORMAT_RECREATE           0x10    // (Re)create KMF
#define MEMOLIB_FORMAT_CLEAR_DISK         0x20    // Remove all files
#define MEMOLIB_FORMAT_SIZE_IS_MB         0x40    // "max_space" and "reserve_space" are megabytes


#define MEMOVERSION_FLAG_BETA  0x01

#include <pshpack1.h>

typedef struct {
    uint32    disk_type;     // DISK_TYPE_xxx; Taken from the CSD register
    uint32    version;
    uint32    data_size;     // Device size in sectors of 512 bytes
    uint32    read_time;
    uint32    wr_factor;
    uint32    read_blk_size;
    uint32    wr_blk_size;
    uint32    trans_speed;
    uint32    file_format;
    uint32    erase_value;
    uint32    serial_number;
    uint32    date_code;
    uint32    m_id;
    uint32    oem_id;
    char      product_name[10];
    char      product_revision;
} diskInfo;


// Info about the file system; taken from the boot sector
typedef struct
{
    uint32 fat_size;          // Size of the FAT, in sectors
    uint32 fat_type;          // 12 or 16 depending on FAT type.
    uint32 dir_entries;       // Number of directory entries in the root dir
    uint32 cluster_size;      // Two-logarithm of the cluster size in sectors
    uint32 fat1_start;        // First FAT starts in this sector
    uint32 first_data_sector; // First sector available for data
    uint32 last_data_sector;  // Last sector available for data
} fileSysInfo;


typedef struct
{
  uint32 maxFileCount;
  uint32 fileCount;
  uint32 diskSize;
  uint32 usedDiskSize;
  uint32 padding[30];
} fileSysUsage;

#define MEMO_DISKFLG_IS_PRESENT           0x01
#define MEMO_DISKFLG_IS_WRITE_PROTECTED   0x02

#define MEMO_POWERFLG_BATTERY_FAULT       0x01
#define MEMO_POWERFLG_BATTERY_CHARGING    0x02
#define MEMO_POWERFLG_BATTERY_OK          0x04
#define MEMO_POWERFLG_EXTERNAL_POWER      0x08
#define MEMO_POWERFLG_USB_POWER           0x10
#define MEMO_POWERFLG_BATTERY_FAULT_NTC   0x20
#define MEMO_POWERFLG_BATTERY2_FAULT      0x40
#define MEMO_POWERFLG_BATTERY2_CHARGING   0x80
#define MEMO_POWERFLG_BATTERY2_OK         0x200
#define MEMO_POWERFLG_BATTERY2_FAULT_NTC  0x400

typedef struct
{
  uint32    diskFlags;        // Any combination of MEMO_DISKFLG_xxx
  uint32    powerFlags;       // Any combination of MEMO_POWERFLG_xxx
  int32     temperature;      // Temperature in Celsius (to convert to Farenheit, multiply with 9/5 and add 32)
  int32     vBattery;         // Battery voltage in mV
  uint32    reserved[16];
} MemoHardwareInfo;


// CAN channel settings bitrate etc.
typedef struct
{
    int32 channel;
    int32 PSC;
    int32 PrSeg;
    int32 PhSeg1;
    int32 PhSeg2;
    int32 SJW;
    int32 samples;
    int32 silent;
    int32 highSpeed;
} CANtiming;


typedef struct {
    int32   type;
    int32   preTrigger;
    int32   postTrigger;
    uint32  trigNo;      // Bitmask with the activated trigger(s)
    uint32  timeStamp;
} memoLogTrigger;


typedef struct {
  int32   type;
  int32   preTrigger;
  int32   postTrigger;
  uint32  trigNo;         // Bitmask with the activated trigger(s)
  int64   timeStamp;      // timestamp in units of 1 nanoseconds
} memoLogTriggerEx;

typedef struct
{
  uint32 lioMajor;     // Lio major version
  uint32 lioMinor;     // Lio minor version
  uint32 fwMajor;      // Firmware major version
  uint32 fwMinor;      // Firmware major version
  uint32 fwBuild;      // Firmware build version
  uint32 serialNumber; // Serial
  uint32 eanHi;        // EANHI
  uint32 eanLo;        // EANLO
} memoLogVersionEx;



#ifndef canMSG_RTR
#  define canMSG_RTR            0x0001      // Message is a remote request
#  define canMSG_STD            0x0002      // Message has a standard ID
#  define canMSG_EXT            0x0004      // Message has an extended ID
#  define canMSG_WAKEUP         0x0008      // Message was received in wakeup mode
#  define canMSG_NERR           0x0010      // NERR was active during the message
#  define canMSG_ERROR_FRAME    0x0020      // Message is an error frame
#  define canMSG_TXACK          0x0040      // Message is a TX ACK (msg is really sent)
#  define canMSG_TXRQ           0x0080      // Message is a TX REQUEST (msg is transfered to the chip)
#  define canMSGERR_OVERRUN     0x0600      // An overrun condition.
#  define canFDMSG_EDL          0x010000    // Obsolete, use MSGFLAG_FDF instead
#  define canFDMSG_FDF          0x010000    // Message is an FD message (CAN FD)
#  define canFDMSG_BRS          0x020000    // Message is sent/received with bit rate switch (CAN FD)
#  define canFDMSG_ESI          0x040000    // Sender of the message is in error passive mode (CAN FD)
#endif

// A CAN message
typedef struct {
    uint32 id;            // The identifier
    uint32 timeStamp;     // timestamp in units of 10 microseconds
    uint32 channel;       // The channel on which the message arrived, 0,1,...
    uint32 dlc;           // The length of the message
    uint32 flags;         // Message flags
    uint32 data[64];      // Message data (64 bytes)
} memoLogMsg;


typedef struct {
  uint32 id;            // The identifier
  int64  timeStamp;     // timestamp in units of 1 nanoseconds
  uint32 channel;       // The channel on which the message arrived, 0,1,...
  uint32 dlc;           // The data length of the message in bytes
  uint32 flags;         // Message flags
  uint8  data[64];      // Message data (64 bytes)
} memoLogMsgEx;


typedef struct {
  uint32 calendarTime;   // RTC date (unix format)
  uint32 timeStamp;      // timestamp in units of 1 nanoseconds
} memoLogRtcClock;


typedef struct {
  uint32 calendarTime;   // RTC date (unix format)
  int64  timeStamp;
} memoLogRtcClockEx;


typedef struct {
    MemoLogType     type;
    memoLogMsg      msg;
    memoLogRtcClock rtc;
    memoLogTrigger  trig;
} memoLogEvent;


typedef struct {
  uint32                type;   // MEMOLOG_TYPE_xxx
  union {
    memoLogMsgEx        msg;
    memoLogRtcClockEx   rtc;
    memoLogTriggerEx    trig;
    memoLogVersionEx    ver;
    uint8               raw[128];
  } x;
} memoLogEventEx;

#include <poppack.h>


/*
 * =============================================================================
 *   Memorator and disk functions
 * =============================================================================
 */

// Call this function to initialize the library.
void WINAPI memoInitializeLibrary(void);

KvaMemoStatus WINAPI memoProbeDeviceVersion(memoDevice mDevice,
                                            int32 memoNr,
                                            uint32 *firmwareVersion,
                                            uint32 *fileVersion,
                                            uint32 *configVersion);

KvaMemoStatus WINAPI memoProbeFileVersion(const char *fn,
                                          uint32 *fileVersion,
                                          uint32 *configVersion);

// Connect to a Memorator (or a KMF file resident on a hard disk)
// and obtain a handle for subsequent operations.
// The flags argument is currently unused.
MemoHandle    WINAPI memoOpen(KvaMemoStatus *stat, int32 memoNr,
                              int32 flags, memoDevice mDevice);

MemoHandle    WINAPI memoOpenDrv(KvaMemoStatus *stat, const char *drvName, int32 flags,
                           memoDevice mDevice);

// Close the connection to Memorator. The handle becomes invalid.
KvaMemoStatus WINAPI memoClose(MemoHandle h);

// Read file system info
KvaMemoStatus WINAPI memoReadFilesystemInfo(MemoHandle h,
                                            fileSysInfo * fSysInfo);

// Get disk usage statistics
KvaMemoStatus WINAPI memoGetFileSystemUsage(MemoHandle h, fileSysUsage *fus);

// Read disk info
KvaMemoStatus WINAPI memoReadDiskInfo(MemoHandle h, diskInfo * disk_info);

// Format disk file
KvaMemoStatus WINAPI memoFormatDisk(MemoHandle h, uint32 *format_stat);

KvaMemoStatus WINAPI memoFormatDiskEx(MemoHandle h, char *path,
                                      uint32 flags, uint32 reserve_space,
                                      uint32 max_space, uint32 max_files,
                                      uint32 *format_stat);

// Clear the logging data but keep everything else
KvaMemoStatus WINAPI memoClearDataDisk(MemoHandle h, uint32 *stat);

// Get date and time from the RTC chip.  The time is returned in
// standard unix format (number of seconds since 1-JAN-1970).
KvaMemoStatus WINAPI memoGetRTC(MemoHandle h, uint32 *t);

// Set date and time in the RTC. The time is returned in standard unix
// format (number of seconds since 1-JAN-1970).
KvaMemoStatus WINAPI memoSetRTC(MemoHandle h, uint32 t);

// Put the Memorator in configuration mode. Interval is in milliseconds.
KvaMemoStatus WINAPI memoConfigModeSetInterval(MemoHandle h, int interval);
KvaMemoStatus WINAPI memoConfigModeGetInterval(MemoHandle h, int * interval);
// Call this periodically to keep the device in configuration mode
KvaMemoStatus WINAPI memoConfigModeRefresh(MemoHandle h);

// Create a thread to periodically refresh the memorator.
KvaMemoStatus WINAPI memoConfigModeSetAutomatic(MemoHandle h, int onoff);

KvaMemoStatus WINAPI memoConfigModeGetDiskStatus(MemoHandle h, int *diskStat);
KvaMemoStatus WINAPI memoConfigModeGetMode(MemoHandle h, int *configMode);

KvaMemoStatus WINAPI memoFlashAllLeds(MemoHandle h);

KvaMemoStatus WINAPI memoGetHardwareInfo(MemoHandle h, MemoHardwareInfo *hinfo);


// Open the KMF file on a connected Memorator. Call closeDiskFile when
// done. The LIO version is returned.
KvaMemoStatus WINAPI memoOpenDisk(MemoHandle h, uint32 *fileVersion);

// Open a KMF file resident on a local disk. Call closeDiskFile when
// done.
KvaMemoStatus WINAPI memoOpenDiskFile(MemoHandle h, const char *fn,
                                      uint32 *fileVersion);

// Closes the currently open KMF file. The handle remains valid.
KvaMemoStatus WINAPI memoCloseDisk(MemoHandle h);

// Close and immediately open a KMF file (it autodetects if it's a
// memorator or if it's a KMF file. memoOpenDisk or memoOpenDiskFile
// must have been called earlier.
KvaMemoStatus WINAPI memoReopenDisk(MemoHandle h, uint32 *fileVersion);

// Check an open KMF file for errors. Set complete to 1 to check the
// complete file including empty blocks. Set fix to 1 to fix any errors
// (if possible).
KvaMemoStatus WINAPI memoValidateDisk(MemoHandle h, uint32 complete,
                                      uint32 fix);

// Returns the number of log files (KME files) inside the presently
// open KMF file.
KvaMemoStatus WINAPI memoGetLogFileCountEx(MemoHandle h, uint32* fileCount);


// Low-level disk I/O routines.
KvaMemoStatus WINAPI memoDiskReadPhysicalSector(MemoHandle h, uint32 sectorNo,
                                                void* buf, size_t bufsize);

KvaMemoStatus WINAPI memoDiskReadLogicalSector(MemoHandle h, uint32 sectorNo,
                                               void* buf, size_t bufsize);

KvaMemoStatus WINAPI memoDiskWritePhysicalSector(MemoHandle h, uint32 sectorNo,
                                                 void* buf, size_t bufsize);

KvaMemoStatus WINAPI memoDiskWriteLogicalSector(MemoHandle h, uint32 sectorNo,
                                                void* buf, size_t bufsize);

KvaMemoStatus WINAPI memoDiskErasePhysicalSector(MemoHandle h, uint32 sectorNo,
                                                 uint32 count);

KvaMemoStatus WINAPI memoDiskEraseLogicalSector(MemoHandle h, uint32 sectorNo,
                                                uint32 count);

KvaMemoStatus WINAPI memoSetCallbackEx(MemoHandle h, MemoCallback f);

// Get software version information.
KvaMemoStatus WINAPI memoGetSoftwareVersionInfo(MemoHandle h,
                                                memoVersionInfo itemCode,
                                                unsigned int *major,
                                                unsigned int *minor,
                                                unsigned int *build,
                                                unsigned int *flags);

// Get serial number
KvaMemoStatus WINAPI memoGetSerialNumber(MemoHandle h, unsigned int *serial);

// Open the log file (KME file) with the specified index inside the
// presently open KMF file. The approximate number of events in the log
// file is returned.
KvaMemoStatus WINAPI memoLogOpenFile(MemoHandle h, uint32 fileIndx,
                                     uint64 *eventCount);

// CLose the presently open log (KME) file. The handle will stay valid.
void          WINAPI memoLogCloseFile(MemoHandle h);

// Get the time of the first event in the log file. The time is
// returned in standard unix format (number of seconds since
// 1-JAN-1970).
KvaMemoStatus WINAPI memoLogGetStartTime(MemoHandle h, uint32 *start_time);

// Get the time of the last event in the log file. The time is
// returned in standard unix format (number of seconds since
// 1-JAN-1970).
KvaMemoStatus WINAPI memoLogGetEndTime(MemoHandle h, uint32 *end_time);

// Read an event from the log file.
KvaMemoStatus WINAPI memoLogReadEventEx(MemoHandle h, memoLogEventEx *e);

// Read serial number from the log file.
KvaMemoStatus WINAPI memoLogGetSerial(MemoHandle h, uint32 *serialNumber);

// Read EAN number from the log file.
KvaMemoStatus WINAPI memoLogGetEan(MemoHandle h, uint32 *eanHi, uint32 *eanLo);

// Read configuration data from a Memorator.
KvaMemoStatus WINAPI memoReadConfig(MemoHandle h, void *buf,
                                    size_t buflen, size_t *actual_len);

// Write configuration data to a Memorator.
KvaMemoStatus WINAPI memoWriteConfig(MemoHandle h, void *buf, size_t buflen);

// Read configuration data from an SD card.
KvaMemoStatus WINAPI memoReadConfigSD(const MemoHandle h, const char *const dstFilename);

// Write configuration data to an SD card.
KvaMemoStatus WINAPI memoWriteConfigSD(const MemoHandle h, const char *const srcFilename);



//-----------------------------------------------------------------------
//
// New API
//
//-----------------------------------------------------------------------
void WINAPI kvmInitializeLibrary(void);

//-----------------------------------------------------------------------
//
// Memorator Light
//
//-----------------------------------------------------------------------

// Partition the disk into several tracks; any data on the disk is lost.
KvaMemoStatus WINAPI memoPartitionDisk(MemoHandle h, char *path, int tracks);

// Get the number of tracks on a disk
KvaMemoStatus WINAPI  memoGetTrackCount(MemoHandle h, int* tracks);

// Return the usage for a track
KvaMemoStatus WINAPI memoGetFileTrackUsage(MemoHandle h, int track, fileSysUsage *fus);

// Clear data from a track
KvaMemoStatus WINAPI memoClearDataTrack(MemoHandle h, int track);

// Get number of files on a track
KvaMemoStatus WINAPI memoGetLogFileCountTrack(MemoHandle h, int track, uint32* fileCount);


// -----------------------------------------------------------------------
// The following functions are obsolete and will not be implemented nor
// maintained in the future. Do not use them.
// -----------------------------------------------------------------------
uint32        WINAPI memoGetLogFileCount(MemoHandle h);


//
KvaMemoStatus WINAPI memoGetDbaseFile(MemoHandle h,   int filenumber, char *path, char *dbname);
KvaMemoStatus WINAPI memoPutDbaseFile(MemoHandle h,   int filenumber, char *filename);
KvaMemoStatus WINAPI memoEraseDbaseFile(MemoHandle h, int filenumber);

// Old obsolete callback type.
typedef void (*memolib_callback_type)(int);
KvaMemoStatus WINAPI memoSetCallback(MemoHandle h, memolib_callback_type f);


#ifdef __cplusplus
}
#endif

#endif //KVAMEMOLIB_H
