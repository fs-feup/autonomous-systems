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
**  Functions to access sectors of a volume, low-level partition tables, etc.
**  Functions to access the physical as well as the logical disk.
**  Function to post sectors to the large buffer in the write task
** -----------------------------------------------------------------------------
*/

#ifndef DISKIO_H
#define DISKIO_H
#include <stdint.h>
#include "kvdebug.h"

typedef enum {
  dio_OK = 0,
  dio_CRCError,         // 1: Our own checksum failed. May be OK for some operations.
  dio_SectorErased,     // 2: The sector is empty. Probably OK at all times.
  dio_Pending,          // 3:
  dio_IllegalRequest,   // 4:
  dio_QueueFull,        // 5:
  dio_FileNotFound,     // 6:
  dio_FileError,        // 7:
  dio_NotImplemented,   // 8:
  dio_Custom1,          // 9: Just if another package wants to store someting
                        // in a status variable. This value is
                        // (normally) not assigned by a diskio call.
  dio_NotFormatted,     // 10:
  dio_Timeout,          // 11:
  dio_WrongDiskType,    // 12:
  dio_DiskError,        // 13: The disk reported an error.
  dio_DiskCommError,    // 14: A disk communication problem was detected.
  dio_NoDisk,           // 15: The disk is missing
  dio_NoMemory,         // 16: Memory allocation failed; or other resource shortage
  dio_UserCancel,       // 17: User cancelled the operation
  dio_CpldError,        // 18:
  dio_ConfigError,      // 19: Some type of configuration error (e.g. corrupt config file)
  dio_DiskIdle,         // 20: The R1 response says the disk is in idle state.
  dio_EOF,              // 21: End Of File while reading.
  dio_SPI               // 22: SPI bus in use by another module; wait
} DioResult;

typedef enum { // Commands to message buffer
  dio_Write = 0,
  dio_Flush
} DioCommand;

/* Definitions used for the SD Cards
* ----------------------------------
* In our code, we call 512 bytes a sector, and we erase at least this many bytes
* at a time, and writes them all together. Reading is usually also done for
* whole sectors, but if performance gains from smaller chunks, we might implement
* that.
*/
#define DIO_SECTOR_SIZE                   512

/* The size of a single KMF data file (used by logio and logio)
* 1024*2048 = 2097152 1GB file */
#define LIO_FILESIZE 2097152

typedef struct
{
  // Info about the file system; taken from the boot sector.
  // If fat_type == 0, then the info is not valid.
  uint16_t fat_size;              // Size of the FAT, in sectors
  uint16_t fat_type;              // 16 (or 32) depending on FAT type.
  uint16_t dir_entries;           // Number of directory entries in the root dir
  uint16_t cluster_size;          // Two-logarithm of the cluster size in sectors
  uint32_t fat1_start;            // First FAT starts in this sector
  uint32_t first_data_sector;     // First sector available for data
  uint32_t last_data_sector;      // Last sector available for data
  uint32_t dir_start;             // First sector of the root directory

  // Info about the "interesting" files on the disk.
  uint32_t param_start_sector;
  uint32_t param_end_sector;
  uint32_t logdata_start_sector;
  uint32_t logdata_end_sector;
  uint32_t dbase_start_sector;
  uint32_t dbase_end_sector;

} DioFileSystemInfo;


#define DISK_TYPE_INVALID   0
#define DISK_TYPE_MMC       1
#define DISK_TYPE_SD        2
#define DISK_TYPE_SDHC      3

typedef struct {
  uint8_t         disk_type;  // DISK_TYPE_xxx
  // The following are taken from the CSD register
  uint8_t         version;
  uint8_t         read_time;
  uint8_t         wr_factor;
  uint32_t        data_size;  // Device size in sectors of 512 bytes
  uint16_t        read_blk_size;
  uint16_t        wr_blk_size;
  uint16_t        trans_speed;
  uint8_t         file_format;
  // The following are taken from the SCR register.
  // The value read from erased sectors. Manufacturer dependent, can be
  // either  0x00 or 0xff.
  uint8_t         erase_value;
  // The following are taken from the CID register.
  uint32_t        m_id;
  uint8_t         oem_id[2];
  char            product_name[10];
  uint32_t        serial_number;
  // We code the date code like this: upper byte year (offset from
  // 1997) and lower byte month.
  uint16_t        date_code;
  uint8_t         product_revision;
  uint8_t         _padding;
} DioDiskInfo;

extern DioFileSystemInfo    gFSInfo;
extern DioDiskInfo          gDiskInfo;

typedef struct {
  uint8_t   data[DIO_SECTOR_SIZE-2];
  uint16_t  crc;       // Crc maintained by us
  uint16_t  xtraCrc;   // Crc generated by the protocol talking to the memory card.
} DioSector;

//CompilerAssert(sizeof(DioSector) == DIO_SECTOR_SIZE + 2);

#ifdef __cplusplus
extern "C" {
#endif

// Initialize the diskio.c module
// Note: in win32 mode, call dioOpenDiskImage() first.
DioResult dioInit (void);

// Obtain disk information and place it in certain global variables
DioResult dioGetDeviceInfo(void);

/* Low level disk access.
* Functions containing 'physical' in the name operates on the physical disk.
* The application will normally operate on a large file created on a formatted
* disk at startup which allows us to use the same logic when reading a file via
* a card reader or a complete file copied to the hard disk.
*
* Routines ending in '_nowait' are asynchronous. Upon a successfull call, *iosb
* is set to dio_Pending and dio_OK is returned.
' When the request is completed, *iosb is set to dio_OK or possibly another
* value in case of an error. There is room for a limited number of requests.
* When writing, the buffer must remain intact until the operation is completed
* (i.e., no copying is done).
*
*/

DioResult dioFlushBuffer(void *lioSector);
int dioIsFlushInProgress(void);


// Non-blocking function used by logio
DioResult dioPostSector(void *lioSector);

// Blocking functions
DioResult dioReadSectorWait(DioSector *buffer, uint32_t sector);
DioResult dioReadPhysicalSectorWait(void *buffer, uint32_t sector);
DioResult dioReadSectorsWait(DioSector *buffer, uint32_t sector, uint16_t count);
DioResult dioReadPhysicalSectorsWait(void *buffer, uint32_t sector, uint16_t count);
DioResult dioWriteSectorWait(DioSector *buffer, uint32_t sector);
DioResult dioWritePhysicalSectorWait(void *buffer, uint32_t sector);

DioResult dioWaitUntilQueueEmpty (void);

// Format the disk and allocate the log files.
DioResult dioFormatDisk(uint32_t reserveSpace, uint32_t dbaseSpace, uint8_t fat32);

// Suspend further disk I/O by turning the disk off.
// Call dioInit() and dioGetDeviceInfo() to get it back.
void dioSuspendDisk(void);

// Print some info
void dioDumpDiskInfo (void);
void dioDumpFilesystemInfo (void);

#define CRC_INIT 0xffff
typedef uint16_t Crc;
Crc crcAdd(void *buf, Crc crc, int32_t n);

/* Adds the mount point path to filename before open/unlink */
DioResult dioOpen (const char* file, int flags, int mode, int *fd);
DioResult dioUnlink (const char* file);

#ifdef __cplusplus
}
#endif

#endif // DISKIO_H
