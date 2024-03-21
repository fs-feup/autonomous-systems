/*
**             Copyright 2023 by Kvaser AB, Molndal, Sweden
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

#ifndef KCAN_IOCTL_FLASH_H
#define KCAN_IOCTL_FLASH_H
#ifdef __KERNEL__
#include <linux/types.h>
#else
#include <stdint.h>
#endif /* __KERNEL__ */

#define KCAN_FLASH_DOWNLOAD_CHUNK 24

#define FIRMWARE_DOWNLOAD_COMMIT  1
#define FIRMWARE_DOWNLOAD_WRITE   2
#define FIRMWARE_DOWNLOAD_FINISH  3
#define FIRMWARE_DOWNLOAD_STARTUP 4
#define FIRMWARE_DOWNLOAD_ERASE   5

#define FIRMWARE_STATUS_OK           0 // OK
#define FIRMWARE_ERROR_NOPRIV        1 // No privilegie
#define FIRMWARE_ERROR_ADDR          2 // Wrong address
#define FIRMWARE_ERROR_FLASH_FAILED  3 // Flash device failed
#define FIRMWARE_ERROR_COMMAND       4 // Unknown command
#define FIRMWARE_ERROR_PARAM_FULL    5 // Parameter memory full
#define FIRMWARE_ERROR_PWD_WRONG     6
#define FIRMWARE_ERROR_VERSION_WRONG 7 // New fw version not accepted
#define FIRMWARE_ERROR_BAD_CRC       8
#define FIRMWARE_ERROR_INVALID_ARG   9 // Invalid argument
#define FIRMWARE_ERROR_VERIFY_DIFF   10 // Comparison differs

typedef struct {
    int status;
    int tag; // FIRMWARE_DOWNLOAD_xxx
    union {
        struct {
            uint8_t ver_major;
            uint8_t ver_minor;
            uint16_t ver_build;
            uint32_t ean[2];
            uint8_t dryrun;
            uint8_t flash_procedure_version;
            uint16_t buffer_size;
        } setup;
        struct {
            uint32_t address;
            uint32_t len;
            uint8_t data[KCAN_FLASH_DOWNLOAD_CHUNK];
        } data;
        struct {
            uint32_t area;
        } erase;
    } x;
} KCAN_FLASH_PROG;
#endif /* KCAN_IOCTL_FLASH_H */
