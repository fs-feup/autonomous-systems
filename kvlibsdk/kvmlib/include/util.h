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
**   Utility functions for memolib
** -----------------------------------------------------------------------------
*/

#ifndef UTIL_H_
#define UTIL_H_
#include <stdint.h>

#define CONFIG_FILE_NAME    "PARAM.LIF"
#define LOGDATA_FILE_NAME   "LOGDAT00.KMF"
#define LOGDATA_FILE_NAME2  "LOG00000.KMF"
#define DATABASE_FILE_NAME  "DATABASE.BIN"
#define SD_DISK_SECTOR_SIZE DIO_SECTOR_SIZE
#define LIO_FIRST_AVAILABLE_SECTOR 8
#define LIO_HOUSEKEEPING_SECTOR 0x2000
#define MAX_NO_CHANNELS   5

#define ONE_BILLION 1000000000ULL

uint8_t dlcToBytesFD(uint8_t dlc);

 //===========================================================================
//  Note: There is no way to ask firmware about the lio (KMF) and config
//  (param.lif) version that firmware needs. It not possible to get the config
//  version from a KMF file either. This forces us to infer (guess) which
//  version we need and this is done in the following  functions to at least
//  keep the ugliness contained. This should be done in firmware!

void lioVerToConfigVer (uint8_t lioMajor, uint8_t lioMinor,
                               uint8_t *configMajor, uint8_t *configMinor);

void fwVerToConfigVer (uint8_t fwMajor, uint8_t fwMinor,
                              uint8_t *lioMajor, uint8_t *lioMinor,
                              uint8_t *configMajor, uint8_t *configMinor);

uint16_t toVersion (uint8_t major, uint8_t minor);

void splitVersion (uint16_t version, uint8_t *major, uint8_t *minor);

#endif
