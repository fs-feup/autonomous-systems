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
** Description: OS dependant utility functions for WINDOWS and LINUX
** -----------------------------------------------------------------------------
*/

#ifndef COMMON_OS_UTIL_H
#define COMMON_OS_UTIL_H

#include<string>
#include <inttypes.h>
typedef int IoctlHandle;
typedef int LockHandle;
typedef void* InstanceHandle;
typedef void* InstanceAdress;
#define INVALID_LOCK_HANDLE -1
#define INVALID_IOCTL_HANDLE -1


//---------------------------------------------------------------------------
// String functions
//---------------------------------------------------------------------------
bool os_splitpath(const std::string path,
     std::string& drive, std::string& dir,
     std::string& base, std::string& ext);

bool os_makepath(std::string& path,
     const std::string& drive, const std::string& dir,
     const std::string& base, const std::string& ext);


std::string os_filesep();


//---------------------------------------------------------------------------
// IOCTL
//---------------------------------------------------------------------------
IoctlHandle os_open_device(int card);

void os_close_device(IoctlHandle hnd);

int os_ioctl(IoctlHandle hnd, unsigned int code, void * buf, size_t len);


//---------------------------------------------------------------------------
// Locks
//---------------------------------------------------------------------------
LockHandle os_create_named_lock(const char * name);

void os_destroy_named_lock(LockHandle lock);

void os_sleep(unsigned int ms);


//---------------------------------------------------------------------------
// Files
//---------------------------------------------------------------------------
bool os_get_file_size(FILE *stream, uint64_t *size);

int64_t os_ftell(FILE *stream);

int os_fseek(FILE *stream, int64_t offset, int whence);

FILE* os_open_tmp_file(char* prefix, char** filename);

int os_mkdir(const char *path);

void os_rewind(FILE *stream);

int os_fsetpos(FILE *stream, const fpos_t *pos);

int os_fgetpos(FILE *stream, fpos_t *pos);
//---------------------------------------------------------------------------
// Libraries
//---------------------------------------------------------------------------
InstanceHandle os_load_library(const std::string &name);

void os_unload_library(InstanceHandle hnd);

InstanceAdress os_get_symbol_adress(InstanceHandle hnd, const char *symbol);

#endif
