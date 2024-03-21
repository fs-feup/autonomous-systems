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
** Description: File utility functions for LINUX
** ---------------------------------------------------------------------------
*/

#include "os_util.h"
#include "vcan_ioctl.h"
#include <libgen.h>
#include <cstring>
#include <cstdlib>
#include <cstdio>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/file.h>
#include <errno.h>
#include <time.h>
#include <dlfcn.h>
#include <algorithm>
//===========================================================================
//                           L I N U X
//===========================================================================


//---------------------------------------------------------------------------
// String functions
//---------------------------------------------------------------------------

//===========================================================================
bool os_splitpath(const std::string path,
       std::string& drive, std::string& dir,
       std::string& base, std::string& ext)
{
  // Copy the string since dirname() and basename() modifies string
  char *cPath = strdup(path.c_str());
  char *cBase = strdup(path.c_str());
  if (!cPath || !cBase) {
    if (cPath) free(cPath);
    if (cBase) free(cBase);
    return false;
  }

  // Split path
  char *pPath = dirname(cPath);
  char *pBase = basename(cBase);
  if (!pPath || !pBase) {
    if (cPath) free(cPath);
    if (cBase) free(cBase);
    return false;
  }

  // Get extension
  dir = std::string(pPath);
  std::string fname(pBase);
  std::string::size_type idx;
  idx = fname.rfind('.');
  if(idx != std::string::npos) {
    ext = fname.substr(idx+1);
    base = fname.substr(0, idx);
  } else {
    ext = "";
    base = fname;
  }
  drive = "";

  if (cPath) free(cPath);
  if (cBase) free(cBase);
  return true;

}

//===========================================================================
bool os_makepath(std::string& path,
    const std::string& drive, const std::string& dir,
    const std::string& base, const std::string& ext)
{
  if (drive.size()) {
    path = drive;
  }

  if (dir.size()) {
    if (path.size()) {
      path += os_filesep() + dir;
    } else {
      path = dir;
    }
  }

  if (base.size()) {
    if (path.size()) {
      path += os_filesep() + base;
    } else {
      path = base;
    }
  } else {
    if (path.size()) {
      path += os_filesep();
  }
  }


  if (ext.size()) {
    if (path.size()) {
      path += std::string(".") + ext;
    }
  }

  return true;
}

//===========================================================================
std::string os_filesep()
{
  return std::string("/");
}

//---------------------------------------------------------------------------
// IOCTL
//---------------------------------------------------------------------------

//===========================================================================
int os_ioctl(IoctlHandle hnd, unsigned int code, void * buf, size_t /* len */)
{
  return !ioctl(hnd, code, buf);
}

//===========================================================================
IoctlHandle os_open_device(int card)
{
  IoctlHandle hnd = INVALID_IOCTL_HANDLE;
  char drv_name [100] = {0};
  const char *driver_base = "mhydra";
  int count = 1;
  uint32_t dev_card_number;
  int dev_no = 0;

  while (hnd == INVALID_IOCTL_HANDLE) {

    snprintf(drv_name, sizeof(drv_name), "/dev/%s%d", driver_base, dev_no);

    while (hnd == INVALID_IOCTL_HANDLE && count < 10) {
      hnd = open(drv_name, O_RDONLY);
      if (hnd != -1)
        break;
      count++;
    }

    if (hnd == INVALID_IOCTL_HANDLE) {
      break;
    }

    if (os_ioctl(hnd, VCAN_IOC_GET_CARD_NUMBER, &dev_card_number, sizeof(dev_card_number))) {
      if (card == (int) dev_card_number) {
        break;
      }
    }

    close(hnd);
    hnd = INVALID_IOCTL_HANDLE;
    dev_no++;
  }

  return hnd;
}

//===========================================================================
void os_close_device(IoctlHandle hnd)
{
  if (hnd != INVALID_IOCTL_HANDLE) {
    close(hnd);
  }
}

//---------------------------------------------------------------------------
// Locks
//---------------------------------------------------------------------------
static void debug_named_lock(int code)
{
  switch (code) {
  case EWOULDBLOCK:
    // Expected result; for locked library
    // printf("The file is locked and the LOCK_NB flag was selected.\n");
      break;

  case EBADF:
    printf("Argument is not an open file descriptor.\n");
      break;

  case EINTR:
    printf("While waiting to acquire a lock, the call was interrupted by delivery of a signal.\n");
      break;

  case EINVAL:
    printf("Operation is invalid.\n");
      break;

  case ENOLCK:
    printf("The kernel ran out of memory for allocating lock records.\n");
      break;

  default:
    printf("Unknown errno %d\n", code);
      break;
  }
}


//===========================================================================
LockHandle os_create_named_lock(const char * name)
{
  mode_t oldmask = umask(0);
  std::string str = std::string(P_tmpdir) + os_filesep() + std::string(name) + std::string(".pid");
  LockHandle fd = open(str.c_str(), O_CREAT | O_EXCL | O_RDWR, 0666);
  umask(oldmask);

  if (fd < 0) {
    if (errno != EEXIST) {
      printf("ERROR: Could not create lock file '%s'. Errno: %d\n", str.c_str(), errno);
      return INVALID_LOCK_HANDLE;
    }

    // The file already exists; open it
    fd = open(str.c_str(), O_RDWR);
    if (fd < 0) {
      printf("ERROR: Could not open lock file '%s'. Errno: %d\n", str.c_str(), errno);
      return INVALID_LOCK_HANDLE;
    }
  }


  if (flock(fd, LOCK_EX | LOCK_NB)) {
    debug_named_lock(errno);
    return INVALID_LOCK_HANDLE;
  }

  return fd;
}


//===========================================================================
void os_destroy_named_lock(LockHandle lock)
{
  if (lock != INVALID_LOCK_HANDLE) {
    if ( flock(lock, LOCK_UN) ) {
      debug_named_lock(errno);
    }
    close(lock);
  }
}

//===========================================================================
void os_sleep(unsigned int ms)
{
  struct timespec req;
  struct timespec rem;

  req.tv_sec = ms/1000;
  req.tv_nsec = (ms - req.tv_sec*1000)*1000000;

  if (nanosleep(&req, &rem)) {
    // Ignore result
  }
}

//---------------------------------------------------------------------------
// Files; use CFLAGS  += -D_FILE_OFFSET_BITS=64
//---------------------------------------------------------------------------

//===========================================================================
bool os_get_file_size(FILE *stream, uint64_t *size)
{
  if (!stream || !size) return false;

  int fd = fileno(stream);
  if (fd < 0) return false;

  struct stat fileStat;
  if (fstat(fd, &fileStat) < 0) {
    return false;
  }
  *size = fileStat.st_size;
  return true;
}

//===========================================================================
int64_t os_ftell(FILE *stream)
{
  return ftello(stream);
}

//===========================================================================
int os_fseek(FILE *stream, int64_t offset, int whence)
{
  return fseeko(stream, offset, whence);
}

//===========================================================================
// Opens a unique temporary file. The file will be automatically deleted
// when it is closed or the program terminates.
FILE* os_open_tmp_file(char* /* prefix */, char **filename)
{
  *filename = NULL;
  return tmpfile();
}
//===========================================================================
// Note: File permission bits of the mode argument is modified by the
//       process' file creation mask.
int os_mkdir(const char *path)
{
  return mkdir(path, 0777);
}

// ===========================================================================
void os_rewind(FILE *stream)
{
  rewind (stream);
}

// ===========================================================================
int os_fsetpos(FILE *stream, const fpos_t *pos)
{
  return fsetpos (stream, pos);
}

// ===========================================================================
int os_fgetpos(FILE *stream, fpos_t *pos)
{
  return fgetpos (stream, pos);
}


//---------------------------------------------------------------------------
// Libraries
//---------------------------------------------------------------------------
//===========================================================================
InstanceHandle os_load_library(const std::string &name)
{
  std::string fullname = "lib" + name + ".so";
  return dlopen (fullname.c_str(), RTLD_LAZY|RTLD_DEEPBIND);
}

//===========================================================================
void os_unload_library(InstanceHandle hnd)
{
  dlclose(hnd);
}

//===========================================================================
InstanceAdress os_get_symbol_adress(InstanceHandle hnd, const char *symbol)
{
  return dlsym(hnd, symbol);
}
