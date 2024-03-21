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

#include "kvaConverter.h"
#include "KvaLogReader.h"
#include "kvdebug.h"
#include "os_util.h"

KvaLogReader::~KvaLogReader()
{
  PRINTF(("KvaLogReader::~KvaLogReader()\n"));
}

KvlcStatus KvaLogReader::interpret_event(void* event, imLogData* logEvent){
  (void)event;
  (void)logEvent;
  PRINTF(("default implementation of interpret_event\n"));
  return kvlcERR_NOT_IMPLEMENTED; 
}

// Move to os_util.h in the future
int64_t KvaLogReader::get_file_size(const char *filename)
{
  struct stat file;
  if (!filename) {
    return 0;
  }
  if (!stat(filename,&file)) {
    return file.st_size;
  }
  return 0;
}


KvlcStatus KvaLogReader::open_file(const char *filename)
{
  if (isOpened) {
    PRINTF(("KvaLogReader::open_file, a file is already open\n"));
    return kvlcERR_FILE_ERROR;
  }

  //infile = fopen(filename, isBinary() ? "rb" : "r");
  infile = utf_fopen(filename, (isBinary() ? "rb" : "r"));
  if (!infile) {
    PRINTF(("KvaLogReader::open_file, could not open file %s\n", filename));
    return kvlcERR_FILE_ERROR;
  }
  isOpened = true;

  file_size = get_file_size(filename);

  file_position = 0;
  PRINTF(("KvaLogReader::open_file, open_file %s seems to have worked ok\n", filename));
  return kvlcOK;
}

KvlcStatus KvaLogReader::close_file()
{
  if (!isOpened) {
    PRINTF(("KvaLogReader::close_file, file not open\n"));
  }
  if (infile) {
    fclose(infile);
    infile = NULL;
  }
  isOpened = false;
  return kvlcOK;
}

KvlcStatus KvaLogReader::read_line(char *string, int num)
{
  fpos_t position;

  if (!isOpened) {
    PRINTF(("KvaLogReader::read_line, file not open\n"));
    return kvlcERR_FILE_ERROR;
  }

  if (fgets (string, num, infile )) {
    // Ignore return value
  }

  if (ferror(infile)) {
    PRINTF(("KvaLogReader::read_line, ferror\n"));
    return kvlcERR_FILE_ERROR;
  }
  if (feof(infile)) {
    PRINTF(("KvaLogReader::read_line, feof\n"));
    return kvlcEOF;
  }

  // Unportable, but should be OK on Win32 (fpos_t is really opaque).
  // (No _ftelli64 in VS6 or .NET.)
  os_fgetpos(infile, &position);

// Move to os_util.h in the future
  file_position = position.__pos;

  return kvlcOK;
}

// skip ahead filepos by num bytes
KvlcStatus KvaLogReader::move_fpos(size_t num) {
  fpos_t pos;

  if (os_fgetpos(infile, &pos)){
    return kvlcFail;
  }

  pos.__pos += num;

  if (os_fsetpos(infile, &pos)){
    return kvlcFail;
  }

  return kvlcOK;
}

KvlcStatus KvaLogReader::read_file(char *string, size_t num)
{
  fpos_t position;

  if (!isOpened) {
    PRINTF(("KvaLogReader::read_file(), file not open\n"));
    return kvlcERR_FILE_ERROR;
  }

  if (fread (string , num , 1, infile)) {
    // Ignore return value
  }

  if (ferror(infile)) {
    PRINTF(("KvaLogReader::read_file, ferror\n"));
    return kvlcERR_FILE_ERROR;
  }
  if (feof(infile)) {
    PRINTF(("KvaLogReader::read_file, feof\n"));
    return kvlcEOF;
  }

  // Unportable, but should be OK on Win32 (fpos_t is really opaque).
  // (No _ftelli64 in VS6 or .NET.)
  os_fgetpos(infile, &position);
  // Move to os_util.h in the future

  file_position = position.__pos;
  return kvlcOK;
}

KvaLogReader::KvaLogReader()
{
  PRINTF(("KvaLogReader::KvaLogReader()\n"));
  start_of_measurement64 = 0;
  last_clock_event = 0;
  first_timestamp = 0;
  isOpened = false;
  infile = NULL;
}

KvlcStatus KvaLogReader::next_file()
{
  PRINTF(("KvaLogReader::next_file()\n"));
  return kvlcOK;
}
