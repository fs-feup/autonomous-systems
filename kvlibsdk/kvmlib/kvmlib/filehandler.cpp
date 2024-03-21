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
**  Read, write and convert KME files.
** -----------------------------------------------------------------------------
*/

#include "filehandler.h"
#include "KvaLogReader_Kme24.h"
#include "KvaLogReader_Kme25.h"
#include "KvaLogReader_Kme40.h"
#include "KvaLogReader_Kme50.h"
#include "KvaLogReader_Kme60.h"

#include "KvaLogWriter.h"
#include "KvaLogWriter_Kme24.h"
#include "KvaLogWriter_Kme25.h"
#include "KvaLogWriter_Kme40.h"
#include "KvaLogWriter_Kme50.h"
#include "KvaLogWriter_Kme60.h"
#include "KvaLogWriter_PlainAsc.h"
#include "KvaLogReader_memoLogEvent.h"
#include "kvdebug.h"
class fileHandler;

static const int OVERWRITE_EXISTING_FILE = 1;
static const size_t MAX_FILENAME_LENGTH = 2048;
static const size_t MAX_OPEN_HANDLES = 1024;
static const bool DEBUG_MODE = false;
static char DEBUG_FILENAME[] = "debug.txt";
static const FileHandle FREE_HANDLE = 0;

static FileHandle OpenFileHandles[MAX_OPEN_HANDLES] = {FREE_HANDLE};

// ===========================================================================
// Local functions
// ===========================================================================
static bool isInvalid (FileHandle h)
{
  if ( 0 == h || INVALID_FILE_HANDLE_VALUE == h ) return true;

  for (size_t k = 0; k < MAX_OPEN_HANDLES; k++) {
    if ( h == OpenFileHandles[k] ) {
      return false;
    }
  }
  return true;
}

// ===========================================================================
static bool fileHandleAvailable (void)
{
  for (size_t k = 0; k < MAX_OPEN_HANDLES; k++) {
    if ( FREE_HANDLE == OpenFileHandles[k] ) {
      return true;
    }
  }
  return false;
}

// ===========================================================================
static void setInUse (fileHandler *fh)
{
  FileHandle h = (FileHandle) fh;
  for (size_t k = 0; k < MAX_OPEN_HANDLES; k++) {
    if ( FREE_HANDLE == OpenFileHandles[k] ) {
      OpenFileHandles[k] = h;
      return;
    }
  }
}

// ===========================================================================
static void clearInUse (fileHandler *fh)
{
  FileHandle h = (FileHandle) fh;
  for (size_t k = 0; k < MAX_OPEN_HANDLES; k++) {
    if ( h == OpenFileHandles[k] ) {
      OpenFileHandles[k] = FREE_HANDLE;
      return;
    }
  }
}

// ===========================================================================
static bool isTooLong(const char *filename)
{
  return (strnlen(filename, MAX_FILENAME_LENGTH) >= MAX_FILENAME_LENGTH);
}


// ===========================================================================
// Class fileHandler
// ===========================================================================
class fileHandler
{
  private:
    FileType      type;
    char          filename[MAX_FILENAME_LENGTH];
    unsigned int  count;
    uint64_t lastTimeStamp;
    KvaLogReader  *reader;
    KvaLogReader  *converter;
    KvaLogWriter  *writer;
    KvaLogWriter  *dbwriter;

  protected:

  public:
    fileHandler(const char *fname);
    ~fileHandler();

    FileStatus openRead (FileType ftype);
    FileStatus openWrite (FileType ftype);
    FileStatus readEvent (void *e);
    FileStatus writeEvent (void *e);
    FileStatus eventCount (uint64_t *cnt);
    FileStatus close (void);
    void toMemoEventEx (imLogData *le, void *e);
};

// ===========================================================================
fileHandler::fileHandler(const char *fname)
{
  unsigned int mask = 0xFF;
  count = 0;
  lastTimeStamp = 0;
  memset(filename, 0, sizeof(filename));
  strncpy(filename, fname, MAX_FILENAME_LENGTH - 1);
  reader = NULL;
  writer = NULL;
  converter = new KvaLogReader_memoLogEvent();
  dbwriter = NULL;

  if (DEBUG_MODE) {
    dbwriter = new KvaLogWriter_PlainAsc();
    if (dbwriter) {
      int on = 1;
      dbwriter->set_filename(DEBUG_FILENAME);
      dbwriter->set_property_value(KVLC_PROPERTY_OVERWRITE, &on, sizeof(on));
      dbwriter->set_property_value(KVLC_PROPERTY_CHANNEL_MASK, &mask, sizeof(mask));
      dbwriter->set_property_value(KVLC_PROPERTY_START_OF_MEASUREMENT, &on, sizeof(on));
      dbwriter->set_property_value(KVLC_PROPERTY_CALENDAR_TIME_STAMPS, &on, sizeof(on));
      dbwriter->open_file();
      dbwriter->write_header();
    }
  }
  setInUse(this);
}

// ===========================================================================
fileHandler::~fileHandler()
{
  count = 0;
  lastTimeStamp = 0;
  memset(filename, 0, sizeof(filename));
  if (reader) delete reader;
  if (writer) delete writer;
  if (dbwriter) delete dbwriter;
  if (converter) delete converter;
  reader    = NULL;
  writer    = NULL;
  dbwriter  = NULL;
  converter = NULL;
  clearInUse(this);
}

// ===========================================================================
FileStatus fileHandler::close()
{
  FileStatus status = FileStatusOK;

  if (reader) {
    if ( kvlcOK != reader->close_file() ) {
      status = FileStatusFail;
    }
  }

  if (writer) {
    if ( kvlcOK != writer->close_file() ) {
      status = FileStatusFail;
    }
  }

  if ( dbwriter ) {
    dbwriter->close_file();
  }
  delete this;
  return status;
}

// ===========================================================================
FileStatus fileHandler::openRead (FileType ftype)
{
  switch (ftype)
  {
    case KME24:
      reader = new KvaLogReader_Kme24();
      break;

    case KME25:
      reader = new KvaLogReader_Kme25();
      break;

    case KME40:
      reader = new KvaLogReader_Kme40();
      break;

    case KME50:
      reader = new KvaLogReader_Kme50();
      break;

    case KME60:
      reader = new KvaLogReader_Kme60();
      break;

    default:
      reader = NULL;
      break;
  }

  if ( !reader ) {
    return FileStatusERR_NOT_IMPLEMENTED;
  }

  if ( kvlcOK != reader->open_file(filename) ) {
    return FileStatusERR_FILE_NOT_FOUND;
  }
  type = ftype;
  return FileStatusOK;
}

// ===========================================================================
FileStatus fileHandler::openWrite (FileType ftype)
{
  int on = 1;
  unsigned int mask = 0xFF;

  switch (ftype)
  {
    case KME24:
      writer = new KvaLogWriter_Kme24();
      break;

    case KME25:
      writer = new KvaLogWriter_Kme25();
      break;

    case KME40:
      writer = new KvaLogWriter_Kme40();
      break;

    case KME50:
      writer = new KvaLogWriter_Kme50();
      break;

    case KME60:
      writer = new KvaLogWriter_Kme60();
      break;
    default:
      writer = NULL;
      break;
  }

  if ( !writer ) {
    return FileStatusERR_NOT_IMPLEMENTED;
  }

  if ( kvlcOK != writer->set_filename(filename) ) {
    return FileStatusERR_FILE_ERROR;
  }

  if ( kvlcOK != writer->set_property_value(KVLC_PROPERTY_OVERWRITE, &on, sizeof(on)) ) {
      return FileStatusERR_FILE_ERROR;
  }

  if ( kvlcOK != writer->set_property_value(KVLC_PROPERTY_CHANNEL_MASK, &mask, sizeof(mask)) ) {
    return FileStatusERR_FILE_ERROR;
  }

  if ( kvlcOK != writer->open_file() ) {
    return FileStatusERR_FILE_ERROR;
  }
  type = ftype;
  return FileStatusOK;
}

// ===========================================================================
FileStatus fileHandler::eventCount (uint64_t *cnt)
{
  if ( reader ) {
    *cnt = reader->event_count();
  }
  if ( writer ) {
    *cnt = count;
  }
  return FileStatusOK;
}

// ===========================================================================
FileStatus fileHandler::readEvent (void *e)
{
  imLogData logEvent;
  KvlcStatus status = kvlcOK;

  if ( !reader ) {
    return FileStatusERR_ILLEGAL_REQUEST;
  }

  logEvent.common.new_data = false;

  while (kvlcOK == status && logEvent.common.new_data == false)
  {
    status = reader->read_row(&logEvent);
  }

  if ( kvlcEOF == status ) {
    return FileStatusERR_NOLOGMSG;
  }

  if ( kvlcERR_FILE_ERROR == status ) {
    return FileStatusERR_FILE_ERROR;
  }

  if ( kvlcOK != status ) {
    return FileStatusFail;
  }

  toMemoEventEx(&logEvent, e);

  if ( DEBUG_MODE && dbwriter != NULL ) {
    imLogData logEventDebug;
    converter->interpret_event(e, &logEventDebug);
    dbwriter->write_row(&logEventDebug);
  }
  return FileStatusOK;
}

// ===========================================================================
FileStatus fileHandler::writeEvent (void *e)
{
  imLogData logEvent;
  KvlcStatus status;

  if ( !writer ) {
    return FileStatusERR_ILLEGAL_REQUEST;
  }

  status = converter->interpret_event(e, &logEvent);
  if ( kvlcOK != status ) {
    return FileStatusFail;
  }

  status = writer->write_row(&logEvent);
  if ( kvlcOK != status ) {
    return FileStatusFail;
  }
  count++;
  return FileStatusOK;
}

// ===========================================================================
// API
// ===========================================================================
FileHandle openFile (const char *filename, FileStatus *status, FileType type)
{
  if ( !status ) {
    return INVALID_FILE_HANDLE_VALUE;
  }

  if ( !filename || isTooLong(filename) ) {
    *status = FileStatusERR_PARAM;
    return INVALID_FILE_HANDLE_VALUE;
  }

  if ( !fileHandleAvailable() ) {
    *status = FileStatusERR_NO_FREE_HANDLE;
    return INVALID_FILE_HANDLE_VALUE;
  }

  fileHandler *fh = new fileHandler(filename);

  *status = fh->openRead(type);

  if ( FileStatusOK != *status ) {
    closeFile(fh);
    return INVALID_FILE_HANDLE_VALUE;
  }

  return (FileHandle) fh;
}

// ===========================================================================
FileHandle createFile (const char *filename, FileStatus *status,  FileType type)
{
  if ( !status ) {
    return INVALID_FILE_HANDLE_VALUE;
  }

  if ( !filename || isTooLong(filename) ) {
    *status = FileStatusERR_PARAM;
    return INVALID_FILE_HANDLE_VALUE;
  }

  if ( !fileHandleAvailable() ) {
    *status = FileStatusERR_NO_FREE_HANDLE;
    return INVALID_FILE_HANDLE_VALUE;
  }

  fileHandler *fh = new fileHandler(filename);

  *status = fh->openWrite(type);
  if ( FileStatusOK != *status ) {
    closeFile(fh);
    return INVALID_FILE_HANDLE_VALUE;
  }

  return (FileHandle) fh;
}

// ===========================================================================
FileStatus closeFile (FileHandle h)
{
  if ( isInvalid(h) ) {
    return FileStatusERR_INVALID_HANDLE;
  }

  fileHandler *fh = (fileHandler*) h;

  return fh->close();
}

// ===========================================================================
FileStatus countEvents (FileHandle h, uint64_t *count)
{
  if ( isInvalid(h) ) {
    return FileStatusERR_INVALID_HANDLE;
  }

  if ( !count ) {
    return FileStatusERR_PARAM;
  }

  fileHandler *fh = (fileHandler*) h;

  return fh->eventCount(count);
}

// ===========================================================================
FileStatus readEvent (FileHandle h, void *e)
{
if ( isInvalid(h) ) {
    return FileStatusERR_INVALID_HANDLE;
  }

  if ( !e ) {
    return FileStatusERR_PARAM;
  }
  fileHandler *fh = (fileHandler*) h;

  return fh->readEvent(e);
}

// ===========================================================================
FileStatus writeEvent (FileHandle h, void *e)
{
  if ( isInvalid(h) ) {
    return FileStatusERR_INVALID_HANDLE;
  }

  if ( !e ) {
    return FileStatusERR_PARAM;
  }

  fileHandler *fh = (fileHandler*) h;

  return fh->writeEvent(e);
}

// ===========================================================================
void fileHandler::toMemoEventEx(imLogData *le, void *e)
{
  memoLogEventEx *me = (memoLogEventEx*) e;
  switch(le->common.type) {

    case ILOG_TYPE_RTC:
    {
      me->type          = MEMOLOG_TYPE_CLOCK;
      me->x.rtc.calendarTime = (uint32_t)(le->common.nanos_since_1970 / ONE_BILLION);

      // High resolution timer for RTC in KME24, KME50, KME60, otherwise use last time stamp.
      if (type == KME50 || type == KME24 || type == KME60) {
        me->x.rtc.timeStamp = le->common.time64;
      } else {
        me->x.rtc.timeStamp = lastTimeStamp;
      }
    }
    break;

    case ILOG_TYPE_MESSAGE:
    {
      me->type          = MEMOLOG_TYPE_MSG;
      me->x.msg.flags   = le->msg.flags;
      me->x.msg.id      = le->msg.id;
      me->x.msg.channel = le->msg.channel;
      me->x.msg.dlc     = dlcToNumBytesFD(le->msg.dlc);
      memcpy(me->x.msg.data, le->msg.data, 64);
      me->x.msg.timeStamp = le->common.time64;
      lastTimeStamp = le->common.time64;
    }
    break;

    case ILOG_TYPE_TRIGGER:
    {
      me->type               = MEMOLOG_TYPE_TRIGGER;
      me->x.trig.trigNo      = le->trig.trigNo;
      me->x.trig.postTrigger = le->trig.postTrigger;
      me->x.trig.preTrigger  = le->trig.preTrigger;
      me->x.trig.timeStamp   = le->common.time64;
      me->x.trig.type        = le->trig.type;
      lastTimeStamp = le->common.time64;
    }
    break;

    case ILOG_TYPE_VERSION:
    {
      me->type               = MEMOLOG_TYPE_VERSION;
      me->x.ver.lioMajor     = le->ver.lioMajor;
      me->x.ver.lioMinor     = le->ver.lioMinor;
      me->x.ver.fwMajor      = le->ver.fwMajor;
      me->x.ver.fwMinor      = le->ver.fwMinor;
      me->x.ver.fwBuild      = le->ver.fwBuild;
      me->x.ver.serialNumber = le->ver.serialNumber;
      me->x.ver.eanHi        = le->ver.eanHi;
      me->x.ver.eanLo        = le->ver.eanLo;
    }
    break;

    case ILOG_TYPE_CANOTHER:
    {
      me->type = MEMOLOG_TYPE_INVALID;
      PRINTF(("ERROR: ILOG_TYPE_CANOTHER unexpected!\n"));
    }
    break;

    default:
      me->type = MEMOLOG_TYPE_INVALID;
      PRINTF(("ERROR: Unknown imLogData type: %d\n", le->common.type));
      break;
  }
}
