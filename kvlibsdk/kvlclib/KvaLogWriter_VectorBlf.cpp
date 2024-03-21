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
**  Writer for "Vector BLF" i.e. BLF files created by CANalyzer et al
** ---------------------------------------------------------------------------
*/

#include <stdint.h>
#include "common_defs.h"
#include "KvaLogWriter_VectorBlf.h"
#include "kvdebug.h"
#include "os_util.h"

#undef USE_BINLOG
#ifdef USE_BINLOG
#include "binlog.h"
#undef PRINTF
#define PRINTF(X) printf X
#endif

#define BL_FILE_SIGNATURE               0x47474F4C
#define BL_OBJ_SIGNATURE                0x4A424F4C
#define BL_OBJ_TYPE_CAN_MESSAGE         1
#define BL_OBJ_TYPE_CAN_ERROR           2
#define BL_OBJ_TYPE_CAN_OVERLOAD        3
#define BL_OBJ_TYPE_APP_TRIGGER         5
#define BL_OBJ_TYPE_CONTAINER          10
#define BL_COMPRESSION_NONE             0
#ifndef USE_BINLOG
#define BL_OBJ_FLAG_TIME_ONE_NANS       2
#define BL_TRIGGER_FLAG_SINGLE_TRIGGER  0
#define CAN_MSG_FLAGS_EXT( dir, rtr, wu, nerr) \
  (((rtr&1) << 7) + ((wu&1) << 6) + ((nerr&1) << 5) + (dir&0xF))
#endif
#define BLF_ID_EXT 0x80000000

struct blf_file_header
{
  uint32_t file_signature;
  uint32_t header_size;
  uint32_t header_version;

  uint8_t header_app_id;
  // BL_APPID_UNKNOWN      0
  // BL_APPID_CANALYZER    1
  // BL_APPID_CANOE        2
  // BL_APPID_CANSTRESS    3
  // BL_APPID_CANLOG       4
  // BL_APPID_CANAPE       5
  // BL_APPID_CANCASEXLLOG 6

  uint8_t header_options;
  // BL_COMPRESSION_NONE    0
  // BL_COMPRESSION_SPEED   1
  // BL_COMPRESSION_DEFAULT 6
  // BL_COMPRESSION_MAX     9

  uint8_t header_app_major;
  uint8_t header_app_minor;
  uint64_t header_compressed_size;
  uint64_t header_uncompressed_size;
  uint32_t header_object_count;
  uint8_t header_app_build;
  uint8_t unknown_1[3];
  uint16_t header_start_year;
  uint16_t header_start_month;
  uint16_t header_start_weekday;
  uint16_t header_start_day;
  uint16_t header_start_hour;
  uint16_t header_start_minute;
  uint16_t header_start_second;
  uint16_t header_start_milliseconds;

  uint16_t header_date_2[8]; // Endtime; another year/month/wday/day/hour/min/sec/ms.

  uint8_t unknown_2[72];
};

static const int header_size_v1 = 32;
static const int header_size_v2 = 40;

struct blf_object_header
{
  uint32_t header_signature;
  uint16_t header_size;
  uint16_t header_version;
  uint32_t object_size;
  uint32_t object_type;

  uint32_t object_flags;
  uint8_t timestamp_status;
  uint8_t reserved;
  uint16_t object_version;
  uint64_t object_timestamp;

  // Optional: only in header v2.
  uint64_t original_timestamp;
};

struct blf_can_message
{
  uint16_t channel;
  uint8_t flags;
  uint8_t dlc;
  uint32_t id;
  uint8_t data[8];
};

struct blf_errorframe
{
  uint16_t channel;
  uint16_t length;
};

struct blf_overloadframe
{
  uint16_t channel;
  uint16_t dummy;
};

struct blf_trigger
{
  uint64_t pre_timestamp;
  uint64_t post_timestamp;
  uint16_t channel;
  uint16_t flags;
  uint32_t app;
};

struct output
{
  char buffer[256];
  int length;
  time_uint64 timestamp;
  int object_count;
};

static void blf_header_v1 (struct output *output, uint32_t size, uint32_t type);

#ifdef USE_BINLOG
HANDLE blf_handle;
#endif

//--------------------------------------------------------------------------
KvaLogWriter_VectorBlf::KvaLogWriter_VectorBlf()
{
  PRINTF(("KvaLogWriter_VectorBlf::KvaLogWriter_VectorBlf()\n"));
  header_written = 0;
  object_count = 0;

#ifdef USE_BINLOG
  blf_handle = BLCreateFile("test.blf", GENERIC_WRITE);
  PRINTF(("BLCreateFile = %d\n", blf_handle));
  PRINTF(("BLSetApplication = %d\n", BLSetApplication(blf_handle, 0, 0, 0, 0)));
  PRINTF(("BLSetWriteOptions = %d\n", BLSetWriteOptions(blf_handle, 0, 0)));
#endif
}

KvaLogWriter_VectorBlf::~KvaLogWriter_VectorBlf()
{
  PRINTF(("~KvaLogWriter_VectorBlf\n"));
}

//--------------------------------------------------------------------------
static void write_container (struct output *output, uint32_t compressed_size,
                             uint32_t uncompressed_size)
{
  struct blf_object_header *header = (struct blf_object_header *)output->buffer;
  blf_header_v1(output, compressed_size, BL_OBJ_TYPE_CONTAINER);
  header->header_size = 16;
  header->object_flags = 0;
  header->object_timestamp = uncompressed_size;
  output->length = 32;
}

//--------------------------------------------------------------------------
KvlcStatus KvaLogWriter_VectorBlf::write_header()
{
  char buffer[sizeof (blf_file_header)];
  struct blf_file_header *header = (struct blf_file_header *)buffer;
  KvlcStatus kvstatus = kvlcOK;

  PRINTF(("KvaLogWriter_VectorBlf::write_header\n"));

  memset(buffer, 0, sizeof buffer);
  header->file_signature = BL_FILE_SIGNATURE;
  header->header_size = sizeof buffer;
  header->header_version = 5;
  header->header_options = BL_COMPRESSION_NONE;
  header->header_app_id = 0;
  header->header_app_major = 0;
  header->header_app_minor = 0;
  header->header_app_build = 0;

  if (header_written)
    {
      int64_t pos = os_ftell(outfile);
      os_fseek(outfile, 0, SEEK_SET);
      header->header_compressed_size = file_size;
      header->header_uncompressed_size = file_size;
      header->header_object_count = object_count;
      if (start_of_logging) {
        struct tm newtime;
        get_calendar_time(start_of_logging / ONE_BILLION, &newtime);
        header->header_start_year = newtime.tm_year+1900;
        header->header_start_month = newtime.tm_mon + 1;
        header->header_start_weekday = newtime.tm_wday;
        header->header_start_day = newtime.tm_mday;
        header->header_start_hour = newtime.tm_hour;
        header->header_start_minute = newtime.tm_min;
        header->header_start_second = newtime.tm_sec;
        header->header_start_milliseconds = 0;
      }
      fwrite(buffer, sizeof buffer, 1, outfile);

      struct output output;
      uint64_t compressed_size = file_size - sizeof buffer - 32;
      uint32_t uncompressed_size = (uint32_t)compressed_size;
      write_container(&output, (uint32_t)compressed_size, uncompressed_size);
      fwrite(output.buffer, output.length, 1, outfile);

      os_fseek(outfile, pos, SEEK_SET);
#ifdef USE_BINLOG
      PRINTF(("BLCloseHandle = %d\n", BLCloseHandle(blf_handle)));
#endif
      return kvlcOK;
    }
  else
    {
      kvstatus = write_file(buffer, sizeof buffer);
      if (kvstatus == kvlcOK)
        {
          memset(buffer, 0, sizeof buffer);
          kvstatus = write_file(buffer, 32);
        }
    }

  header_written = 1;
  return kvstatus;
}

//--------------------------------------------------------------------------
static void blf_header_v1 (struct output *output, uint32_t size, uint32_t type)
{
  struct blf_object_header *header = (struct blf_object_header *)output->buffer;
  memset(header, 0, header_size_v1);
  header->header_signature = BL_OBJ_SIGNATURE;
  header->header_size = header_size_v1;
  header->header_version = 1;
  header->object_size = header_size_v1 + size;
  header->object_type = type;
  header->object_flags = BL_OBJ_FLAG_TIME_ONE_NANS;
  header->object_version = 0;
  header->object_timestamp = output->timestamp;
  output->length = header->object_size;

  if (type != BL_OBJ_TYPE_CONTAINER) {
    output->object_count = 1;
  }
}

//--------------------------------------------------------------------------
static KvlcStatus write_trigger (struct output *output, imLogData_trigger * /* trig */)
{
  struct blf_trigger *trigger = (struct blf_trigger *)(output->buffer + header_size_v1);

  blf_header_v1(output, sizeof (struct blf_trigger), BL_OBJ_TYPE_APP_TRIGGER);
  trigger->pre_timestamp = output->timestamp;
  trigger->post_timestamp = output->timestamp;
  trigger->channel = 1;
  trigger->flags = BL_TRIGGER_FLAG_SINGLE_TRIGGER;
  trigger->app = 0;

#ifdef USE_BINLOG
  {
    VBLAppTrigger appTrigger;
    memset(&appTrigger, 0, sizeof appTrigger);
    appTrigger.mHeader.mBase.mSignature = BL_OBJ_SIGNATURE;
    appTrigger.mHeader.mBase.mHeaderSize = sizeof( appTrigger.mHeader);
    appTrigger.mHeader.mBase.mHeaderVersion = 1;
    appTrigger.mHeader.mBase.mObjectSize = sizeof( VBLAppTrigger);
    appTrigger.mHeader.mBase.mObjectType = BL_OBJ_TYPE_APP_TRIGGER;
    appTrigger.mHeader.mObjectFlags = BL_OBJ_FLAG_TIME_ONE_NANS;
    appTrigger.mHeader.mObjectTimeStamp = output->timestamp;
    appTrigger.mPreTriggerTime = output->timestamp;
    appTrigger.mPostTriggerTime = output->timestamp;
    appTrigger.mChannel = 1;
    appTrigger.mFlags = BL_TRIGGER_FLAG_SINGLE_TRIGGER;
    appTrigger.mAppSecific2 = 0;
    if (! BLWriteObject(blf_handle, &appTrigger.mHeader.mBase))
      {
        PRINTF(("BLWriteObject failed\n"));
      }
  }
#endif

  return kvlcOK;
}

//--------------------------------------------------------------------------
static KvlcStatus write_errorframe (struct output *output, imLogData_canMessage *msg)
{
  struct blf_errorframe *error = (struct blf_errorframe *)(output->buffer + header_size_v1);

  blf_header_v1(output, sizeof (struct blf_errorframe), BL_OBJ_TYPE_CAN_ERROR);
  error->channel = msg->channel + 1;
  error->length = 0;

  return kvlcOK;
}

//--------------------------------------------------------------------------
static KvlcStatus write_overloadframe (struct output *output, imLogData_canMessage *msg)
{
  struct blf_overloadframe *error = (struct blf_overloadframe *)(output->buffer + header_size_v1);

  blf_header_v1(output, sizeof (struct blf_overloadframe), BL_OBJ_TYPE_CAN_OVERLOAD);
  error->channel = msg->channel + 1;
  error->dummy = 0;

  return kvlcOK;
}

//--------------------------------------------------------------------------
static uint8_t convert_flags (unsigned flags)
{
  int dir = (flags & (canMSG_TXACK | canMSG_TXRQ)) != 0;
  int rtr = (flags & canMSG_RTR) != 0;
  int wu = (flags & canMSG_WAKEUP) != 0;
  int nerr = (flags & canMSG_NERR) != 0;
  return CAN_MSG_FLAGS_EXT(dir, rtr, wu, nerr);
}

//--------------------------------------------------------------------------
static KvlcStatus write_message (struct output *output, imLogData_canMessage *msg)
{
  struct blf_can_message *can = (struct blf_can_message *)(output->buffer + header_size_v1);

  if (msg->flags & canMSG_ERROR_FRAME)
    {
      return write_errorframe(output, msg);
    }
  else if (msg->flags & canMSGERR_OVERRUN)
    {
      return write_overloadframe(output, msg);
    }

  blf_header_v1(output, sizeof (struct blf_can_message), BL_OBJ_TYPE_CAN_MESSAGE);

  can->channel = msg->channel + 1;
  can->flags = convert_flags(msg->flags);
  can->dlc = msg->dlc;
  can->id = msg->id;
  if (msg->flags & canMSG_EXT) {
    can->id |= BLF_ID_EXT;
  }
  memcpy(can->data, msg->data, 8);

#ifdef USE_BINLOG
  {
    VBLCANMessage message;
    memset(&message, 0, sizeof message);
    message.mHeader.mBase.mSignature = BL_OBJ_SIGNATURE;
    message.mHeader.mBase.mHeaderSize = sizeof(message.mHeader);
    message.mHeader.mBase.mHeaderVersion = 1;
    message.mHeader.mBase.mObjectSize = sizeof(VBLCANMessage);
    message.mHeader.mBase.mObjectType = BL_OBJ_TYPE_CAN_MESSAGE;
    message.mHeader.mObjectFlags = BL_OBJ_FLAG_TIME_ONE_NANS;
    message.mHeader.mObjectTimeStamp = output->timestamp;
    message.mChannel = msg->channel + 1;
    message.mFlags = convert_flags(msg->flags);
    message.mDLC = msg->dlc;
    message.mID = msg->id;
    memcpy(message.mData, msg->data, msg->dlc);
    if (! BLWriteObject(blf_handle, &message.mHeader.mBase))
      {
        PRINTF(("BLWriteObject failed\n"));
      }
  }
#endif

  return kvlcOK;
}

//--------------------------------------------------------------------------
KvlcStatus KvaLogWriter_VectorBlf::write_row(imLogData *logEvent)
{
  KvlcStatus kvstatus;
  struct output output;

  output.timestamp = logEvent->common.time64 + get_property_offset();
  output.object_count = 0;

  if (!isOpened) {
    PRINTF(("KvaLogWriter_VectorBlf::write_row, file is not opened\n"));
    return kvlcERR_FILE_ERROR;
  }

  if (!start_of_logging) setStartOfLogging(logEvent);

  switch (logEvent->common.type)
    {
    case ILOG_TYPE_TRIGGER:
      kvstatus = write_trigger(&output, &logEvent->trig);
      break;

    case ILOG_TYPE_MESSAGE:
      if (logEvent->msg.flags & canFDMSG_FDF) {
        if (dlcToNumBytesFD(logEvent->msg.dlc) > 8) {
          data_truncation_occurred = true;
        }
      }
      if (logEvent->msg.flags & canMSGERR_OVERRUN) {
        overrun_occurred = true;
      }
      kvstatus = write_message(&output, &logEvent->msg);
      break;

    case ILOG_TYPE_RTC:
    case ILOG_TYPE_CANOTHER:
    case ILOG_TYPE_VERSION:
      // Nothing corresponding in BLF.
      output.length = 0;
      kvstatus = kvlcOK;
      break;

    default:
      PRINTF(("logEvent->type = %d, frame_counter = %lu\n",
              logEvent->common.type,
              logEvent->common.event_counter));
      kvstatus = kvlcERR_INVALID_LOG_EVENT;
      break;
    }

  if (kvstatus == kvlcOK && output.length > 0)
    {
      kvstatus = write_file(output.buffer, output.length);
      object_count += output.object_count;
    }

  return kvstatus;
}

KvlcStatus KvaLogWriter_VectorBlf::close_file()
{
  header_written = 0;
  object_count = 0;
  return KvaLogWriter::close_file();
}

#define NAME        "Vector BLF"
#define EXTENSION   "blf"
#define DESCRIPTION "CAN frames in Vector BLF format"

class KvaWriterMaker_VectorBlf : public KvaWriterMaker
{
  public:
    KvaWriterMaker_VectorBlf() : KvaWriterMaker(KVLC_FILE_FORMAT_VECTOR_BLF) {
      propertyList[KVLC_PROPERTY_START_OF_MEASUREMENT] = true;
      propertyList[KVLC_PROPERTY_FIRST_TRIGGER] = true;
      propertyList[KVLC_PROPERTY_USE_OFFSET] = true;
      propertyList[KVLC_PROPERTY_OFFSET] = true;
      propertyList[KVLC_PROPERTY_CHANNEL_MASK] = true;
      propertyList[KVLC_PROPERTY_CROP_PRETRIGGER] = true;
      propertyList[KVLC_PROPERTY_SIZE_LIMIT] = true;
      propertyList[KVLC_PROPERTY_TIME_LIMIT] = true;
      propertyList[KVLC_PROPERTY_OVERWRITE] = true;
    }
    int getName(char *str) { return sprintf(str, "%s", NAME); }
    int getExtension(char *str) { return sprintf(str, "%s", EXTENSION); }
    int getDescription(char *str) { return sprintf(str, "%s", DESCRIPTION); }

  private:
    KvaLogWriter *OnCreateWriter() {
      return new KvaLogWriter_VectorBlf();
    }
}  registerKvaLogWriter_VectorBlf;
