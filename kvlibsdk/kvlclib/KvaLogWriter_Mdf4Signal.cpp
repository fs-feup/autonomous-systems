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
**  Writer for MDF (Vector's Signal based MDF, their old binary format)
** ---------------------------------------------------------------------------
*/

#include "common_defs.h"
#include "KvaLogWriter_Mdf4Signal.h"
#include "TimeConv.h"
#include "kvdebug.h"





long long KvaLogWriter_Mdf4Signal::get_bytes_written() {
  if (!mdf) {
    return -1;
  }
  return mdf->size();
}

KvaLogWriter_Mdf4Signal::KvaLogWriter_Mdf4Signal()
{
  PRINTF(("KvaLogWriter_Mdf4Signal::KvaLogWriter_Mdf4Signal()"));
  isSetup = 0;
  mdf = NULL;
  mdfFileHandle = NULL;
}

KvaLogWriter_Mdf4Signal::~KvaLogWriter_Mdf4Signal()
{
  PRINTF(("KvaLogWriter_Mdf4Signal::~KvaLogWriter_Mdf4Signal()"));
  if (mdf) {
    delete mdf;
    mdf = NULL;
  }
  if (mdfFileHandle) {
    fclose(mdfFileHandle);
    mdfFileHandle = NULL;
  }
  isSetup = 0;
}

KvlcStatus KvaLogWriter_Mdf4Signal::open_file()
{
  KvlcStatus status = kvlcOK;
  const char *filename = get_filename();

  if (split_files()) {
    filename = get_next_filename();
  }

  status = file_status();
  if (status != kvlcOK) {
    return status;
  }

  PRINTF(("KvaLogWriter_Mdf4Signal::open_file"));
  //mdfFileHandle = fopen(filename, "wb");
  mdfFileHandle = utf_fopen(filename, "wb");
  if (!mdfFileHandle) {
    PRINTF(("ERROR: Unable to open '%s'\n", filename));
    return kvlcERR_FILE_ERROR;
  }
  isOpened = true;

  // Since the MDF version property isn't set until after the KvaLogWriter is
  // created, we have to delay some initialization to now (instead of in Mdf3's
  // constructor).
  status = create_mdf_converter();

  return status;
}

KvlcStatus KvaLogWriter_Mdf4Signal::close_file()
{
  if (!mdf) return kvlcOK;
  KvlcStatus stat = kvlcOK;

  if (start_of_logging) {
    unsigned int year, month, day, hour, minute, second;

    unix_time(start_of_logging / ONE_BILLION,
                &year,
                &month,
                &day,
                &hour,
                &minute,
                &second);

    mdf->setStartOfRecording(
            year,
            month,
            day,
            hour,
            minute,
            second
          );
  }
  if (isOpened) {
    mdf->write_header();
    mdf->write_data();
    mdf->write_header();
    mdf->close();
  }
  mdfFileHandle = NULL;
  isOpened = false;
  isSetup = 0;

  if (mdf) {
    delete mdf;
    mdf = NULL;
  }
  if (mdfFileHandle) {
    fclose(mdfFileHandle);
    mdfFileHandle = NULL;
  }

  return stat;
}

static int get_motorola_start_bit_for_display (int start_bit, int dlc)
{
  int bit = start_bit % 8;
  int byte = start_bit / 8;
  return (dlc - 1 - byte) * 8 + bit;
}


KvlcStatus KvaLogWriter_Mdf4Signal::write_header()
{
  unsigned int i;
  MdfStatus      stat;
  PRINTF(("KvaLogWriter_Mdf4Signal::write_header()"));
  if (!mdfFileHandle) return kvlcOK;
  if (!isSetup) {
    mdf->create(mdfFileHandle);

    isSetup = 1;
    for (i = 0; i < databases.size(); i++) {
      KvaDbHnd dh = databases[i];
      KvaDbMessageHnd mh;
      KvaDbSignalHnd sh, mux_signal;
      char msgname[500], signame[500], unit[500], fullname[500];
      unsigned int dbid;
      int          dbdlc;
      unsigned int dbflags;
      unsigned int msgmask = CAN_ID_MASK_ALL_BITS;
      unsigned int chanmask = databases_chanmask[i];

      KvaDbStatus dbstat = kvaDbGetFirstMsg(dh, &mh);
      if (dbstat != kvlcOK) {
        continue;
      }
      while (dbstat == kvlcOK) {
        dbstat = kvaDbGetMsgName(mh, msgname, sizeof(msgname));
        if (dbstat != kvlcOK) {
          break;
        }
        dbstat = kvaDbGetMsgMux(mh, &mux_signal);
        if (dbstat != kvlcOK) {
          break;
        }
        dbstat = kvaDbGetMsgId(mh, &dbid, &dbflags);
        if (dbstat != kvlcOK) {
          PRINTF(("kvaDbGetMsgId error %d\n", dbstat));
        }

        dbstat = kvaDbGetMsgDlc(mh, &dbdlc);
        if (dbstat != kvlcOK) {
          PRINTF(("kvaDbGetMsgDlc error %d\n", dbstat));
        }

        if (get_property_hlp_j1939()) {
          unsigned int msgflags;
          dbstat = kvaDbGetMsgFlags(mh, &msgflags);
          if (dbstat != kvlcOK) {
            PRINTF(("kvaDbGetMsgFlags error %d\n", dbstat));
          } else {
            if (msgflags & KVADB_MESSAGE_EXT && msgflags & KVADB_MESSAGE_J1939) {
              msgmask = get_J1939_mask(dbid);
            }
          }
        }


        for (int channel = 0; channel < 32; channel++){
          if (!(chanmask & (1<<channel))){
            continue;
          }
          dbstat = kvaDbGetFirstSignal(mh, &sh);

          while (dbstat == kvlcOK) {
            int mux_value = 0;
            dbstat = kvaDbGetSignalMode(sh, &mux_value);
            if (dbstat != kvlcOK) {
              PRINTF(("kvaDbGetSignalMode error %d\n", dbstat));
            }
            MuxChecker mux_checker(mux_signal, mux_value, reinterpret_cast<MuxCallback> (&MuxCallbackValueChecker));
            stat = mdf->new_dg(dbid, msgmask, mux_checker, dbdlc, std::string(msgname), channel + 1);
            //whole data group is characterized by multiplex value, stored in MuxChecker, which will be later checked against every event
            //to determine what events data are to be included in this data block
            switch (stat) {
              case MDF_ERROR_DATATYPE:
                PRINTF(("error datatype when doing new_dg"));
                return kvlcERR_MIXED_ENDIANNESS;
              case MDF_ERROR_MEMORY:
                PRINTF(("error memory when doing new_dg"));
                return kvlcERR_INTERNAL_ERROR;
              case MDF_OK:
                break;
              default:
                PRINTF(("this shouldn't happen\n"));
                return kvlcERR_INTERNAL_ERROR;
            }
            dbstat = kvaDbGetSignalName(sh, signame, sizeof(signame));
            if (dbstat != kvlcOK) {
              break;
            }
            PRINTF(("Another signal. This signal's (%s) mode value is %d\n",signame,mux_value));
            if (mProperties.mShowFullyQualifiedNames == ON) {
              dbstat = kvaDbGetSignalQualifiedName(sh, fullname, sizeof(fullname));
              if (dbstat != kvlcOK) {
                break;
              }
            } else {
              strcpy(fullname, signame);
            }

            dbstat = kvaDbGetSignalUnit(sh, unit, sizeof(unit));
            if (dbstat != kvlcOK) {
              break;
            }

            double factor, offset;
            dbstat = kvaDbGetSignalValueScaling(sh, &factor, &offset);
            if (dbstat != kvlcOK) {
              break;
            }
            int startbit;
            int length_in_bits;
            dbstat = kvaDbGetSignalValueSize(sh, &startbit, &length_in_bits);
            if (dbstat != kvlcOK) {
              break;
            }

            KvaDbSignalType       signal_type;
            KvaDbSignalEncoding   signal_encoding;
            MDF_UINT16            mdf_signal_type;

            dbstat = kvaDbGetSignalRepresentationType(sh, &signal_type);
            if (dbstat != kvlcOK) {
              break;
            }

            dbstat = kvaDbGetSignalEncoding(sh, &signal_encoding);
            if (dbstat != kvlcOK) {
              break;
            }

            // default mdf_signal_type = MDF_SIGNAL_DATA_TYPE_BYTE_ARRAY;
            mdf_signal_type = MDF_SIGNAL_DATA_TYPE_UNSIGNED_INT;
            if (signal_encoding == kvaDb_Intel) {
              switch (signal_type) {
                case kvaDb_Signed: {
                  mdf_signal_type = MDF_SIGNAL_DATA_TYPE_SIGNED_INT_LE;
                } break;
                case kvaDb_Unsigned: {
                  mdf_signal_type = MDF_SIGNAL_DATA_TYPE_UNSIGNED_INT_LE;
                } break;
                case kvaDb_Float: {
                  mdf_signal_type = MDF_SIGNAL_DATA_TYPE_IEEE_FLOAT_LE;
                } break;
                case kvaDb_Double: {
                  mdf_signal_type = MDF_SIGNAL_DATA_TYPE_IEEE_DOUBLE_LE;
                } break;
              default:
                return kvlcERR_INTERNAL_ERROR;
              }
              PRINTF(("INTEL:%d,",startbit));
            }
            else if (signal_encoding == kvaDb_Motorola) {
              int startbyte, endbyte, bitoffset;
              // From the Motorola example in MDF specification v3.3

              startbit = get_motorola_start_bit_for_display(startbit, dbdlc);
              startbit = dbdlc*8 - startbit - length_in_bits;
              startbyte = startbit / 8;
              endbyte = (startbit + length_in_bits) / 8;
              bitoffset = startbit + length_in_bits  - endbyte * 8;
              if (bitoffset) {
                bitoffset = 8 - bitoffset;
              }
              startbit = startbyte * 8 + bitoffset;

              switch (signal_type) {
                case kvaDb_Signed: {
                  mdf_signal_type = MDF_SIGNAL_DATA_TYPE_SIGNED_INT_BE;
                } break;
                case kvaDb_Unsigned: {
                  mdf_signal_type = MDF_SIGNAL_DATA_TYPE_UNSIGNED_INT_BE;
                } break;
                case kvaDb_Float: {
                  mdf_signal_type = MDF_SIGNAL_DATA_TYPE_IEEE_FLOAT_BE;
                } break;
                case kvaDb_Double: {
                  mdf_signal_type = MDF_SIGNAL_DATA_TYPE_IEEE_DOUBLE_BE;
                } break;
              default:
                return kvlcERR_INTERNAL_ERROR;
              }
              PRINTF(("Motorola: %d", startbit));
            }
            //there might be many dg nodes, but due to mechanics of new_dg,
            //at that moment current dg pointer of the hdNode shall be addressing the right dg node for our signal
            stat = mdf->new_sig(
                        dbid,
                        fullname,
                        signame,
                        unit,
                        startbit,
                        length_in_bits,
                        mdf_signal_type,
                        MDF_CONVERSION_TYPE_PARAMETRIC_LINEAR,
                        factor,
                        offset
                        );

            switch (stat) {
              case MDF_ERROR_DATATYPE:
                PRINTF(("error datatype when doing new_sig"));
                return kvlcERR_MIXED_ENDIANNESS;
              case MDF_ERROR_MEMORY:
                PRINTF(("error memory when doing new_sig"));
                return kvlcERR_INTERNAL_ERROR;
              case MDF_OK:
                break;
              default:
                PRINTF(("this shouldn't happen\n"));
                return kvlcERR_INTERNAL_ERROR;
            }

            dbstat = kvaDbGetNextSignal(mh, &sh);
            if (dbstat != kvlcOK) {
              break;
            }
          }
        }

        dbstat = kvaDbGetNextMsg(dh, &mh);
        if (dbstat != kvlcOK) {
          break;
        }
      }
    }
    mdf->write_header();
  }

  return kvlcOK;
}

KvlcStatus KvaLogWriter_Mdf4Signal::write_row(imLogData *logEvent)
{
  time_uint64 lastTime = 0;
  KvlcStatus kvstatus = kvlcOK;

  MdfCanFrameType frame;

  if (!isOpened) {
    PRINTF(("KvaLogWriter_Mdf::write_row, file was not opened\n"));
    return kvlcERR_FILE_ERROR;
  }

  mdf->setCompressionLevel(mProperties.mCompressionLevel);

  memset(&frame, 0, sizeof(frame));
  int dataLen = 0;

  if (!start_of_logging) setStartOfLogging(logEvent);

  switch (logEvent->common.type) {

    case ILOG_TYPE_MESSAGE:
    {
      if (!((1<<logEvent->msg.channel) & get_property_channel_mask())) {
        // Don't use this channel so bail out
        return kvlcOK;
      }

      if (logEvent->msg.flags & canMSGERR_OVERRUN) {
        overrun_occurred = true;
      }

      if ((logEvent->msg.flags & canMSG_ERROR_FRAME) != 0) {
        // CAN Error Frame
        return kvlcOK;
      }

      if ((logEvent->msg.flags & canMSG_RTR) != 0) {
        // CAN Remote Frame
        return kvlcOK;
      }

      if (lastTime > logEvent->common.time64) {
        PRINTF(("WARNING, decreasing time\n"));
      }
      lastTime = logEvent->common.time64;

      // Copy data from message
      frame.dlc = logEvent->msg.dlc;
      frame.canId = logEvent->msg.id;
      frame.flags = logEvent->msg.flags;
      if (logEvent->msg.flags & canMSG_EXT) {
        frame.canId |= MDF_ID_EXT;
      }

      if (logEvent->msg.flags & canFDMSG_FDF) {
        dataLen = dlcToNumBytesFD(logEvent->msg.dlc);
      }
      else {
        dataLen = MIN(logEvent->msg.dlc, 8);
      }
      memcpy(frame.data, logEvent->msg.data, dataLen);
      frame.datalength = dataLen;

      mdf->addFrame(logEvent->msg.channel + 1, MDF_CAN_FRAME_TYPE,
        logEvent->common.time64, &frame);

      break;
    }
    case ILOG_TYPE_CANOTHER:
    case ILOG_TYPE_RTC:
    case ILOG_TYPE_TRIGGER:
    case ILOG_TYPE_VERSION:
    {
      break;
    }
    default:
      PRINTF(("logEvent->type = %d, frame_counter = %lu\n\n\n",
              logEvent->common.type, logEvent->common.event_counter));
      kvstatus = kvlcERR_INVALID_LOG_EVENT;
      break;
  }
  logEvent->common.new_data = false;

  return kvstatus;
}
KvlcStatus KvaLogWriter_Mdf4Signal::attach_file(const char *filename)
{
  if (!mdf) {
    mdf = new Mdf4();
    mdf->initAndSetVersion(410);
  }

  if ( mdf->attach(filename) ) {
    return kvlcERR_FILE_ERROR;
  }
  return kvlcOK;
}

KvlcStatus KvaLogWriter_Mdf4Signal::create_mdf_converter()
{
  PRINTF(("KvaLogWriter_Mdf4Signal::create_mdf_converter()"));

  if (!mdf) {
    mdf = new Mdf4();
    mdf->initAndSetVersion(410);
  }
  mdf->setCompressionLevel(mProperties.DEFAULT_COMPRESSION_LEVEL);


  // Not possible with signals
  mdf->useVariableRecordSize(false);

  // Allow 64 byte signals
  if ( get_property_limit_databytes() == 64 ) {
    mdf->useFixedRecordSize64(true);
  }

  return kvlcOK;
}


#define NAME        "MDF v4.1 Signal"
#define EXTENSION   "mf4"
#define DESCRIPTION "Selected signals in MDF v4.1 for Vector CANalyzer"
class KvaWriterMaker_Mdf4Signal : public KvaWriterMaker
{
  public:
    KvaWriterMaker_Mdf4Signal() : KvaWriterMaker(KVLC_FILE_FORMAT_MDF_4X_SIGNAL) {
      propertyList[KVLC_PROPERTY_START_OF_MEASUREMENT] = true;
      propertyList[KVLC_PROPERTY_FIRST_TRIGGER] = true;
      propertyList[KVLC_PROPERTY_CHANNEL_MASK] = true;
      propertyList[KVLC_PROPERTY_SIGNAL_BASED] = true;
      propertyList[KVLC_PROPERTY_SHOW_SIGNAL_SELECT] = true;
      propertyList[KVLC_PROPERTY_CROP_PRETRIGGER] = true;
      propertyList[KVLC_PROPERTY_TIME_LIMIT] = true;
      propertyList[KVLC_PROPERTY_ATTACHMENTS] = true;
      propertyList[KVLC_PROPERTY_LIMIT_DATA_BYTES] = true;
      propertyList[KVLC_PROPERTY_OVERWRITE] = true;
      propertyList[KVLC_PROPERTY_FULLY_QUALIFIED_NAMES] = true;
      propertyList[KVLC_PROPERTY_COMPRESSION_LEVEL]    = true;
      propertyList[KVLC_PROPERTY_HLP_J1939] = true;
    }
    int getName(char *str) { return sprintf(str, "%s", NAME); }
    int getExtension(char *str) { return sprintf(str, "%s", EXTENSION); }
    int getDescription(char *str) { return sprintf(str, "%s", DESCRIPTION); }

  private:
    KvaLogWriter *OnCreateWriter() {
      return new KvaLogWriter_Mdf4Signal();
    }
}  registerKvaLogWriter_Mdf4Signal;

