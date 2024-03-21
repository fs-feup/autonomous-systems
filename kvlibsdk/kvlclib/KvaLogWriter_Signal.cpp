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

#include "common_defs.h"
#include "KvaLogWriter_Signal.h"
#include "SignalRow.h"
#include "TimeConv.h"
#include "kvdebug.h"


//===========================================================================
KvaLogWriter_Signal::KvaLogWriter_Signal()
{
  PRINTF(("KvaLogWriter_Signal::KvaLogWriter_Signal()\n"));
  sr = NULL; // created in write_header
  counter = 1;
  DlcWarningsList.clear();

  samplingSr = NULL;  // created in write_header if needed
  samplingHighResTime = 0;
  samplingAbsTime = 0;
  samplingTimeStep = 0;
}

//===========================================================================
KvaLogWriter_Signal::~KvaLogWriter_Signal()
{
  PRINTF(("KvaLogWriter_Signal::~KvaLogWriter_Signal()\n"));
  delete sr;
  if (samplingSr) delete samplingSr;
}

//===========================================================================
KvlcStatus KvaLogWriter_Signal::write_header()
{
  unsigned int i;
  unsigned int curColumn = 0;
  KvlcStatus kvstatus = kvlcOK;
  SignalRow *original = NULL;;

  // Should only be done once
  if (!sr) {
    sr = new SignalRow(get_property_separator_char());
    for (i = 0; i < databases.size(); i++) {
      KvaDbHnd dh = databases[i];
      KvaDbMessageHnd mh;
      KvaDbSignalHnd sh;
      char msgname[500], signame[500], unit[500], fullname[500];

      KvaDbStatus dbstat = kvaDbGetFirstMsg(dh, &mh);
      if (dbstat != kvaDbOK) {
        continue;
      }
      while (dbstat == kvaDbOK) {
        KvaDbSignalHnd mux_signal;
        dbstat = kvaDbGetMsgName(mh, msgname, sizeof(msgname));
        if (dbstat != kvaDbOK) {
          break;
        }
        dbstat = kvaDbGetMsgMux(mh, &mux_signal);
        if (dbstat != kvaDbOK) {
          break;
        }
        dbstat = kvaDbGetFirstSignal(mh, &sh);
        while (dbstat == kvaDbOK) {
          int mux_value = 0;
          dbstat = kvaDbGetSignalMode(sh, &mux_value);
          if (dbstat != kvaDbOK) {
            PRINTF(("kvaDbGetSignalMode error %d\n", dbstat));
          }
          dbstat = kvaDbGetSignalName(sh, signame, sizeof(signame));
          if (dbstat != kvaDbOK) {
            break;
          }
          dbstat = kvaDbGetSignalQualifiedName(sh, fullname, sizeof(fullname));
          if (dbstat != kvaDbOK) {
            break;
          }

          sr->setName(curColumn, signame);
          sr->setFullName(curColumn, fullname);

          sigName2column.insert(std::pair<std::string, int>{fullname, curColumn});

          sr->setNumDecimals(curColumn, get_property_data_decimals());
          sr->setMux(curColumn, mux_value, mux_signal, reinterpret_cast<MuxCallback> (&MuxCallbackValueChecker));

          dbstat = kvaDbGetSignalUnit(sh, unit, sizeof(unit));
          if (dbstat != kvaDbOK) {
            break;
          }

          sr->setUnit(curColumn, unit);

          double factor, offset;
          dbstat = kvaDbGetSignalValueScaling(sh, &factor, &offset);
          if (dbstat != kvaDbOK) {
            break;
          }
          sr->setScaling(curColumn, factor, offset);

          double minval, maxval;
          dbstat = kvaDbGetSignalValueLimits(sh, &minval, &maxval);
          if (dbstat != kvaDbOK) {
            break;
          }
          sr->setLimits(curColumn, minval, maxval);

          sr->setMessageId(curColumn, mh);

          curColumn++;

          dbstat = kvaDbGetNextSignal(mh, &sh);
          if (dbstat != kvaDbOK) {
            break;
          }
        }

        dbstat = kvaDbGetNextMsg(dh, &mh);
        if (dbstat != kvaDbOK) {
          break;
        }
      }
    }

    if (get_property_show_counter()) {
      sr->setName(curColumn, "Counter");
      sr->setFullName(curColumn, "Counter");
      sr->setUnit(curColumn, "");
    }

    curColumn++;
  }
  
  // Convert from microseconds to nanoseconds
  samplingTimeStep = get_property_sample_and_hold_timestep() * 1000LL;
  if (samplingTimeStep) {
    // Should only be done once
    if (!samplingSr) {
      samplingSr = new SignalRow(get_property_separator_char());
      samplingSr->copySignalValues(sr);
    }
  }

  // Switch SignalRow pointer if we are using sample & hold
  if (samplingSr) {
    original = sr;
    sr = samplingSr;
  }

  kvstatus = write_signal_header();

  // Restore SignalRow pointer if we are using sample & hold
  if (samplingSr) {
    sr = original;
  }

  return kvstatus;
}

//===========================================================================
KvlcStatus KvaLogWriter_Signal::write_event(imLogData *logEvent)
{
  return write_row(logEvent);
}

//----------------------------------------------------------------------------------------
// forward any incoming imLogData as a sequence of
//   push(column, signalvalue, numdecimal)
// ending with a
//   commit(time)
// which states that all pushes since last commit happend at given time

// This means that it is possible to skip the commit until we find a message
// that is treated as a time reference, which gives us the Resampling function
// Werner wants.
KvlcStatus KvaLogWriter_Signal::write_row(imLogData *logEvent)
{
  bool match = false;
  int curColumn = 0;
  KvlcStatus kvstatus = kvlcOK;


  if (!isOpened) {
    PRINTF(("KvaLogWriter_Signal::write_row, file is not opened\n"));
    return kvlcERR_FILE_ERROR;
  }

  if (!start_of_logging) setStartOfLogging(logEvent);

  switch (logEvent->common.type) {
    case ILOG_TYPE_MESSAGE: {
      time_int64 msgTime;

      if (!((1<<logEvent->msg.channel) & get_property_channel_mask())) {
        // dont use this channel so bail out
        return kvlcOK;
      }

      if (logEvent->msg.flags & canMSGERR_OVERRUN) {
        overrun_occurred = true;
      }

      if (logEvent->msg.flags & canMSG_ERROR_FRAME) {
        //length = sprintf(outbuffer, "%sErrorframe\n", outbuffer); match = true;
      } else {
        KvaDbHnd dh;
        KvaDbMessageHnd mh;
        KvaDbSignalHnd sh;
        KvaDbStatus dbstat;
        unsigned int dbflags = 0;
        unsigned int id;
        unsigned int chanmask;
        int dbdlc;
        char sig_name[256];
        char fullname[512];

        msgTime = ((time_int64) logEvent->common.time64 +
                           (time_int64) get_property_offset());

        // time
        sr->setTimeStamp(msgTime); // High resolution timer
        sr->setAbsTime(logEvent->common.nanos_since_1970); // ABS time

        sr->setChannel(logEvent->msg.channel);

        id = logEvent->msg.id;


        // to loop over databases except to be compliant to kvlcAddDatabase()

        for (uint32_t i=0; i < databases.size(); i++){
          dh = databases[i];
          chanmask = databases_chanmask[i]; // masking occurs both per database (kvadblib)
                                            // and as a kvlclib property


          // Find db message
          if (get_property_hlp_j1939()) {
            if ((logEvent->msg.flags & canMSG_EXT) != 0) {
              id |= KVADB_MESSAGE_EXT;
            }
            dbstat = kvaDbGetMsgByPGN(dh, id, &mh);
          } else {
            if ((logEvent->msg.flags & canMSG_EXT) != 0) {
              id &= ~KVADB_MESSAGE_EXT;
              dbflags = KVADB_MESSAGE_EXT;
            }
            dbstat = kvaDbGetMsgByIdEx(dh, id, dbflags, &mh);
          }

          // kvaDbGetMsgByIdEx() / kvaDbGetMsgByPGN() success
          if (dbstat == kvaDbOK){

            dbstat = kvaDbGetMsgDlc(mh, &dbdlc);
            if (dbstat != kvaDbOK) {
              PRINTF(("kvaDbGetMsgDlc failed %d\n", dbstat));
              return kvlcERR_INTERNAL_ERROR;
            }

            // Handle dlc mismatch
            if(dbdlc != logEvent->msg.dlc ){
              dlc_mismatch_occurred = true;
              update_dlc_mismatch_list(logEvent);
            }

            // for each signal sh
            dbstat = kvaDbGetFirstSignal(mh, &sh);
            if (dbstat != kvaDbOK) {
              if (dbstat == kvaDbErr_NoSignal){
                continue;
              } else {
                PRINTF(("kvaDbGetFirstSignal failed %d\n", dbstat));
                return kvlcERR_INTERNAL_ERROR;
              }
            }

            while (dbstat == kvaDbOK) {
              dbstat = kvaDbGetSignalName(sh, sig_name, sizeof(sig_name));
              if (dbstat != kvaDbOK) {
                PRINTF(("kvaDbGetSignalName failed %d\n", dbstat));
                return kvlcERR_INTERNAL_ERROR;
              }
              dbstat = kvaDbGetSignalQualifiedName(sh, fullname, sizeof(fullname));
              if (dbstat != kvaDbOK) {
                PRINTF(("kvaDbGetSignalName failed %d\n", dbstat));
                return kvlcERR_INTERNAL_ERROR;
              }


              curColumn = sigName2column[fullname];


              // begin sig/column check (multiplex check)
              if (chanmask & (1 << (logEvent->msg.channel)) && sr->inEvent(curColumn, reinterpret_cast<unsigned char*>(logEvent->msg.data), logEvent->msg.dlc)) {

                bool gotSignalValue = false;
                if (get_property_enum_values()) {
                  char buf[MAX_ELEM_LEN];
                  dbstat = kvaDbGetSignalValueEnum(sh, buf, sizeof(buf),
                                                  logEvent->msg.data,
                                                  logEvent->msg.dlc);
                  if (dbstat == kvaDbOK) {
                    gotSignalValue = true;
                    sr->setSignalValue(curColumn, buf);
                  }
                }
                if (!gotSignalValue) {
                  double  valdbl;

                  dbstat = kvaDbRetrieveSignalValuePhys(sh, &valdbl, logEvent->msg.data, logEvent->msg.dlc);
                  if (dbstat != kvaDbOK) {
                    PRINTF(("write_row: kvaDbRetrieveSignalValuePhys failed\n"));
                  } else {
                    sr->setSignalValue(curColumn, valdbl);
                  }
                }
                match = true;
              } // end sig/column check

              dbstat = kvaDbGetNextSignal(mh, &sh);
            } // end for each signal

            if (get_property_show_counter()) {
              sr->setSignalValue(curColumn, (double)counter);
              sr->setNumDecimals(curColumn, 0);
            }

          } // end kvaDbGetMsgByIdEx() / kvaDbGetMsgByPGN() success
          else if (dbstat != kvaDbErr_NoMsg){
            PRINTF(("kvaDbGetMsgByIdEx() / kvaDbGetMsgByPGN() failed %d\n", dbstat));
            return kvlcERR_INTERNAL_ERROR;
          }
        } // databases

        if (match) {
          bool write = true;
          // int col;


          if (get_property_merge_lines()) {
            write = sr->isUpdated();
          }

          if (write) {
            if (samplingTimeStep) {
              // Sample & Hold: Get start time from first matched signal
              // and write it to file.
              if (samplingAbsTime == 0) {
                samplingHighResTime = msgTime;
                samplingAbsTime     = logEvent->common.nanos_since_1970;
                kvstatus = write_signals();
                if (kvstatus != kvlcOK) return kvstatus;
              } else {
                // Sample & Hold: Write the previous signal values up to current
                // msgTime. Switch SignalRow pointers to write the previous value.
                SignalRow *original = sr;
                sr = samplingSr;
                while (samplingHighResTime + samplingTimeStep < msgTime) {
                  samplingHighResTime += samplingTimeStep;
                  samplingAbsTime     += samplingTimeStep;
                  samplingSr->setTimeStamp(samplingHighResTime); // High resolution timer
                  samplingSr->setAbsTime(samplingAbsTime);       // ABS time
                  kvstatus = write_signals();
                  if (kvstatus != kvlcOK) {
                    // Restore signal row pointer
                    sr = original;
                    return kvstatus;
                  }
                }
                // Restore signal row pointer
                sr = original;
              }
              // Save previous values
              samplingSr->copySignalValues(sr);
            }
            else {
              kvstatus = write_signals();
              if (kvstatus != kvlcOK) return kvstatus;
            }
            if ( split_on_time(msgTime) ) {
              kvstatus = split_file();
            }
            // Since we have printed the row, we reset its updated-status
            sr->resetUpdated();
          }
          counter++;
        }
      }
      break;
    }
    case ILOG_TYPE_TRIGGER:
    case ILOG_TYPE_RTC:
    case ILOG_TYPE_VERSION:
    {
      break;
    }

    default:
    {
      PRINTF(("logEvent->common.type = %d, frame_counter = %lu\n",
              logEvent->common.type,
              logEvent->common.event_counter));
      break;
    }

  }
  logEvent->common.new_data = false;


  return kvstatus;
}

bool KvaLogWriter_Signal::get_property_full_names()
{
  return (mProperties.mShowFullyQualifiedNames == PROPERTY_ON);
}


unsigned int KvaLogWriter_Signal::get_property_sample_and_hold_timestep()
{
  // 
  return mProperties.mSampleAndHoldTimestep;
}

void KvaLogWriter_Signal::update_dlc_mismatch_list(imLogData *logEvent){
  uint32_t uiSize= (uint32_t)DlcWarningsList.size();
  int AddToList = 1;
  for (unsigned i = 0; i < uiSize; i++) {
    if ((DlcWarningsList.at(i).at(0) == (unsigned int)logEvent->msg.id) && (DlcWarningsList.at(i).at(1) == (unsigned int)logEvent->msg.dlc)) {
      DlcWarningsList.at(i).at(2) = DlcWarningsList.at(i).at(2) + 1;
      AddToList = 0;
    }
  }
  if (AddToList) {
    DlcWarningsList.push_back(std::vector<uint32_t>()); // Add an empty row
    DlcWarningsList[DlcWarningsList.size() - 1].push_back(logEvent->msg.id);
    DlcWarningsList[DlcWarningsList.size() - 1].push_back(logEvent->msg.dlc);
    DlcWarningsList[DlcWarningsList.size() - 1].push_back(1);
  }
}

KvlcStatus KvaLogWriter_Signal::get_list_dlc_mismatch(uint32_t* MsgIds, uint32_t* MsgDlc, uint32_t* MsgOccurance, uint32_t* length)
{
  PRINTF(("KvaLogWriter_Signal::get_list_dlc_mismatch\n"));
  if(dlc_mismatch_occurred)
  {
    uint32_t uiSize = (uint32_t)DlcWarningsList.size();
    if (*length == 0) {
      *length = uiSize;
      return kvlcOK;
    }

    uint32_t uiLen = !(uiSize < *length) ? *length : uiSize; //Get the min value

    for (unsigned i = 0; i < uiLen; i++) {
      MsgIds[i] = DlcWarningsList.at(i).at(0);
      MsgDlc[i] = DlcWarningsList.at(i).at(1);
      MsgOccurance[i] = DlcWarningsList.at(i).at(2);
    }
    *length = uiSize;
  }
  return kvlcOK;
}
