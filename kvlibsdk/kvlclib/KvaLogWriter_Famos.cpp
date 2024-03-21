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
**   Writer for FAMOS, the text/binary DASA Standard Format (DSF) once defined
**   by GOULD Inc.
** ---------------------------------------------------------------------------
*/

#include "common_defs.h"
#include "KvaLogWriter_Famos.h"
#include "TimeConv.h"
#include "kvdebug.h"


#define idMask                0x1FFFFFFF
#define SIGNAL_BUFFER_SIZE    2048

// Don't change the numerical values of these defines, they are according to FAMOS
#define FAMOS_UNSIGNED_FORMAT 5
#define FAMOS_SIGNED_FORMAT   6
#define FAMOS_FLOAT_FORMAT    7
#define FAMOS_DOUBLE_FORMAT   8

#define FAMOS_CHAR_BUFFER 1024
//--------------------------------------------------------------------------
KvaLogWriter_Famos::KvaLogWriter_Famos()
{
  PRINTF(("KvaLogWriter_Famos::KvaLogWriter_Famos()\n"));
  indCb = 0;
}

//--------------------------------------------------------------------------
KvaLogWriter_Famos::~KvaLogWriter_Famos()
{
  PRINTF(("~KvaLogWriter_Famos\n"));
}

//--------------------------------------------------------------------------
KvlcStatus KvaLogWriter_Famos::open_file()
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

  //hFile = fopen(filename, "wb");
  hFile = utf_fopen(filename, "wb");
  if (hFile == NULL) {
    PRINTF(("fopen %s failed.", filename));
    return kvlcERR_FILE_ERROR;
  }
  PRINTF(("fopen succeeded"));

  isOpened = true;

  return kvlcOK;
}

//--------------------------------------------------------------------------
KvlcStatus KvaLogWriter_Famos::write_signals()
{
  int i;
  int groupNo = -1;

  SampleType sample;
  sample.data.dval  = sr->getTimeStamp() / (double)ONE_BILLION;
  char signame[FAMOS_CHAR_BUFFER];
  // Find the group that the signal belongs to
  for (i = 0; i < sr->getSize(); i++) {
    if (sr->isSignalDefined(i)) {
      groupNo = findGroup(i);
      break;
    }
  }

  // Not found; create a new group and add all signals that belongs to this message
  if (groupNo < 0 ) {
    GroupType gp;
    size_t len;
    uint64_t messageId;

    // Add time signal to the new group
    gp.id = (int)groups.size();
    PRINTF(("Create a new group %d.\n", gp.id));
    sprintf(signame,"sgroup_%03d",gp.id);
    gp.buffered_time_samples = new BQueue(signame, SIGNAL_BUFFER_SIZE, sizeof(sample));
    if (!gp.buffered_time_samples->push(&sample)) {
      PRINTF(("write_row: couldn't push time sample to new group %d.",gp.id));
      return kvlcERR_INTERNAL_ERROR;
    }

    // Create time signal name from fullname
    sr->getFullName(signame, i);
    len = createTimeName(&gp, signame);
    gp.timeName = new char[len+1];
    strcpy(gp.timeName,signame);

    messageId = sr->getMessageId(i);

    // Add parameters within the same message to the new group
    for (int k=0; k < sr->getSize(); k++) {
      if (sr->getMessageId(k) == messageId) {
        signal_info_type si;

        si.index = k;
        si.dimension = 0;
        si.format   = FAMOS_DOUBLE_FORMAT;
        si.signBits = sizeof(double) * 8;
        si.noBytes  = sizeof(double);

        // The database has already converted the data, so this does not need
        // but we will put the information into the FAMOS file, but set the
        // conversion field to FALSE.

        si.factor = sr->getFactor(k);
        si.offset = sr->getOffset(k);

        // Create a queue and add the first value
        sample.data.dval = sr->getSignalValue(k);
        sr->getName(signame, k);
        PRINTF(("Adding signal '%s' from column %d to group %d.\n", signame,k,gp.id));
        si.buffered_samples =  new BQueue(signame, SIGNAL_BUFFER_SIZE, sizeof(sample));
        if (!si.buffered_samples->push(&sample)) {
          PRINTF(("write_row: couldn't push data sample"));
          return kvlcERR_INTERNAL_ERROR;
        }
        si.dimension++;

        // Add the signal to the group
        gp.signal_info.push_back(si);
      }
    }

    // Add the group to the list of groups and get next row
    groups.push_back(gp);
    PRINTF(("Group and signal creation finished.\n"));
    sr->resetDefined();
    return kvlcOK;
  }

  // The signal belongs to a group; update time and data for all signals in the group
  if (!groups[groupNo].buffered_time_samples->push(&sample)) {
    PRINTF(("write_row: couldn't push time sample to group %d.\n",groupNo));
    return kvlcERR_INTERNAL_ERROR;
  }

  for (unsigned int k=0; k < groups[groupNo].signal_info.size(); k++) {
    signal_info_type* psi = &groups[groupNo].signal_info[k];
    if (!sr->isSignalDefined(psi->index)) {
      sr->getName(signame, psi->index);
      PRINTF(("Warning: Missing a value for signal %s in group %d.\n", signame, groupNo));
    }
    sample.data.dval = sr->getSignalValue(psi->index);
    if (!psi->buffered_samples->push(&sample)) {
      PRINTF(("write_row: couldn't push data sample"));
      return kvlcERR_INTERNAL_ERROR;
    }
    psi->dimension++;
  }

  sr->resetDefined();
  return kvlcOK;
}



KvlcStatus KvaLogWriter_Famos::close_file()
{
  PRINTF(("KvaLogWriter_Famos::close_file()"));
  PRINTF(("start_of_logging = %lu", start_of_logging));

  char signame[FAMOS_CHAR_BUFFER];
  char unit[FAMOS_CHAR_BUFFER];
  size_t bytesToSkip;
  size_t size;

  KvlcStatus retval = kvlcOK;

  // If the fopen command failed
  if (!isOpened) {
    // Close the databases and files
    KvaLogWriter::close_file();
    return retval;
  }

  // Write the beginning of the file; always the same
  if (writeFamosHeader()) {
    PRINTF(("writeFamosHeader failed."));
    retval = kvlcERR_FILE_ERROR;
  }

  // Measurement start date & start time
  unix_time(start_of_logging / ONE_BILLION,
            &measDate.year, &measDate.month, &measDate.day,
            &measDate.hh, &measDate.mm, &measDate.ss);

  // Write each group to the FAMOS file
  PRINTF(("Write %lu groups to file.\n", groups.size()));
  for (unsigned int groupNo=0; groupNo < groups.size(); groupNo++) {
    GroupType *gp = &groups[groupNo];
    signal_info_type *psi = &gp->signal_info[0];

    // Write the FAMOS time channel (header depends on measDate);
    // ----------------------------------------------------------
    if (writeFamosChannelHeader()) {
      PRINTF(("writeFamosChannelHeader failed."));
      retval = kvlcERR_FILE_ERROR;
    }
    // Update unique buffer index
    indCb++;

    // This only works if all data is double; otherwise check the size for each.
    bytesToSkip = gp->signal_info.size() * sizeof(double);
    if (writeFamosTimeDescription(indCb , bytesToSkip)) {
      PRINTF(("writeFamosTimeDescription failed."));
      retval = kvlcERR_FILE_ERROR;
    }
    // Calculate signal buffer size; time is always 8 bytes
    size = (8+bytesToSkip) * psi->dimension;

    // Time buffer begins at the first byte, byte 0.
    if (writeFamosBufferDescription(gp,indCb, size, 0)) {
      PRINTF(("writeFamosBufferDescription failed."));
      retval = kvlcERR_FILE_ERROR;
    }
    // The calibration and unit for the time
    if (writeFamosCalibration(psi, "s")) {
      PRINTF(("writeFamosCalibration failed."));
      retval = kvlcERR_FILE_ERROR;
    }
    // The name of the time stamp must include the signalname
    //sprintf(timename,"Time_Stamp_%02d",groupNo+1);
    if (writeFamosParameterName(psi, gp->timeName, gp->timeName)) {
      PRINTF(("writeFamosParameterName failed."));
      retval = kvlcERR_FILE_ERROR;
    }

    PRINTF(("Write %lu signals with the group.\n",gp->signal_info.size()));
    for (unsigned int k = 0; k < gp->signal_info.size(); k++) {
      psi = &gp->signal_info[k];

      // Update unique buffer index
      indCb++;
      // Write the FAMOS data channel (header depends on measDate);
      // ----------------------------------------------------------
      if (writeFamosChannelHeader()) {
        PRINTF(("writeFamosChannelHeader failed."));
        retval = kvlcERR_FILE_ERROR;
      }
      // Time is 8 bytes
      if (writeFamosDataDescription(psi, indCb, bytesToSkip)) {
        PRINTF(("writeFamosDataDescription failed."));
        retval = kvlcERR_FILE_ERROR;
      }
      // Time buffer begins after the time (8 bytes)
      if (writeFamosBufferDescription(gp, indCb, size-8*(k+1), 8*(k+1))) {
        PRINTF(("writeFamosBufferDescription failed."));
        retval = kvlcERR_FILE_ERROR;
      }
      // The calibration and unit for the data
      // channel units field
      sr->getUnit(unit, psi->index);
      if (writeFamosCalibration(psi, unit)) {
        PRINTF(("writeFamosCalibration failed."));
        retval = kvlcERR_FILE_ERROR;
      }
      // The comment should be the time name for matching reasons
      sr->getName(signame, psi->index);
      if (writeFamosParameterName(psi, signame, gp->timeName)) {
        PRINTF(("writeFamosParameterName failed."));
        retval = kvlcERR_FILE_ERROR;
      }
    }
    // Write all data for the group to a file
    if (writeFamosDataBuffer(gp)) {
      PRINTF(("writeFamosDataBuffer failed."));
      retval = kvlcERR_FILE_ERROR;
    }
    // Free memory for this group
    for (unsigned int k = 0; k < gp->signal_info.size(); k++) {
      psi = &gp->signal_info[k];
      delete psi->buffered_samples;
    }
    delete gp->buffered_time_samples;
    delete gp->timeName;

  }

  groups.clear();

  // Close the databases and files
  KvaLogWriter::close_file();

  fclose(hFile);

  isOpened = false;
  hFile = NULL;

  return retval;
}

int KvaLogWriter_Famos::writeFamosHeader(void) {
  // Our FAMOS files begins with the exact same lines every time.
  // Note that the two first extries must be written on one line. This is
  // a feature/bug in one of the programs that read FAMOS files.
  bufLength = sprintf(lineBuffer,"|CF,2,1,1;|CK,1,3,1,1;\r\n|NO,1,18,1,8,KvaserAB,3,513;\r\n\r\n");
  return fwrite(lineBuffer,bufLength,1,hFile)  != 1;
}

int KvaLogWriter_Famos::writeFamosChannelHeader(void) {
  // 1-D real numbers (constant)
  fwrite(lineBuffer,sprintf(lineBuffer,"|CG,1,5,1,1,1;\r\n"),1,hFile);

  // Sample calibration data, i.e. equidistant (constant)
  bufLength=sprintf(lineBuffer,"|CD,1,32,1.00000000000000E+00,1,1,n,0,0,0;\r\n");
  fwrite(lineBuffer,bufLength,1,hFile);

  // The trigger date and time
  bufLength = sprintf(dataBuffer,"%u,%u,%u,%u,%u,%.2f",measDate.day,
                      measDate.month,measDate.year,measDate.hh,measDate.mm,(double)measDate.ss);
  bufLength = sprintf(lineBuffer,"|NT,1,%d,%s;\r\n", bufLength, dataBuffer);
  fwrite(lineBuffer,bufLength,1,hFile);

  // An analog, real measurement (constant)
  bufLength=sprintf(lineBuffer,"|CC,1,3,1,1;\r\n");
  return fwrite(lineBuffer,bufLength,1,hFile)  != 1;
 }

int KvaLogWriter_Famos::writeFamosTimeDescription(int Cb, size_t skip) {
  // This entry describes how the data is organized inside the sub-buffer
  // that will be created by Cb.
  int offset = 0;        // The sub-buffer will start at the first sample,
                         // hence no offset here.
  int noConSamples = 1;  // The number of consecutive samples (constant)
  int mask         = 0;  // No masking is done (constant)

  int format       = 8;  // FAMOS double
  int noBytes      = 8;  // Time is double with 8 bytes
  int sigBits      = 64; // All bits used
  bufLength = sprintf(dataBuffer,"%d,%d,%d,%d,%d,%d,%d,%d", Cb, noBytes,
                      format,sigBits, mask, offset, noConSamples, (int)skip);
  bufLength = sprintf(lineBuffer, "|CP,1,%d,%s;\r\n",bufLength,dataBuffer);
  return fwrite(lineBuffer,bufLength,1,hFile)  != 1;
}

int KvaLogWriter_Famos::writeFamosDataDescription(signal_info_type* si, int Cb, size_t skip) {
  // This entry describes how the data is organized inside the sub-buffer
  // that will be created by Cb.
  int offset = 0;       // The sub-buffer will start at the first sample,
                        // hence no offset here.
  int noConSamples = 1; // The number of consecutive samples (constant)
  int mask         = 0; // No masking is done (constant)

  bufLength = sprintf(dataBuffer,"%d,%d,%d,%d,%d,%d,%d,%d", Cb, si->noBytes,
                      si->format,si->signBits, mask, offset, noConSamples, (int)skip);
  bufLength = sprintf(lineBuffer, "|CP,1,%d,%s;\r\n",bufLength,dataBuffer);
  return fwrite(lineBuffer,bufLength,1,hFile)  != 1;
}

int KvaLogWriter_Famos::writeFamosBufferDescription(GroupType *gp, int CP, size_t size, int offset) {
  // This entry describes how the sub-buffer organized inside the binary data buffer
  // that will be created by CS.
  bufLength = sprintf(dataBuffer,"1,0,%d,%d,%d,%d,%d,%d,0,0,0,", CP,gp->id+1,offset,(int)size,0,(int)size);
  bufLength = sprintf(lineBuffer, "|Cb,1,%d,%s;\r\n",bufLength,dataBuffer);
  return fwrite(lineBuffer,bufLength,1,hFile)  != 1;
}

int KvaLogWriter_Famos::writeFamosCalibration(signal_info_type* /* si */, const char *unit) {
  // Calibration and scaling
  int    fConvert = 1; // Should data be converted?
  double factor   = 1; // y=factor*x+offset
  double offset   = 0; //
  int fCalibrated = 1; // Is the parameter calibrated?
  bufLength = sprintf(dataBuffer, "%d,%.14G,%.14G,%d,%d,%s",fConvert,factor,offset,fCalibrated,
                      (int)strlen(unit), unit);
  bufLength = sprintf(lineBuffer, "|CR,1,%d,%s;\r\n",bufLength,dataBuffer);
  return fwrite(lineBuffer,bufLength,1,hFile)  != 1;
}

int KvaLogWriter_Famos::writeFamosParameterName(signal_info_type* /* si */, char *name, char *note) {
  // Name and comment. Note the some programs require that the time sub-channel has
  // the same name and comment. The data sub-channel should then have the time
  // sub-channel's name as comment, e.g. {"t","t"} {"x","t"}.
  bufLength = sprintf(dataBuffer,"0,0,0,%d,%s,%d,%s", (int)strlen(name), name, (int)strlen(note), note);
  bufLength = sprintf(lineBuffer, "|CN,1,%d,%s;\r\n\r\n",bufLength, dataBuffer);
  return fwrite(lineBuffer,bufLength,1,hFile)  != 1;
}

int KvaLogWriter_Famos::writeFamosDataBuffer(GroupType* gp) {

  size_t size = 0;
  SampleType sample;
  signal_info_type *psi = &gp->signal_info[0];

  bufLength = sprintf(lineBuffer, ",%d", gp->id+1);
  // This uses the fact that all parameters are doubles
  size = psi->dimension * (gp->signal_info.size() * psi->noBytes+8) + bufLength;
  bufLength = sprintf(lineBuffer, "|CS,1,%d,%d,", (int)size, gp->id+1);
  fwrite(lineBuffer,bufLength,1,hFile);

  // Write each row of values to the buffer.
  while (gp->buffered_time_samples->size() != 0) {
    if (!gp->buffered_time_samples->pop(&sample)) {
      PRINTF(("Failed to pop time from buffer"));
      return -1;
    }
    fwrite(&(sample.data.dval),8,1,hFile);
    for (unsigned int k=0; k < gp->signal_info.size();k++) {
      psi = &gp->signal_info[k];
      if (!psi->buffered_samples->pop(&sample)) {
        PRINTF(("Failed to pop data from signal buffer."));
        return -1;
      }
      switch (psi->format) {
        case FAMOS_UNSIGNED_FORMAT:
          fwrite(&(sample.data.uval),psi->noBytes,1,hFile);
          break;
        case FAMOS_SIGNED_FORMAT:
          fwrite(&(sample.data.ival),psi->noBytes,1,hFile);
          break;
        case FAMOS_FLOAT_FORMAT:
          fwrite(&(sample.data.fval),psi->noBytes,1,hFile);
          break;
        case FAMOS_DOUBLE_FORMAT:
          fwrite(&(sample.data.dval),psi->noBytes,1,hFile);
          break;
        }
      }
    }
  bufLength = sprintf(lineBuffer, ";\r\n\r\n");
  return fwrite(lineBuffer,bufLength,1,hFile) != 1;
}
// -------------------------------------------------------------------------
int KvaLogWriter_Famos::findGroup(int signalindex) {
  // Find the (first) group that a parameter belongs to
  // or return -1 if not found.
  GroupType *gp;
  signal_info_type *psi;
  for (unsigned int groupNo=0; groupNo < groups.size(); groupNo++) {
    gp = &groups[groupNo];
    for (unsigned int k = 0; k < gp->signal_info.size(); k++) {
      psi = &gp->signal_info[k];
      if (psi->index == signalindex) {
        return gp->id;
      }
    }
  }
  return -1;
}
// -------------------------------------------------------------------------
size_t KvaLogWriter_Famos::createTimeName(GroupType *gp, char *fullname) {
  // Convert the full name to a Famos time signal name.
  char tmp0[FAMOS_CHAR_BUFFER], tmp1[FAMOS_CHAR_BUFFER], tmp2[FAMOS_CHAR_BUFFER];
  int len;

  tmp0[0] = '\0'; tmp1[0] = '\0'; tmp2[0] = '\0';

  // The full name is database.message.signal
  len = sscanf(fullname, "%[^.].%[^.].%s", tmp0,tmp1,tmp2);
  if (3 == len && strlen(tmp1)) {
    sprintf(fullname,"Time_stamp_%s",tmp1);
  } else {
    sprintf(fullname,"Time_stamp_%02d",gp->id+1);
  }
  return strlen(fullname);
}


#define NAME        "FAMOS"
#define EXTENSION   "dat"
#define DESCRIPTION "Selected signals in FAMOS format"

class KvaWriterMaker_Famos : public KvaWriterMaker
{
  public:
    KvaWriterMaker_Famos() : KvaWriterMaker(KVLC_FILE_FORMAT_FAMOS) {
      propertyList[KVLC_PROPERTY_START_OF_MEASUREMENT] = true;
      propertyList[KVLC_PROPERTY_FIRST_TRIGGER] = true;
      propertyList[KVLC_PROPERTY_CHANNEL_MASK] = true;
      propertyList[KVLC_PROPERTY_HLP_J1939] = true;
      propertyList[KVLC_PROPERTY_SIGNAL_BASED] = true;
      propertyList[KVLC_PROPERTY_SHOW_SIGNAL_SELECT] = true;
      propertyList[KVLC_PROPERTY_CROP_PRETRIGGER] = true;
      propertyList[KVLC_PROPERTY_OVERWRITE] = true;
    }
    int getName(char *str) { return sprintf(str, "%s", NAME); }
    int getExtension(char *str) { return sprintf(str, "%s", EXTENSION); }
    int getDescription(char *str) { return sprintf(str, "%s", DESCRIPTION); }

  private:
    KvaLogWriter *OnCreateWriter() {
      return new KvaLogWriter_Famos();
    }
}  registerKvaLogWriter_Famos;

