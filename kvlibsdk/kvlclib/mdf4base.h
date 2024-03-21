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
**  Classes for writing ASAM MDF (Vector), supports ver 4.01
** -----------------------------------------------------------------------------
*/

#ifndef MDF4BASE_H
#define MDF4BASE_H

#include <stdlib.h>
#include <stdio.h>
#include <string>

#include "MdfTypes.h"
#include "BufQueue.h"
#include "MuxChecker.h"


// -----------------------------------------------------------------------------
// String constants
// -----------------------------------------------------------------------------
#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)


// -----------------------------------------------------------------------------
// String constants
// -----------------------------------------------------------------------------
#define MDF_FILE_ID             "MDF     "
#define MDF_VERSION_STRING_410  "4.10    "


// Kvaser version
#define MDF_PROGRAM_ID          "Kvaser"
#define MDF_PROGRAM_ID_MAJOR 1
#define MDF_PROGRAM_ID_MINOR 0
#define MDF_PROGRAM_ID_STRING MDF_PROGRAM_ID TOSTRING(MDF_PROGRAM_ID_MAJOR) TOSTRING(MDF_PROGRAM_ID_MINOR)

// Own definitions
typedef uint64_t  KVMDF_TIMESTAMP;
#define MDF_FALSE (MDF_BOOL)0
#define MDF_TRUE  (MDF_BOOL)~MDF_FALSE
#define MDF_MAX_DLC 64
#define MDF_DT_MAX_SIZE (4096*1000)

// -----------------------------------------------------------------------------
// Signal data types used by converter
// -----------------------------------------------------------------------------
// Default byte order from IDBLOCK
#define MDF_SIGNAL_DATA_TYPE_UNSIGNED_INT     0
#define MDF_SIGNAL_DATA_TYPE_SIGNED_INT       1 // two complements
#define MDF_SIGNAL_DATA_TYPE_IEEE_FLOAT       2 // 4 bytes
#define MDF_SIGNAL_DATA_TYPE_IEEE_DOUBLE      3 // 8-10 bytes
// VAX floating point (4-6) obsolete
// The following types are only supported in >= MDF 2.15
#define MDF_SIGNAL_DATA_TYPE_STRING           7 // NULL terminated
#define MDF_SIGNAL_DATA_TYPE_BYTE_ARRAY       8 // max 8191 bytes, constant record length
// The following types are only supported in >= MDF 3.10
// Big Endian
#define MDF_SIGNAL_DATA_TYPE_UNSIGNED_INT_BE  9
#define MDF_SIGNAL_DATA_TYPE_SIGNED_INT_BE   10 // two complements
#define MDF_SIGNAL_DATA_TYPE_IEEE_FLOAT_BE   11 // 4 bytes
#define MDF_SIGNAL_DATA_TYPE_IEEE_DOUBLE_BE  12 // 8-10 bytes
// Little Endian
#define MDF_SIGNAL_DATA_TYPE_UNSIGNED_INT_LE 13
#define MDF_SIGNAL_DATA_TYPE_SIGNED_INT_LE   14 // two complements
#define MDF_SIGNAL_DATA_TYPE_IEEE_FLOAT_LE   15 // 4 bytes
#define MDF_SIGNAL_DATA_TYPE_IEEE_DOUBLE_LE  16 // 8-10 bytes

// -----------------------------------------------------------------------------
// Conversion types used by converter
// -----------------------------------------------------------------------------
// The Channel Conversion Block CCBLOCK
#define MDF_CONVERSION_TYPE_PARAMETRIC_LINEAR 0
// No support for formulas 1-65534
#define MDF_CONVERSION_TYPE_ONE_TO_ONE        65535


// -----------------------------------------------------------------------------
// Option class for all nodes (used as singleton)
// -----------------------------------------------------------------------------
class Mdf4Options {
public:
  bool useVariableRecordSize;
  bool useFixedRecordSize64;
  bool useDataListNodes;
  bool useZippedData;

  Mdf4Options() {
    useVariableRecordSize  = true;
    useFixedRecordSize64   = false;
    useDataListNodes       = true;
    useZippedData          = true;
  }
};

// -----------------------------------------------------------------------------
// Base class for all block nodes
// -----------------------------------------------------------------------------
class NodeBase {
private:
    MDF_LINK64 current_file_pos;
    int        version;
    uint64_t   last_size;
    bool       size_dirty;
public:
    NodeBase() { version = -1; size_dirty = true; current_file_pos= 0; data_size = 0;};
    virtual ~NodeBase() {};
    void setVersion(int ver) { if (version == -1) { version = ver; } }
    int getVersion() { return version; }
    virtual int write(FILE *mdfFile) = 0;
    MDF_LINK64  getCurrentFilePos() { return current_file_pos; }
    void setCurrentFilePos(MDF_LINK64 new_pos) { current_file_pos = new_pos; }
    virtual uint64_t size() = 0;
    bool isSizeDirty() { return size_dirty; }
    void setSizeDirty(bool dirty = true) { size_dirty = dirty; }
    void setSize(int64_t s) { last_size = s; setSizeDirty(false);}
    uint64_t getLastSize() { return last_size; }
    static Mdf4Options options;
    MDF_UINT64 data_size;
};

// -----------------------------------------------------------------------------
// Interface class used by converter
// -----------------------------------------------------------------------------
class MdfBase {
public:
  virtual int create(const char *filename) = 0;
  virtual void create(FILE *fh) = 0;
  virtual int write_header() = 0;
  virtual int write_data() = 0;

  virtual MdfStatus new_dg(MDF_UINT32 canId, MDF_UINT8 dlc = 8) = 0;
  virtual MdfStatus new_dg(MDF_UINT32 canId, MDF_UINT32 canMask, const MuxChecker& mux, MDF_UINT8 dlc = 8, const std::string& msgname = std::string(), int channel=1) = 0;
  virtual MdfStatus new_sig(MDF_UINT32 canId,
              char *longname,
              char *shortname,
              char *unit,
              MDF_UINT16 StartOffset,
              MDF_UINT16 NumBits,
              MDF_UINT16 SigDataType,
              MDF_UINT16 ChannelConversionType = MDF_CONVERSION_TYPE_ONE_TO_ONE,
              MDF_REAL Factor = 1.0,
              MDF_REAL Offset = 0.0) = 0;

  virtual int setStartOfRecording(
    unsigned int year,
    unsigned char month,
    unsigned char day,
    unsigned char hour,
    unsigned char minute,
    unsigned char second
  ) = 0;
  virtual int addMsg(MDF_UINT32 id, KVMDF_TIMESTAMP ts, MDF_UINT8 *data) = 0;

  virtual uint64_t size() = 0;

  virtual int close() = 0;

  virtual int initAndSetVersion(int version) = 0;

  virtual int addFrame(int channel, int type, KVMDF_TIMESTAMP ts, void *data) = 0;

  virtual int attach(const char *filename) = 0;

  virtual MdfStatus useVariableRecordSize(bool on) = 0;

  virtual MdfStatus useFixedRecordSize64(bool on) = 0;

  virtual MdfStatus useDataListNodes(bool on) = 0;

  virtual MdfStatus useZippedData(bool on) = 0;

  virtual ~MdfBase() {};
};

// -----------------------------------------------------------------------------
// MDF v4.x conversion
// -----------------------------------------------------------------------------
namespace Mdf4Nodes {
  class idNode;
  class hdNode;
}

class Mdf4 : public MdfBase {

  FILE *mdfFile;

  Mdf4Nodes::idNode *id;
  Mdf4Nodes::hdNode *hd;

  int version;

public:
  int create(const char *filename);
  void create(FILE *fh);
  int write_header();
  int write_data();

  int setStartOfRecording(
    unsigned int year,
    unsigned char month,
    unsigned char day,
    unsigned char hour,
    unsigned char minute,
    unsigned char second
  );
  int addFrame(int channel, int type, KVMDF_TIMESTAMP ts, void *data);

  int attach(const char *filename);

  MdfStatus useVariableRecordSize(bool on);
  MdfStatus useFixedRecordSize64(bool on);
  MdfStatus useDataListNodes(bool on);
  MdfStatus useZippedData(bool on);

  uint64_t size();

  int close();

  int initAndSetVersion(int version);

  void setCompressionLevel(int level);
  Mdf4();
  ~Mdf4();

  #define MDF_CAN_FRAME_TYPE    0
  #define MDF_ERROR_FRAME_TYPE  1
  #define MDF_REMOTE_FRAME_TYPE 2

    // Not used for CAN frames
  MdfStatus new_dg(MDF_UINT32 canId, MDF_UINT8 dlc = 8);
  MdfStatus new_dg(MDF_UINT32 canId, MDF_UINT32 canMask, const MuxChecker& mux, MDF_UINT8 dlc = 8, const std::string& msgname = std::string(), int channel = 1);
  MdfStatus new_sig(MDF_UINT32 canId,
              char *longname,
              char *shortname,
              char *unit,
              MDF_UINT16 StartOffset,
              MDF_UINT16 NumBits,
              MDF_UINT16 SigDataType,
              MDF_UINT16 ChannelConversionType = MDF_CONVERSION_TYPE_ONE_TO_ONE,
              MDF_REAL Factor = 1.0,
              MDF_REAL Offset = 0.0);
  int addMsg(MDF_UINT32 /* id */ , KVMDF_TIMESTAMP /* ts */, MDF_UINT8 * /* data */) {return -1;}; // Not used

  private:
  static const unsigned long MDF_OLD_MAX_SIZE = (unsigned long) -1;

};

#endif
