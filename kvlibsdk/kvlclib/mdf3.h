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
**  Classes for writing Bosch MDF (Vector), supports ver 3.20, 3.00 and 2.03
** -----------------------------------------------------------------------------
*/

#ifndef MDF3_H
#define MDF3_H


#include <stdlib.h>
#include <stdio.h>

#include "BufQueue.h"
#include "mdfbase.h"

#include <pshpack1.h>

// 3.2 Overview of Block Types Used
/*
IDBLOCK   Identification block
HDBLOCK   Header block
TXBLOCK   Text block
PRBLOCK   Program block
DGBLOCK   Data group block
CGBLOCK   Channel group block
CNBLOCK   Channel block
CCBLOCK   Channel conversion block
TRBLOCK   Trigger block
CDBLOCK   Channel dependency block
CEBLOCK   Channel Extension block
*/
namespace Mdf3Nodes {
#define MDF_ID(x,y) (x+y*256)

// 3.3 The File Identification Block IDBLOCK
typedef struct {
  MDF_CHAR    FileIdentifier[8];            // "MDF     "
  MDF_CHAR    FormatIdentifier[8];          // "3.20    "
  MDF_CHAR    ProgramIdentifier[8];         // "Kvaser  "
  MDF_UINT16  DefaultByteOrder;             // MDF_LITTLE_ENDIAN
  #define MDF_LITTLE_ENDIAN 0
  #define MDF_BIG_ENDIAN    1
  MDF_UINT16  DefaultFloatingPointFormat;   // 0 = IEEE 754
  MDF_UINT16  VersionNumber;                // 320
  MDF_UINT16  Reserved1;
  MDF_CHAR    Reserved2[2];
  MDF_CHAR    Reserved3[30];
} IDBLOCK;

typedef struct {
  // MDF_CHAR    BlockTypeIdentifier[2];
  MDF_UINT16  BlockTypeId;
  MDF_UINT16  BlockSize;
} BL_HEAD;

// 3.4 The Header Block HDBLOCK
#define MDF_ID_HDBLOCK MDF_ID('H','D')
typedef struct {
  BL_HEAD     Head;                         // "HD"
                                            // sizeof(HDBLOCK)
  MDF_LINK    FirstDataGroupBlock;          // Pointer to the first data group block (DGBLOCK)
  MDF_LINK    MeasurementFileComment;       // Pointer to the measurement file comment text (TXBLOCK) (NIL allowed)
  MDF_LINK    ProgramBlock;                 // Pointer to the program block (PRBLOCK) (NIL allowed)
  MDF_UINT16  NumberOfDataGroups;           // Number of data groups (redundant information)
  MDF_CHAR    Date[10];                     // Date at which the recording was started in "DD:MM:YYYY" format
  MDF_CHAR    Time[8];                      // Time at which the recording was started in "HH:MM:SS" format for locally displayed time, i.e. considering the daylight saving time (DST)
  MDF_CHAR    Author[32];                   // Author's name
  MDF_CHAR    Organization[32];             // Name of the organization or department
  MDF_CHAR    Project[32];                  // Project name
  MDF_CHAR    Subject[32];                  // Subject / Measurement object, e.g. vehicle information

  // The following data was added since 3.20 and only their default is noted here
  MDF_UINT64  TimeStamp;                    // 0
  MDF_INT16   UtcTimeOffset;                 // 0
  MDF_UINT16  TimeQualityClass;             // 0
  MDF_CHAR    TimerIdentification[32];      // ""

} HDBLOCK;

// 3.5 The Text Block TXBLOCK
#define MDF_ID_TXBLOCK MDF_ID('T','X')
typedef struct {
  BL_HEAD     Head;                         // "TX"
                                            // sizeof TXBLOCK, may vary
  MDF_CHAR    *Text;                        // Text, new line: CR+LF, end of text: \0
} TXBLOCK;

// 3.6 The Program-Specific Block PRBLOCK
#define MDF_ID_PRBLOCK MDF_ID('P','R')
typedef struct {
  BL_HEAD     Head;                         // "PR"
                                            // sizeof PRBLOCK, may vary
  MDF_CHAR    *Text;                        // Text, new line: CR+LF, end of text: \0
} PRBLOCK;


// 3.7 The Trigger Block TRBLOCK
typedef struct {
  MDF_REAL    TriggerTime;
  MDF_REAL    PreTriggerTime;
  MDF_REAL    PostTriggerTime;
} TREVENT;

#define MDF_ID_TRBLOCK MDF_ID('T','R')
typedef struct {
  BL_HEAD Head;                         // "TR"
                                        // sizeof TRBLOCK, may vary
  MDF_LINK    TriggerComment;               // Pointer to trigger comment text (TXBLOCK)
  MDF_UINT16  NumberOfTriggerEvents;        // Number of trigger events (0 allowed)
  TREVENT *TriggerEvent;                // Trigger events
} TRBLOCK;

// 3.8 The Data Group Block DGBLOCK
#define MDF_ID_DGBLOCK MDF_ID('D','G')
typedef struct {
  BL_HEAD Head;                             // "DG"
                                            // sizeof(DGBLOCK)
  MDF_LINK    NextDataGroupBlock;           // Pointer to next data group block (DGBLOCK) (NIL allowed)
  MDF_LINK    FirstChannelGroup;            // Pointer to first channel group block (CGBLOCK) (NIL allowed)
  MDF_LINK    TriggerBlock;                 // Pointer to trigger block (TRBLOCK) (NIL allowed)
  MDF_LINK    DataBlock;                    // Pointer to the data block
  MDF_UINT16  NumberOfChannelGroups;        // Number of channel groups (redundant information)
  MDF_UINT16  NumberOfRecordIDs;            // Number of record IDs in the data block
  #define MDF_ID_DATA_RECORDS_WITHOUT_ID  0
  #define MDF_ID_BEFORE_DATA_RECORD       1
  #define MDF_ID_AFTER_DATA_RECORD        2
  MDF_UINT32  Reserved;
} DGBLOCK;

// 3.9 The Channel Group Block CGBLOCK
#define MDF_ID_CGBLOCK MDF_ID('C','G')
typedef struct {
  BL_HEAD Head;                             // "CG"
                                            // sizeof(CGBLOCK)
  MDF_LINK    NextChannelGroup;             // Pointer to the next channel group block (CGBLOCK) (NIL allowed)
  MDF_LINK    FirstChannelBlock;            // Pointer to the first channel block (CNBLOCK) (NIL allowed)
  MDF_LINK    Comment;                      // Pointer to the channel group comment text (TXBLOCK) (NIL allowed)
  MDF_UINT16  RecordID;                     // RecordID, i.e. value of the idientifier for a record if the DGBLOCK defines a number of record IDs > 0
  MDF_UINT16  NumberOfChannels;             // Number of channels (redundant information)
  MDF_UINT16  SizeOfDataRecord;             // Size of data record in Bytes (without record ID), i.e. size of plain data for a each recorded sample of this channel group
  MDF_UINT32  NumberOfRecords;              // Number of record of this type in the data block, i.e. number of samples for this channel group
} CGBLOCK;

// 3.10 The Channel Block CNBLOCK
#define MDF_ID_CNBLOCK MDF_ID('C','N')
typedef struct {
  BL_HEAD Head;                             // "CN"
                                            // sizeof(CNBLOCK)
  MDF_LINK    NextChannelBlock;
  MDF_LINK    ConversionFormula;
  MDF_LINK    SourceDependingExtensions;
  MDF_LINK    DependencyBlock;
  MDF_LINK    Comment;
  MDF_UINT16  ChannelType;
  #define MDF_CHANNEL_TYPE_DATA 0
  #define MDF_CHANNEL_TYPE_TIME 1
  MDF_CHAR    ShortSignalName[32];
  MDF_CHAR    SignalDescription[128];
  MDF_UINT16  StartOffsetInBits;
  MDF_UINT16  NumberOfBits;
  MDF_UINT16  SignalDataType;
  MDF_BOOL    ValidRangeFlag;
  MDF_REAL    MinSignalValue;
  MDF_REAL    MaxSignalValue;
  MDF_REAL    SamplingRate;

  // The following were added to the Mdf standard with version 3.00
  MDF_LINK    TX_LongSignalName;
  MDF_LINK    TX_DisplayName;
  MDF_UINT16  AdditionalByteOffset;
} CNBLOCK;

// 3.11 The Channel Conversion Block CCBLOCK
#define MDF_ID_CCBLOCK MDF_ID('C','C')
typedef struct {
  BL_HEAD     Head;                           // "CC" & BlockSize (may vary)
  MDF_BOOL    PhysicalValueRangeValidFlag;
  MDF_REAL    MinPhysValue;
  MDF_REAL    MaxPhysValue;
  MDF_CHAR    PhysicalUnit[20];
  MDF_UINT16  ConversionType;
  // #define MDF_CONVERSION_TYPE_PARAMETRIC_LINEAR 0
  // No support for formulas 1-65534
  //#define MDF_CONVERSION_TYPE_ONE_TO_ONE        65535
  MDF_UINT16  SizeInformation;
  // #define MDF_CONVERSION_SIZE_INFORMATION_PARAMETRIC_LINEAR 2
  // #define MDF_CONVERSION_SIZE_INFORMATION_ONE_TO_ONE        0
  MDF_REAL    P1;
  MDF_REAL    P2;
} CCBLOCK;

#include <poppack.h>

class ccNode : public NodeBase {
public:
  CCBLOCK     cc;

  ccNode(int version, const char *unit, MDF_UINT16 type, MDF_REAL p1=0.0, MDF_REAL p2=1.0);
  virtual ~ccNode();

  int write(FILE *mdfFile);
  unsigned long size();
};

class txNode : public NodeBase {
public:
  TXBLOCK   tx;
  txNode(int version, char *str);
  virtual ~txNode();

  int write(FILE *mdfFile);
  unsigned long size();
};

class prNode : public NodeBase {
public:
  PRBLOCK   pr;
  prNode(int version, char *str);
  virtual ~prNode();

  int write(FILE *mdfFile);
  unsigned long size();
};

class cnNode : public NodeBase {
public:
  CNBLOCK   cn;
  txNode    *tx_comment;
  txNode    *tx_long_signal_name;
  ccNode    *cc; // channel conversion
  // ignores a lot of possible children here
  cnNode    *next;

  cnNode(int version,
         const char *long_name,
         const char *name,
         const char *unit,
         MDF_UINT16 StartOffset,
         MDF_UINT16 NumBits,
         MDF_UINT16 SigDataType,
         MDF_UINT16 ChannelConversionType = MDF_CONVERSION_TYPE_ONE_TO_ONE,
         MDF_REAL Factor = 1.0,
         MDF_REAL Offset = 0.0);
  virtual ~cnNode();

  int write(FILE *mdfFile);
  unsigned long size();
};

class trNode : public NodeBase {
public:
  TRBLOCK   tr;
  trNode(int version, int number_of_triggers, char *trigger_comment = NULL);
  virtual ~trNode();

  txNode    *comment;
  int write(FILE *mdfFile);
  unsigned long size();
};

class cgNode : public NodeBase {
public:
  CGBLOCK   cg;
  cnNode    *cn;
  txNode    *comment;
  cgNode    *next;
  cgNode(int version, char *cg_comment = NULL);
  virtual ~cgNode();

  MdfStatus new_cn(const char *long_name,
                    const char *name,
                    const char *unit,
                    MDF_UINT16 StartOffset,
                    MDF_UINT16 NumBits,
                    MDF_UINT16 SigDataType,
                    MDF_UINT16 ChannelConversionType = MDF_CONVERSION_TYPE_ONE_TO_ONE,
                    MDF_REAL Factor = 1.0,
                    MDF_REAL Offset = 0.0);
  void      increaseNumberOfMessages(int num = 1);

  int write(FILE *mdfFile);
  unsigned long size();
};

class dgNode : public NodeBase{

public:
  DGBLOCK   dg;
  cgNode    *cg;
  trNode    *tr;
  unsigned long dataRecords;
  dgNode    *next;
  dgNode(int version);
  virtual ~dgNode();

  MdfStatus new_cg(MDF_UINT8 dlc);

  int write(FILE *mdfFile);
  unsigned long size();

  MdfStatus setRecordSize(MDF_UINT8 recordSize);
  MdfStatus addMsg(MDF_UINT32 id, KVMDF_TIMESTAMP ts, MDF_UINT8 *data);
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
  int write_data(FILE *mdfFile);

  MuxChecker mux_checker;
  /* MuxChecker is used to organize those multiplexed signals, whose multiplexor value is same, to same dgNode.
   * E.g. if data is organized as [ mux [ value_sig1 ][ value_sig2 ] ], then two signals will share same multiplexor value,
   * thus there will be two signals in corresponding dgNode.
   * To create different dgNodes for signals with different mux_values is the only effective way of avoiding data duplication.*/
  MDF_UINT32 canId;
  MDF_UINT32 canMask;
  BQueue     *bq;
  MDF_UINT8  RecordSize;
  char       *record;
};

class hdNode : public NodeBase {
public:
  HDBLOCK hd;
  txNode *tx;
  prNode *pr;
  dgNode *dg, *cur_dg;
  MdfStatus new_dg(MDF_UINT32 canId, MDF_UINT32 canMask, const MuxChecker& mux, MDF_UINT8 dlc = 8);
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
  hdNode(int version);
  virtual ~hdNode();

  int setStartOfRecording(
    unsigned int year,
    unsigned char month,
    unsigned char day,
    unsigned char hour,
    unsigned char minute,
    unsigned char second
  );
  int addMsg(MDF_UINT32 id, KVMDF_TIMESTAMP ts, MDF_UINT8 *data);
  int write_data(FILE *mdfFile);

  int write(FILE *mdfFile);
  unsigned long size();
};

class idNode : public NodeBase {
public:
  IDBLOCK id;
  idNode(int version);
  virtual ~idNode();

  int write(FILE *mdfFile);
  unsigned long size();

  void setEndianess(MDF_UINT16 endianess) { id.DefaultByteOrder = endianess; }
};

}; // end-of namespace
#endif
