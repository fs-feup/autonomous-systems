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

#ifndef _MDF_H_
#define _MDF_H_

#include <stdlib.h>
//#include <inttypes.h>
#include <stdio.h>
#include <time.h>
#include "common_defs.h"
/*

The log file format used by the CANalyzer seems to be a bit convoluted.
There's a file header (mdf_header_t) followed by a sequence of blocks.
The blocks are started with a signature - two letters - and a two-byte
length that includes the signature.

The first block is always a HD block containing file creation date etc.
The HD block contains three "pointers" or file offsets (counted from the
start of the file) and the first one points at a DG block near the end of the file,
the second one points at a TX block at the end of the file, and the third one
seems unused.

The DG block points to a CG block earlier in the file that points to
an earlier CN block that points to a CN block that ... that points
to a CC block that looks rather empty.

The MDF format is likely a general format and just a subset is
actually being used for the CANalyzer.

*/

#include <pshpack1.h>

/*
** File Header.
** Most fields are zero.
*/
typedef struct {
    char            signature[8];      // "MDF       "
    unsigned char   _fill[8];          // null, could be file version info?
    char            creator[12];        // "CANwin I"
    uint32_t        magic;             // ??? 0xC8, 200 dec. magic number?
    unsigned char   _fill3[32];        // null
} mdf_header_t;

#define MDF_HEADER      "MDF     "
#define MDF_MAGIC       0xC8          // Version 2.00 (200 decimal)
#define MDF_NEWMAGIC    0x12C         // Version 3.00 (300 decimal)


/*
** Another file header, however this time it's structured like all other blocks
** in the file.
*/

typedef struct {
    WORD        magic;              // 'H' 'D'
    WORD        length;             // Length of block including magic.

    DWORD       dgPos;              // Points at a DG block.
    DWORD       txPos;              // Points at the last TX block.
    DWORD       offset_3;           // Unknown, always zero
    WORD        nnn;                // Unknown, set to one?

    char        date[10];           // DD:MM:YYYY with *real* colons!
    char        time[8];            // HH:mm:ss
    char        comment[128];       // Fill with null - CYN says 'MDF-Library'
} mdf_hd_block_t;
#define MDF_HD_BLOCK_MAGIC  ('H' + 256*'D') // 0x4448


/*
** General block.
*/
typedef struct {
  WORD        magic;
  WORD        length;             // Length of block including magic.
  union {
    struct {
      DWORD next0;
      DWORD next1;
    };
    char        data[1];            // Data (variable length)
  };
} mdf_block;


/*
** Block for text. Used as last block by CYN.
*/
typedef struct {
    WORD        magic;
    WORD        length;             // Length of block including magic.
    char        text[1];            // Data....
} mdf_tx_block;
#define MDF_TX_BLOCK_MAGIC  ('T' + 256*'X')  // 0x5854

/*
** A DG block.
*/
typedef struct {
  WORD        magic;
  WORD        length;             // Length of block including magic.
  DWORD       z1;
  DWORD       next;
  DWORD       z2;
  DWORD       z3;
  WORD        z4, z5;
} mdf_dg_block;
#define MDF_DG_BLOCK_MAGIC  ('D' + 256*'G')  // 0x4744

/*
** A CG block.
*/
typedef struct {
  WORD        magic;
  WORD        length;             // Length of block including magic.
  DWORD       z1;
  DWORD       next;
  DWORD       z2;
  WORD        z3,z4,z5;
  DWORD       recCount;
} mdf_cg_block;
#define MDF_CG_BLOCK_MAGIC  ('C' + 256*'G')  // 0x4743


/*
** A CN block (column).
*/
typedef struct {
  WORD        magic;
  WORD        length;       // Length of block including magic.
  DWORD       next0;        // Used for all records but the last one
  DWORD       next1;        // Used in the last record (first in file)
  char        z1[12];       // {0, ...}
  WORD        flag1;
  char        colName[160];
  WORD        pos;
  WORD        len;
  WORD        flag2;
  char        z5[26];
} mdf_cn_block;
#define MDF_CN_BLOCK_MAGIC  ('C' + 256*'N')  // 0x4e43

/*
** A CC block
*/
typedef struct {
    WORD        magic;
    WORD        length;            // Length of block including magic.
    DWORD       z1;
    DWORD       next;
    char        z[42];
    double      dt;                // Unsure; always 0.00010000000
} mdf_cc_block;
#define MDF_CC_BLOCK_MAGIC  ('C' + 256*'C')  // 0x4343


/* Event records, length 23 bytes */
typedef struct {
  BYTE z1;             // Always 1
  BYTE type;
  char z[17];
  DWORD time;          // Time in 0.00001 s
} mdf_event_block;

typedef struct {
  BYTE z1;
  BYTE type;
  WORD channel;
  DWORD canId;
  BYTE dir;
  BYTE rtr;
  BYTE dlc;
  BYTE data[8];
  DWORD time;
} mdf_evtMsg_block;

typedef struct {
  BYTE z1;
  BYTE type;
  WORD channel;
  DWORD canId;         // 0x0000ffff
  BYTE dir;            // 00=Rx 01=Tx 02=TxRq
  BYTE rtr;            // always 0
  BYTE dlc;            // always 8
  BYTE data[8];        // don't care
  DWORD time;
} mdf_evtErrorFrame_block;

typedef struct {
  BYTE z1;
  BYTE type;
  WORD channel;
  BYTE b[15];
  DWORD time;
} mdf_evtBusstat_block;


// Values for type
#define MDF_EVTTYPE_MESSAGE 0x00
#define MDF_EVTTYPE_BUSSTAT 0x08
#define MDF_EVTTYPE_ERRORFRAME 0x09

// Values for dir
#define MDF_EVTDIR_RX   0x00
#define MDF_EVTDIR_TX   0x01
#define MDF_EVTDIR_TXRQ 0x02

#define MDF_ID_EXT 0x80000000 // Marks than a CAN id is extended
#include <poppack.h>


class MDFfile {
private:
  FILE *f;
  bool writing;
  char *tmpStr;
  mdf_block *tmpBlock;
  mdf_event_block evtBlock;
  bool corrupt;
  size_t mBytesWritten;

public:
  mdf_header_t header;
  mdf_hd_block_t hdBlock;
  mdf_dg_block *dgBlock;
  mdf_cg_block *cgBlock;
  mdf_cc_block *ccBlock;
  mdf_cn_block *cnBlocks[16];
  unsigned int cnBlockCount;

  MDFfile(void);
  ~MDFfile();
  bool isOpen(void) { return f != NULL; }
  bool isCorrupt(void) { return corrupt; }
  int open(const char *fileName);
  int handle(FILE *f);
  int create(const char *fileName, time_t wallclock = 0);
  void sync(void);
  int close(void);
  char *mkTmpStr(const char *s, int len);
  mdf_block *readBlock(uint32_t pos, bool alloc = false);
  uint64_t eventCount(void) { return (uint64_t)cgBlock->recCount; }
  mdf_event_block *readEvent(uint64_t i);
  int writeEvent(mdf_event_block *evtBlock);

  char *getDate(void) { return mkTmpStr(hdBlock.date, sizeof(hdBlock.date)); }
  char *getTime(void) { return mkTmpStr(hdBlock.time, sizeof(hdBlock.time)); }
  char *getCreator(void) { return mkTmpStr(header.creator, sizeof(header.creator)); }
  char *getComment(void) { return mkTmpStr(hdBlock.comment, sizeof(hdBlock.comment)); }
  char *getText(void);
  size_t getBytesWritten() {return mBytesWritten;}
};

void print_mdf(MDFfile *mdf);
void copy_mdf(MDFfile *mdfR, char *outFile);


#endif // _MDF_H_
