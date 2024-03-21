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
**  Classes for writing Bosch MDF (Vector), supports version 4.00
** -----------------------------------------------------------------------------
*/

#ifndef MDF4_H
#define MDF4_H

#include <stdlib.h>
#include <stdio.h>
#include <string>
#include <map>
#include <vector>
#include "common_defs.h"
#include "BufQueue.h"
#include "mdf4base.h"


#include <pshpack1.h>

// -----------------------------------------------------------------------------
// MDF v4.1
// -----------------------------------------------------------------------------
/*
  Use MdfCanFrameType to send CAN and CAN FD frames to converter. The same
  structure is used to store data in MDF 4.1, but uses only 8 bytes of the
  data field:

  - Maximum length records (MLSD) can only store 8 bytes data and should only
    be used for classic CAN signals.

  - Variable length records (VLSD) replaces the 8 data bytes with an offset
    into a signal data (SD) block. Each SD block entry has 4 bytes for length
    of data and up to 64 bytes data.
*/

static const MDF_REAL TIMESTAMP_TO_SECONDS = 1.0E-09;

typedef struct {
  uint32_t canId;
  uint32_t flags;
  uint8_t  dlc;
  uint8_t  datalength;
  uint8_t  data[64];
} MdfCanFrameType;

// Byte offset into MdfCanFrameType
static const size_t FIRST_DATA_BYTE = 10;
static const size_t DATA_LENGTH_BYTE = 9;

// Byte offset into MDF record: KVMDF_TIMESTAMP (8 bytes) + MdfCanFrameType
static const size_t MDF_FIRST_DATA_BYTE = FIRST_DATA_BYTE + sizeof(KVMDF_TIMESTAMP);


static const size_t MDF_MLSD_RECORD_SIZE = sizeof(KVMDF_TIMESTAMP) +
                                           sizeof(MdfCanFrameType) -
                                           56; // Only 8 bytes of 64 used in data.

static const size_t MDF_MLSD_RECORD_SIZE_FD = sizeof(KVMDF_TIMESTAMP) +
                                              sizeof(MdfCanFrameType);

static const size_t MDF_VLSD_RECORD_SIZE = sizeof(KVMDF_TIMESTAMP) +
                                           sizeof(MdfCanFrameType) -
                                           56; // Only 8 bytes of 64 used in data.


static const size_t MDF_MAX_SD_BLOCK_SIZE = 4 + 64; // 4 bytes data length + 64 bytes data

static const size_t MDF_ERROR_FRAME_SIZE  = 9;
static const size_t MDF_REMOTE_FRAME_SIZE = 9;
static const size_t MDF_DATA_FRAME_SIZE   = MDF_MLSD_RECORD_SIZE - sizeof(KVMDF_TIMESTAMP);
static const size_t MDF_DATA_FRAME_SIZE_FD = MDF_MLSD_RECORD_SIZE_FD - sizeof(KVMDF_TIMESTAMP);


namespace Mdf4Nodes {

  // ---------------------------------------------------------------------------
  // Struct definitions used by Node classes
  // ---------------------------------------------------------------------------

  static const MDF_UINT32 CAN_ID_NOT_USED = 0xFFFFFFFF;

  // 4.5 The File Identification Block IDBLOCK
  typedef struct {
    MDF_CHAR    id_file[8];         // "MDF     "
    MDF_CHAR    id_vers[8];         // "4.00    "
    MDF_CHAR    id_prog[8];         // Kvaser + Major version + Minor version, e.g. "Kvaser10"
    MDF_BYTE   id_reserved1[4];     // 0
    MDF_UINT16 id_ver; // Version number of the MDF format, i.e. 400 for this version
    MDF_BYTE   id_reserved2[34];    // Reserved
  } IDBLOCK;

  // Common header for all other blocks
  typedef struct {
    MDF_CHAR    id[4];
    MDF_BYTE    reserved[4];
    MDF_UINT64  length;
    MDF_UINT64  link_count;
  } BL_HEAD;

  //4.6  The Header Block HDBLOCK
  #define MDF_ID_HDBLOCK "##HD"
  typedef struct {
    BL_HEAD     head;                         // "HD"
                                              // sizeof(HDBLOCK)
    // Link section
    MDF_LINK64    hd_dg_first;          // Pointer to the first data group block (DGBLOCK) (NIL allowed)
    MDF_LINK64    hd_fh_first;          // Pointer to first file history block (FHBLOCK) (at least one)
    MDF_LINK64    hd_ch_first;          // Pointer to first channel hierarchy block (CHBLOCK) (NIL allowed)
    MDF_LINK64    hd_at_first;          // Pointer to first attachment block (ATBLOCK) (NIL allowed)
    MDF_LINK64    hd_ev_first ;         // Pointer to first event block (EVBLOCK) (NIL allowed)
    MDF_LINK64    hd_md_comment;        // Pointer to the measurement file comment(TXBLOCK or MDBLOCK) (can be NIL)

    // Data section
    MDF_UINT64  hd_start_time_ns;  // Time stamp at start of measurement in nanoseconds elapsed since 00:00:00 01.01.1970 (UTC time or local time, depending on "local time" flag, see [UTC])
    MDF_INT16   hd_tz_offset_min;  // Time zone offset in minutes.
    MDF_INT16   hd_dst_offset_min; // Daylight saving time (DST) offset in minutes
    MDF_UINT8   hd_time_flags;             // Time flags
    MDF_UINT8   hd_time_class;     // Time quality class
    MDF_UINT8   hd_flags;          // Flags
    MDF_UINT8   hd_reserved;       // Reserved
    MDF_REAL    hd_start_angle_rad; // Start angle in radians at start of measurement
    MDF_REAL    hd_start_distance_m; //Start distance in meters at start of measurement
  } HDBLOCK;


  // 4.7 The Meta Data Block MDBLOCK
  #define MDF_ID_MDBLOCK "##MD"
  typedef struct {
    BL_HEAD     Head;                    // "MD"
                                         // sizeof MDBLOCK, may vary
    MDF_BYTE    *md_data;                // XML string UTF-8 encoded
  } MDBLOCK;


  // 4.8 The Text Block TXBLOCK
  #define MDF_ID_TXBLOCK "##TX"
  typedef struct {
    BL_HEAD     Head;                         // "TX"
                                              // sizeof TXBLOCK, may vary
    MDF_BYTE    *tx_data;                     // Text in utf-8, new line: CR+LF, end of text: \0
  } TXBLOCK;

  // 4.9 The File History Block FHBLOCK
  #define MDF_ID_FHBLOCK "##FH"
  typedef struct {
    BL_HEAD     Head;            // "FB"
    MDF_LINK64    fh_fh_next ;     // Link to next FHBLOCK (can be NIL if list finished)
    MDF_LINK64    fh_md_comment;   // Link to MDBLOCK containing comment about the creation or modification of the MDF file.
    MDF_UINT64  fh_time_ns;      // Time stamp at which the file has been changed/created (first entry) in nanoseconds
    MDF_INT16 fh_tz_offset_min;  // Time zone offset in minutes.
    MDF_INT16 fh_dst_offset_min; // Daylight saving time (DST) offset in minutes
    MDF_UINT8 fh_time_flags;     // Time Flags
    MDF_BYTE fh_reserved[3];     // Reserved
  } FHBLOCK;

  // 4.10 The Channel Hierarchy Block  CHBLOCK
  #define MDF_ID_CHBLOCK "##CH"
  typedef struct {
    BL_HEAD     Head;              // "CH"
                                   // sizeof CHBLOCK, may vary
    MDF_LINK64    ch_ch_next;        // Link to next sibling CHBLOCK (can be NIL)
    MDF_LINK64    ch_ch_first;       // Link to first child CHBLOCK (can be nil)
    MDF_LINK64    ch_tx_name;        // Link to TXBLOCK with the name of the hierarchy level.
    MDF_LINK64    ch_md_comment;     // Link to TXBLOCK or MDBLOCK (can be NIL)
    MDF_LINK64   *ch_element;        // Link triple (N*3) with references to the channels for this hierarchy level.
    MDF_UINT32  ch_element_count;  // Number of channels N referenced by this CHBLOCK.
    MDF_UINT8   ch_type;           // Type of hierarchy level
    MDF_BYTE    ch_reserved[3];    // Reserved
  } CHBLOCK;

  // 4.11 The Attachment Block ATBLOCK
  #define MDF_ID_ATBLOCK "##AT"
  typedef struct {
    BL_HEAD     Head;                // "AT"
                                     // sizeof ATBLOCK, may vary
    MDF_LINK64  at_at_next;          // Link to next ATBLOCK (linked list) (can be NIL)
    MDF_LINK64  at_tx_filename;      // Link to TXBLOCK with the path and file name of the embedded or referenced file (can only be NIL if data is embedded).
    MDF_LINK64  at_tx_mimetype;      // LINK to TXBLOCK with MIME content-type text that gives information about the attached data. Can be NIL
    MDF_LINK64  at_md_comment ;      // Link to MDBLOCK (can be NIL)
    MDF_UINT16  at_flags;            // Flags
    MDF_UINT16  at_creator_index;    // Flags
    MDF_BYTE    at_reserved[4];      // Reserved
    MDF_BYTE    at_md5_checksum[16];  // 128-bit value for MD5 check sum
    MDF_UINT64  at_original_size;    // Original data size in Bytes
    MDF_UINT64  at_embedded_size;    // Embedded data size N, i.e. number of Bytes
    MDF_BYTE    *at_embedded_data;   // Contains binary embedded data (possibly compressed).
  } ATBLOCK;

  #define AT_BLOCK_BUFFER_SIZE (1024*1024*4)
  #define MDF_AT_FLAG_EMBEDDED   0x0001
  #define MDF_AT_FLAG_COMPRESSED 0x0002
  #define MDF_AT_FLAG_MD5_VALID  0x0004

  // 4.12 The Event Block EVBLOCK
  #define MDF_ID_EVBLOCK "##EV"
  typedef struct {
    BL_HEAD     Head;                // "EV"
                                     // sizeof ATBLOCK, may vary
    MDF_LINK64    ev_ev_next;          // Link to next EVBLOCK (linked list) (can be NIL)
    MDF_LINK64    ev_ev_parent;        // Referencing link to EVBLOCK with parent event (can be NIL).
    MDF_LINK64    ev_ev_range;         // Referencing link to EVBLOCK with event that defines the beginning of a range
    MDF_LINK64    ev_tx_name;          // Pointer to TXBLOCK with event name (can be NIL)
    MDF_LINK64    ev_md_comment;       // Pointer to TX/MDBLOCK (can be NIL)
    MDF_LINK64    *ev_scope;           // List of links to channels and channel groups to which the event applies
    MDF_LINK64    *ev_at_reference;    // List of attachments for this event
    MDF_UINT8   ev_type;             // Event type
    MDF_UINT8   ev_sync_type;        // Sync type
    MDF_UINT8   ev_range_type;       // Range type
    MDF_UINT8   ev_cause;            // Cause of event
    MDF_UINT8   ev_flags;            // Flags
    MDF_BYTE    ev_reserved[3];      // Reserved
    MDF_UINT32  ev_scope_count;      // Length M of ev_scope list. Can be zero.
    MDF_UINT16  ev_attachment_count; // Length N of ev_at_reference list
    MDF_UINT16  ev_creator_index;    // Creator index, i.e. zero-based index of FHBLOCK in global list
    MDF_INT64   ev_sync_base_value;  // Base value for synchronization value.
    MDF_REAL    ev_sync_factor;      // Factor for event synchronization value.
  } EVBLOCK;


  // 4.13 The Data Group Block DGBLOCK
  #define MDF_ID_DGBLOCK "##DG"
  typedef struct {
    BL_HEAD Head;                             // "DG"
                                              // sizeof(DGBLOCK)
    MDF_LINK64    dg_dg_next;                  // Pointer to next data group block (DGBLOCK) (can be NIL)
    MDF_LINK64    dg_cg_first;                 // Pointer to first channel group block (CGBLOCK) (can be NIL)
    MDF_LINK64    dg_data;                     // Pointer to data block (DTBLOCK) or data list block (DLBLOCK) (can be NIL)
    MDF_LINK64    dg_md_comment ;              // Pointer to comment and additional information (TXBLOCK or MDBLOCK) (can be NIL)
    MDF_UINT8   dg_rec_id_size;              // Number of Bytes used for record IDs in the data block.
    #define MDF_ID_DATA_RECORDS_WITHOUT_ID        0
    #define MDF_ID_UINT8_BEFORE_DATA_RECORD       1
    #define MDF_ID_UINT16_BEFORE_DATA_RECORD      2
    #define MDF_ID_UINT32_BEFORE_DATA_RECORD      4
    #define MDF_ID_UINT64_BEFORE_DATA_RECORD      8
    MDF_BYTE  dg_reserved[7];
  } DGBLOCK;


  // 4.14 The Channel Group Block CGBLOCK
  #define MDF_ID_CGBLOCK "##CG"
  typedef struct {
    BL_HEAD Head;                   // "CG"
                                    // sizeof(CGBLOCK)
    MDF_LINK64  cg_cg_next;         // Pointer to the next channel group block (CGBLOCK) (NIL allowed)
    MDF_LINK64  cg_cn_first;        // Pointer to the first channel block (CNBLOCK) (NIL allowed)
    MDF_LINK64  cg_tx_acq_name;     // Pointer to acquisition name (TXBLOCK) (can be NIL)
    MDF_LINK64  cg_si_acq_source;   // Pointer to acquisition source (SIBLOCK) (can be NIL)
    MDF_LINK64  cg_sr_first;        // Pointer to first sample reduction block (SRBLOCK) (can be NIL)
    MDF_LINK64  cg_md_comment;      // Pointer to comment and additional information (TXBLOCK or MDBLOCK) (can be NIL)
    MDF_UINT64  cg_record_id;       // Record ID
    MDF_UINT64  cg_cycle_count;     // Number of cycles, i.e. number of samples for this channel group.
    MDF_UINT16  cg_flags;           // Flags
    MDF_UINT16  cg_path_separator;  // Path separator value since 4.1
    MDF_BYTE    cg_reserved[4];     // Reserved
    //MDF_BYTE    cg_reserved[6];     // Reserved 4.0
    MDF_UINT32  cg_data_bytes;      // Number of data Bytes (after record ID)
    MDF_UINT32  cg_inval_bytes;     // Number of additional Bytes for record used for invalidation bits. Can be zero.
  } CGBLOCK;

  const MDF_UINT32 MDF_CG_FLAGS_BUS_EVENT = 0x2;
  const MDF_UINT32 MDF_CG_FLAGS_PLAIN_BUS_EVENT = 0x4;
  const MDF_UINT32 MDF_CG_PATH_SEPARATOR_DOT = 46; // ASCII for '.'


  // 4.15 The Source Information Block SIBLOCK
  #define MDF_ID_SIBLOCK "##SI"
  typedef struct {
    BL_HEAD Head;                   // "SI"
                                    // sizeof(CGBLOCK)
    MDF_LINK64  si_tx_name;        // Pointer to TXBLOCK with name (identification) of source (must not be NIL).
    MDF_LINK64  si_tx_path;        // Pointer to TXBLOCK with (tool-specific) path of source (can be NIL).
    MDF_LINK64  si_md_comment;     // Pointer to source comment and additional information (TXBLOCK or MDBLOCK) (can be NIL)
    MDF_UINT8   si_type;           // Source type (2 = BUS  source is a bus)
    MDF_UINT8   si_bus_type;       // Bus type (2 = CAN)
    MDF_UINT8   si_flags;          // Flags
    MDF_BYTE    si_reserved[5];    // Reserved
  } SIBLOCK;

  const MDF_UINT8 SI_TYPE_BUS     = 2;
  const MDF_UINT8 SI_BUS_TYPE_CAN = 2;

  // 4.16 The Channel Block CNBLOCK
  #define MDF_ID_CNBLOCK "##CN"
  typedef struct {
    BL_HEAD Head;                             // "CN"
                                              // sizeof(CNBLOCK)
    MDF_LINK64  cn_cn_next;        // Pointer to next channel block (CNBLOCK) (can be NIL)
    MDF_LINK64  cn_composition;    // Composition of channels: Pointer to channel array block (CABLOCK) or channel block (CNBLOCK) (can be NIL).
    MDF_LINK64  cn_tx_name;        // Pointer to TXBLOCK with name (identification) of channel.
    MDF_LINK64  cn_si_source;      // Pointer to channel source (SIBLOCK) (can be NIL)
    MDF_LINK64  cn_cc_conversion;  // Pointer to the conversion formula (CCBLOCK) (can be NIL)
    MDF_LINK64  cn_data;           // Pointer to channel type specific signal data
    MDF_LINK64  cn_md_unit;        // Pointer to TXBLOCK/MDBLOCK with designation for physical unit
    MDF_LINK64  cn_md_comment;     // Pointer to TXBLOCK/MDBLOCK with comment and additional information
    MDF_UINT8   cn_type;           // Channel type
    MDF_UINT8   cn_sync_type;      // Sync type
    MDF_UINT8   cn_data_type;      // Channel data type of raw signal value
    MDF_UINT8   cn_bit_offset;     // Bit offset (0-7): first bit (=LSB) of signal value after Byte offset has been applied
    MDF_UINT32  cn_byte_offset;    // Offset to first Byte in the data record that contains bits of the signal value.
    MDF_UINT32  cn_bit_count;      // Number of bits for signal value in record.
    MDF_UINT32  cn_flags;          // Flags
    MDF_UINT32  cn_inval_bit_pos;  // Position of invalidation bit
    MDF_UINT8   cn_precision;      // Precision for display of floating point values. 0xFF means unrestricted precision (infinite).
    MDF_BYTE    cn_reserved[3];    // Reserved
    MDF_REAL    cn_val_range_min;  // Minimum signal value that occurred for this signal (Only valid if "value range valid" flag (bit 3) is set.)
    MDF_REAL    cn_val_range_max;  // Maximum signal value that occurred for this signal
    MDF_REAL    cn_limit_min;      // Lower limit for this signal
    MDF_REAL    cn_limit_max;      // Upper limit for this signal
    MDF_REAL    cn_limit_ext_min;  // Lower extended limit for this signal
    MDF_REAL    cn_limit_ext_max;  // Upper extended limit for this signal
  } CNBLOCK;

  const MDF_UINT8 MDF_CN_TYPE_FIXED_LENGTH            = 0;
  const MDF_UINT8 MDF_CN_TYPE_VARIABLE_LENGTH         = 1;
  const MDF_UINT8 MDF_CN_TYPE_MASTER_CHANNEL          = 2;
  const MDF_UINT8 MDF_CN_TYPE_VIRTUAL_MASTER_CHANNEL  = 3;
  const MDF_UINT8 MDF_CN_TYPE_SYNCHRONIZATION_CHANNEL = 4;
  const MDF_UINT8 MDF_CN_TYPE_MAXIMUM_LENGTH          = 5;
  const MDF_UINT8 MDF_CN_TYPE_VIRTUAL                 = 6;

  const MDF_UINT8 MDF_CN_SYNC_TYPE_NONE     = 0;
  const MDF_UINT8 MDF_CN_SYNC_TYPE_TIME     = 1;
  const MDF_UINT8 MDF_CN_SYNC_TYPE_ANGLE    = 2;
  const MDF_UINT8 MDF_CN_SYNC_TYPE_DISTANCE = 3;
  const MDF_UINT8 MDF_CN_SYNC_TYPE_INDEX    = 4;

  const MDF_UINT32 MDF_CN_FLAGS_VALUE_RANGE_OK = 8;
  const MDF_UINT32 MDF_CN_FLAGS_DVAL = 64;
  const MDF_UINT32 MDF_CN_FLAGS_BUS_EVENT = 1024;


  // 3.11 The Channel Conversion Block CCBLOCK
  #define MDF_ID_CCBLOCK "##CC"

  typedef struct {
    BL_HEAD     Head;              // "CC" & BlockSize (may vary)
    MDF_LINK64    cc_tx_name;        // Link to TXBLOCK with name (identifier) of conversion (can be NIL).
    MDF_LINK64    cc_md_unit;        // Link to TXBLOCK/MDBLOCK with physical unit of signal data (can be NIL)
    MDF_LINK64    cc_md_comment;     // Link to TXBLOCK/MDBLOCK with comment of conversion (can be NIL)
    MDF_LINK64    cc_cc_inverse;     // Link to CCBLOCK for inverse formula (can be NIL)
    // Note: cc_ref is not used with current conversion types
    // MDF_LINK64    cc_ref;            // List of additional links to TXBLOCKs with strings or to CCBLOCKS  with partial conversion rules.
    MDF_UINT8   cc_type;           // Conversion type (formula identifier)
    MDF_UINT8   cc_precision;      // Precision for display of floating point values
    MDF_UINT16  cc_flags;          // Flags
    MDF_UINT16  cc_ref_count;      // Length M of cc_ref list with additional links.
    MDF_UINT16  cc_val_count;      // Length N of cc_val list with additional parameters.
    MDF_REAL    cc_phy_range_min;  // Minimum physical signal value that occurred for this signal.
    MDF_REAL    cc_phy_range_max;  // Maximum physical signal value that occurred for this signal.
    MDF_REAL    *cc_val ;          // List of additional conversion parameters. Length of list is given by cc_val_count. The list can be empty.
  } CCBLOCK;

  const MDF_UINT32 MDF_CC_FLAGS_PHYSICAL_RANGE_OK = 2;


  // 4.21  The Data Block (DTBLOCK)
  #define MDF_ID_DTBLOCK "##DT"
  typedef struct {
    BL_HEAD     Head;              // "DT" & BlockSize (may vary)
  } DTBLOCK;

  // Vector CANFD frame is 4 byte length and up to 64 byte data
  typedef struct {
    uint32_t length;
    uint8_t data[64];
  } CanFd_Type;

  // 4.21  The Variable Length Signal Data Block (SD)
  #define MDF_ID_SDBLOCK "##SD"
  typedef struct {
    BL_HEAD     Head;              // "SD" & BlockSize (may vary)
  } SDBLOCK;

 // 5.26 The Data Zipped Block DZBLOCK
  #define MDF_ID_DZBLOCK "##DZ"

  typedef struct {
    BL_HEAD     Head;                 // "DZ" & BlockSize (may vary)
    MDF_BYTE    dz_org_block_type[2]; // Block type identifier of the original (replaced)
                                      // data block without the "##" prefix, i.e.
                                      // either "DT", "SD" or "RD".
    MDF_UINT8   dz_zip_type;          // Zip algorithm used to compress the data
    MDF_BYTE    dz_reserved;          // Reserved
    MDF_UINT32  dz_zip_parameter;     // Only for MDF_DZ_ZIP_TYPE_TRANSPOSE_DEFLATE. Number of Bytes used as columns.
    MDF_UINT64  dz_org_data_length;   // Length of uncompressed data in Bytes; max 2^22 (4MByte).
    MDF_UINT64  dz_data_length;       // Length of compressed data in Bytes.
    // MDF_BYTE    dz_data            // Zipped data
  } DZBLOCK;

  const MDF_UINT8 MDF_DZ_ZIP_TYPE_DEFLATE = 0;
  const MDF_UINT8 MDF_DZ_ZIP_TYPE_TRANSPOSE_DEFLATE = 1;

  // 5.26 The Data List Block DLBLOCK (OFFSET version)
  #define MDF_ID_DLBLOCK "##DL"

  typedef struct {
    BL_HEAD     Head;                 // "DL" & BlockSize
    MDF_LINK64  dl_dl_next;           // Link to next data list block (DLBLOCK) (can be NILL)
    //MDF_LINK64  *dl_data            // Links (N) to data blocks; one of (DT, SD, RD, DZ)
    MDF_UINT8   dl_flags;             // Flags
    MDF_BYTE    dl_reserved[3];       // Reserved
    MDF_UINT32  dl_count;             // Number of referenced blocks;
    // MDF_UINT64  dl_equal_length;   // Only present if "equal length" flag (bit 0 in dl_flags) is set
    // MDF_UINT64 dl_offset           // (N) Only present if "equal length" flag (bit 0 in dl_flags) is not set.
  } DLBLOCK;


  // 5.26 The Header List Block HLBLOCK
  #define MDF_ID_HLBLOCK "##HL"

  typedef struct {
    BL_HEAD     Head;                 // "DL" & BlockSize
    MDF_LINK64  hl_dl_first;          // Link to first data list block (DLBLOCK)
    MDF_UINT16  hl_flags;             // Flags
    MDF_UINT8   hl_zip_type;          // Zip algorithm, see dz_zip_type in DZBLOCK
    MDF_BYTE    dl_reserved[5];       // Reserved
  } HLBLOCK;

  static const MDF_UINT16 MDF_HL_FLAGS_EQ_LENGTH = 1;

  #include <poppack.h>

  // ---------------------------------------------------------------------------
  // Node classes
  // ---------------------------------------------------------------------------

  class mdNode : public NodeBase {
  public:
    MDBLOCK   md;
    mdNode(int version, int type, const char *str, const char *display = NULL);
    static const int FH_MD_COMMENT = 0;
    static const int CC_MD_UNIT    = 1;
    static const int CN_MD_COMMENT = 2;
    static const int SI_MD_COMMENT = 3;
    ~mdNode();

    int write(FILE *mdfFile);
    uint64_t size();
  };

  class txNode : public NodeBase {
  public:
    TXBLOCK   tx;
    txNode(int version, const char *str);
    ~txNode();

    int write(FILE *mdfFile);
    uint64_t size();
  };

  class ccNode : public NodeBase {
  public:
    CCBLOCK     cc;
    static const MDF_UINT8 CC_TYPE_CONVERSION_TYPE_ONE_TO_ONE = 0;
    static const MDF_UINT8 CC_TYPE_CONVERSION_TYPE_PARAMETRIC_LINEAR = 1;
    txNode    *md_unit;

    ccNode(int version, const char *unit, MDF_UINT16 type, MDF_REAL p1=0.0, MDF_REAL p2=1.0);
    ~ccNode();

    int write(FILE *mdfFile);
    uint64_t size();
  };

  class dlNode;
  class dtNode : public NodeBase {
  public:
    DTBLOCK   dt;
    DZBLOCK   dz;
    dtNode(int version);
    ~dtNode();

    int write(FILE *mdfFile);
    uint64_t size();

    BQueue     *bq;
    MDF_UINT8  RecordSize;
    char       *record;
    unsigned long dataRecords;
    dlNode *dl;
  };

  class sdNode : public NodeBase {
  public:
    SDBLOCK   sd;
    DZBLOCK   dz;
    sdNode(int version);
    ~sdNode();

    int write(FILE *mdfFile);
    uint64_t size();

    BQueue     *bqv;
    MDF_UINT8  RecordSize;
    char       *record;
    unsigned long dataRecords;
    MDF_UINT64 position;
    dlNode *dl;
  };

  class dzNode : public NodeBase {
  public:
    DZBLOCK   dz;
    dzNode(int version);
    ~dzNode();

    int write(FILE *mdfFile);
    uint64_t size();
  };

  class siNode : public NodeBase {
  public:
    SIBLOCK   si;
    txNode *tx_name;
    txNode *tx_path;
    mdNode *md_comment;
    siNode(int version, int channel);
    ~siNode();

    int write(FILE *mdfFile);
    uint64_t size();
  };

  class cnNode : public NodeBase {
  public:
    CNBLOCK   cn;
    mdNode    *md_comment;
    txNode    *tx_long_signal_name;
    ccNode    *cc; // channel conversion
    cnNode    *next;
    cnNode    *cn_composition;
    cnNode    *cn_data;
    NodeBase  *cn_data_sd;
    siNode    *cn_si_source;


    cnNode(int version,
           char *long_name,
           char *name,
           char *unit,
           MDF_UINT16 StartOffset,
           MDF_UINT16 NumBits,
           MDF_UINT16 SigDataType,
           MDF_UINT16 ChannelConversionType = MDF_CONVERSION_TYPE_ONE_TO_ONE,
           MDF_REAL Factor = 1.0,
           MDF_REAL Offset = 0.0);
    cnNode(int version);
    ~cnNode();


    int write(FILE *mdfFile);
    uint64_t size();

    // Some of the available data types in MDF4
    static const MDF_UINT8 CN_DATA_TYPE_UNSIGNED_INT_LE  = 0;
    static const MDF_UINT8 CN_DATA_TYPE_UNSIGNED_INT_BE  = 1;
    static const MDF_UINT8 CN_DATA_TYPE_SIGNED_INT_LE    = 2;
    static const MDF_UINT8 CN_DATA_TYPE_SIGNED_INT_BE    = 3;
    static const MDF_UINT8 CN_DATA_TYPE_IEEE_FLOAT_LE    = 4;
    static const MDF_UINT8 CN_DATA_TYPE_IEEE_FLOAT_BE    = 5;
    static const MDF_UINT8 CN_DATA_TYPE_STRING_ASCII     = 6; // (ISO-8859-1 Latin, NULL terminated)
    static const MDF_UINT8 CN_DATA_TYPE_STRING_UTF8      = 7; // (NULL terminated)
    static const MDF_UINT8 CN_DATA_TYPE_STRING_UTF16_LE  = 8;  // (NULL terminated)
    static const MDF_UINT8 CN_DATA_TYPE_STRING_UTF16_BE  = 9;  // (NULL terminated)
    static const MDF_UINT8 CN_DATA_TYPE_BYTE_ARRAY       = 10;  // (NULL terminated)

    MDF_UINT8 toChannelDataType(MDF_UINT16 SigDataType);
  };

  class cgNode : public NodeBase {
  public:
    CGBLOCK   cg;
    cnNode    *cn;
    cgNode    *next;
    cgNode(int version, char *cg_comment = NULL);
    cgNode(int version, int channel);
    ~cgNode();

    txNode    *tx_acq_name;
    siNode    *si_acq_source;

    MdfStatus new_cn(char *long_name,
                      char *name,
                      char *unit,
                      MDF_UINT16 StartOffset,
                      MDF_UINT16 NumBits,
                      MDF_UINT16 SigDataType,
                      MDF_UINT16 ChannelConversionType,// = MDF_CONVERSION_TYPE_ONE_TO_ONE,
                      MDF_REAL Factor = 1.0,
                      MDF_REAL Offset = 0.0);
    void      increaseNumberOfMessages(int num = 1);
    MdfStatus new_vector_cn(void);
    int write(FILE *mdfFile);
    uint64_t size();
  };

  class hlNode : public NodeBase{
  public:
    HLBLOCK   hl;
    dlNode *dl;
    hlNode(int version);
    ~hlNode();

    int write(FILE * /* mdfFile */) {return 0;};
    int write_data(FILE *mdfFile);
    uint64_t size();
    void add_data_node(NodeBase *node);
  };

  class dlNode : public NodeBase{
  private:
    char *input;
    char *output;
    size_t input_len;

  public:
    DLBLOCK   dl;
    std::vector<NodeBase*> nodeList;
    dlNode* next;
    dlNode(int version);
    ~dlNode();

    int write(FILE *mdfFile);
    int write_data(FILE *mdfFile);
    uint64_t size();
    void add_data_node(NodeBase *node);
    void add_data_bytes(char *data, size_t len);
    void get_zipped_data(char **data, size_t *len);
  };

  class dgNode : public NodeBase{

  public:
    DGBLOCK   dg;
    cgNode    *cg;
    dtNode    *dt;
    dlNode    *dl;
    hlNode    *hl;
    sdNode    *sd;
    dlNode    *dl_sd;
    hlNode    *hl_sd;
    dgNode    *next;
    cnNode    *cn_time;
    dgNode(int version);
    ~dgNode();

    MdfStatus new_cg(MDF_UINT8 dlc);
    MdfStatus new_mdf4_cg(int channel, int type);

    int write(FILE *mdfFile);
    uint64_t size();

    MdfStatus setRecordSize(MDF_UINT32 recordSize);
    MdfStatus setVariableRecordSize(MDF_UINT32 recordSize);
    MdfStatus addFrame(int channel, int type, KVMDF_TIMESTAMP ts, MDF_UINT8 *data);
    MdfStatus new_vector_can_frame(int channel, int type);
    int write_data(FILE *mdfFile);

    MuxChecker mux_checker;
    /* MuxChecker is used to organize those multiplexed signals, whose multiplexor value is same, to same dgNode.
     * E.g. if data is organized as [ mux [ value_sig1 ][ value_sig2 ] ], then two signals will share same multiplexor value,
     * thus there will be two signals in corresponding dgNode.
     * To create different dgNodes for signals with different mux_values is the only effective way of avoiding data duplication.*/
    MDF_UINT32 canId;
    MDF_UINT32 canMask;
    BQueue     *bq;
    int can_channel;
    int frame_type;

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
  };

  class fhNode : public NodeBase {
  public:
    FHBLOCK fh;
    mdNode *md;
    fhNode(int version);
    ~fhNode();
    int write(FILE *mdfFile);
    uint64_t size();
  };

  class atNode : public NodeBase {
  public:
    ATBLOCK   at;
    atNode(int version, const char *fname);
    ~atNode();

    int write(FILE *mdfFile);
    int write_data(FILE *mdfFile);
    uint64_t size();
    txNode* tx;
    atNode *next;
    std::string filename;
  };


  class hdNode : public NodeBase {
  public:
    HDBLOCK hd;
    txNode *tx;
    fhNode *fh;
    dgNode *dg, *cur_dg;
    atNode *at;
    MdfStatus new_vector_dg(int channel, int type);
    MdfStatus new_vector_can_frame(int channel, int type);
    hdNode(int version);
    ~hdNode();

    int setStartOfRecording(
      unsigned int year,
      unsigned char month,
      unsigned char day,
      unsigned char hour,
      unsigned char minute,
      unsigned char second
    );
    int setStartOfRecording(  uint64_t t);

    int addFrame(int channel, int type, KVMDF_TIMESTAMP ts, MDF_UINT8 *data);
    int write_data(FILE *mdfFile);

    int write(FILE *mdfFile);
    uint64_t size();
    void attach(const char *fname);
    // For signals
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
  int addMsg(MDF_UINT32 id, KVMDF_TIMESTAMP ts, MDF_UINT8 *data);
  int createMdf4Channel(int ch, int type, MDF_UINT32 id = CAN_ID_NOT_USED, const std::string& msgname = std::string());
  };

  class idNode : public NodeBase {
  public:
    IDBLOCK id;
    idNode(int version);
    ~idNode();

    int write(FILE *mdfFile);
    uint64_t size();
  };


  // Strings used by Vector MDF format
  const std::string MDF_PATH_SEP = ".";

  // Frame names
  const std::string CAN_ERROR_NAME = "CAN_ErrorFrame";
  const std::string CAN_FRAME_NAME = "CAN_DataFrame";
  const std::string CAN_REMOTE_NAME = "CAN_RemoteFrame";

  // Channel types
  const int CH_SINGLE_WIRE = 0;

  enum ChannelType {SINGLE_WIRE, KVASER_FLAGS, FLAGS_EX, DIR, WAKE_UP, SRR, R0, R1, EDL,
  BRS, ESI, ID, IDE, FRAME_DURATION, BIT_COUNT, DLC, DATA_LENGTH, DATA_BYTES,
  TIME_OFFSET_BRS, TIME_OFFSET_CRC, TX_ATTEMPTS_REQ, TX_ATTEMPTS_MAX, CRC, BTR_EXT_CFG


  };

  class ch {
    public:
      const std::string& getName(ChannelType type) {return mName[type];}
      const std::string& getComment(ChannelType type) {return mComment[type];}

      const char * getNameC(ChannelType type) {return getName(type).c_str();}
      const char * getCommentC(ChannelType type) {return getComment(type).c_str();}
    private:
      std::map<ChannelType, std::string> mName;
      std::map<ChannelType, std::string> mComment;

    public:
      ch() {
        mName   [SINGLE_WIRE] = "SingleWire";
        mComment[SINGLE_WIRE] = "Bit flag indicating a single wire operation.";
        mName   [KVASER_FLAGS] = "KvaserFlags";
        mComment[KVASER_FLAGS] = "Combination of Kvaser specific bit flags for the message.";
        mName   [FLAGS_EX] = "KvaserFlagsEx";
        mComment[FLAGS_EX] = "Combination of bit flags for the message.";
        mName   [DIR] = "Dir";
        mComment[DIR] = "Bit signal indicating the direction (Rx, Tx).";
        mName   [WAKE_UP] = "WakeUp";
        mComment[WAKE_UP] = "Bit flag indicating a wake-up message (high voltage).";
        mName   [SRR] = "SRR";
        mComment[SRR] = "Substitute Remote Request bit.";
        mName   [R0] = "R0";
        mComment[R0] = "Reserved bit R0.";
        mName   [R1] = "R1";
        mComment[R1] = "Reserved bit R1.";
        mName   [EDL] = "EDL";
        mComment[EDL] = "Extended Data Length bit.";
        mName   [BRS] = "BRS";
        mComment[BRS] = "Bit Rate Switch bit.";
        mName   [ESI] = "ESI";
        mComment[ESI] = "Error State Indicator bit.";
        mName   [ID] = "ID";
        mComment[ID] = "ID of the CAN message.";
        mName   [IDE] = "IDE";
        mComment[IDE] = "Identifier Extension bit.";
        mName   [FRAME_DURATION] = "FrameDuration";
        mComment[FRAME_DURATION] = "Duration for transmission of the frame in nanoseconds.";
        mName   [BIT_COUNT] = "BitCount";
        mComment[BIT_COUNT] = "Frame length in bits.";
        mName   [DLC] = "DLC";
        mComment[DLC] = "Data length code.";
        mName   [DATA_LENGTH] = "DataLength";
        mComment[DATA_LENGTH] = "Length of stored payload in bytes.";
        mName   [DATA_BYTES] = "DataBytes";
        mComment[DATA_BYTES] = "Payload data bytes containing the signal values.";
        mName   [TIME_OFFSET_BRS] = "TimeOffsetBRS";
        mComment[TIME_OFFSET_BRS] = "Time offset of bit rate switch within BRS field.";
        mName   [TIME_OFFSET_CRC] = "TimeOffsetCRCDel";
        mComment[TIME_OFFSET_CRC] = "Time offset of bit rate switch within CRC delimiter field.";
        mName   [TX_ATTEMPTS_REQ] = "TxAttemptsReq";
        mComment[TX_ATTEMPTS_REQ] = "Number of required transmission attempts.";
        mName   [TX_ATTEMPTS_MAX] = "TxAttemptsMax";
        mComment[TX_ATTEMPTS_MAX] = "Max number of transmission attempts.";
        mName   [CRC] = "CRC";
        mComment[CRC] = "Checksum of CAN frame.";
        mName   [BTR_EXT_CFG] = "BtrExtCfg";
        mComment[BTR_EXT_CFG] = "CAN- or CAN-FD bit timing configuration.";
      };
  };
}; // end namespace Mdf4Nodes


#endif
