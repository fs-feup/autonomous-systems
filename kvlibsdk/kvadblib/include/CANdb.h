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
**   Container Class for CAN Database Files / Michael Eisele
**
** -----------------------------------------------------------------------------
*/

#ifndef CANDB_H
#define CANDB_H

#include <inttypes.h>
#include <cstring>
typedef int64_t __int64 ;
#define _atoi64 atoll
#define _stricmp strcasecmp
#define stricmp strcasecmp

#include <cstdint>

// ****************************************************************************

#define CANDB_EXT_FLAG          0x80000000      // Extended CAN

// ****************************************************************************

#define CANDB_FILE_FORMAT_FLAG_READ     (1<<0)
#define CANDB_FILE_FORMAT_FLAG_WRITE    (1<<1)

// ****************************************************************************

int candb_first ();
int candb_last ();

// ****************************************************************************

class CANdb;
class CANdbCluster;
class CANdbFileIo;
class CANdbFileFormat;
class CANdbEnumValue;
class CANdbValue;
class CANdbSignal;
class CANdbMessage;
class CANdbNode;
class CANdbNodeEntry;
class CANdbMemoryRange;
class CANdbSignalEncoding;
class CANdbSignalEncodingScale;
class CANdbAttribute;
class CANdbAttributeDefinition;

// ****************************************************************************

typedef CANdbFileIo* (*CANdbFileIoBuildPtr) ();

// Deletion callback typedefs
typedef void (*CANdbClusterDeletionCallbackType)(CANdbCluster *pInstance);
typedef void (*CANdbMessageDeletionCallbackType)(CANdbMessage *pInstance);
typedef void (*CANdbSignalDeletionCallbackType)(CANdbSignal *pInstance);
typedef void (*CANdbNodeDeletionCallbackType)(CANdbNode *pInstance);
typedef void (*CANdbAttributeDeletionCallbackType)(CANdbAttribute *pInstance);
typedef void (*CANdbAttributeDefinitionDeletionCallbackType)(CANdbAttributeDefinition *pInstance);
typedef void (*CANdbEnumValueDeletionCallbackType)(CANdbEnumValue *pInstance);

#ifdef OPT_CANDB_INT64
#include "pf.h"
typedef PfUInt64 CANdbLargeUnsignedInteger;
//typedef unsigned __int64 CANdbLargeUnsignedInteger;
#endif

void candb_set_string (char **var, const char *s);

// ****************************************************************************

typedef void (*CANdbSignalCallback) (void *context, CANdbValue *value, unsigned int arg);


enum CANdbSignalType {
       CANDB_INVALID,
       CANDB_SIGNED,
       CANDB_UNSIGNED,
       CANDB_FLOAT,
       CANDB_DOUBLE
       //CANDB_STRING,
       //CANDB_MODE,
     };


enum CANdbEnvVarType {
       CANDB_EV_INVALID,
       CANDB_EV_INTEGER,
       CANDB_EV_FLOAT,
       CANDB_EV_STRING,
       CANDB_EV_DATA,
     };


enum CANdbEnvVarAccess {
      CANDB_EV_INVALID_ACCESS,
      CANDB_EV_UNRESTRICTED,
      CANDB_EV_READ,
      CANDB_EV_WRITE,
      CANDB_EV_READWRITE,
     };


enum CANdbAttributeType {
       CANDB_ATTR_TYPE_INVALID,
       CANDB_ATTR_TYPE_INTEGER,
       CANDB_ATTR_TYPE_HEX,
       CANDB_ATTR_TYPE_FLOAT,
       CANDB_ATTR_TYPE_ENUMERATION,
       CANDB_ATTR_TYPE_STRING
     };


enum CANdbAttributeOwner {
       CANDB_ATTR_OWNER_INVALID,
       CANDB_ATTR_OWNER_DB,
       CANDB_ATTR_OWNER_MESSAGE,
       CANDB_ATTR_OWNER_NODE,
       CANDB_ATTR_OWNER_SIGNAL,
       CANDB_ATTR_OWNER_ENVIRONMENT
     };


enum CANdbAttributeRemoveType {
       CANDB_ATTR_REM_TYPE_INVALID,
       CANDB_ATTR_REM_TYPE_NODE,
       CANDB_ATTR_REM_TYPE_MESSAGE,
       CANDB_ATTR_REM_TYPE_SIGNAL,
       CANDB_ATTR_REM_TYPE_ENVVAR,
       CANDB_ATTR_REM_TYPE_DB
     };


class CANdbEnumValue {
        friend class CANdbSignal;

        char *name;
        int  value;
        CANdbEnumValue *next;

      public:
        CANdbEnumValue (int value, const char *name);
        ~CANdbEnumValue ();

        static void register_deletion_callback(CANdbEnumValueDeletionCallbackType pFunc) {
          CANdbEnumValue::DeletionCallback = pFunc;
        }

        void set_name (const char*);
        void set_value (int v);

        void set_next (CANdbEnumValue* n) { next = n; }
        CANdbEnumValue* get_next (void) const { return next; }
        const char* get_name () const { return name; }
        int get_value () const { return value; }

        void extend_memory_range (CANdbMemoryRange& mr) const;

      private:
        static CANdbEnumValueDeletionCallbackType DeletionCallback;
      };


class CANdbDefaultMessageData {
        CANdbMessage    *message;

      public:
        CANdbDefaultMessageData ();
        virtual ~CANdbDefaultMessageData ();

        void set_message (CANdbMessage *m);
        CANdbMessage* get_message () const { return message; }

        virtual void data_changed () = 0;
        virtual void message_changed () = 0;
        virtual const unsigned char *get_data_ptr () const = 0;
      };


class CANdbMessageData {
        CANdbMessage    *message;
        unsigned char   *data_ptr;
        unsigned int    id,
                        flags,
                        data_length,
                        data_length_max;
        bool            valid;

      public:
        CANdbMessageData (unsigned int data_length_max = 8);
        virtual ~CANdbMessageData ();

        bool is_valid () const { return valid; }
        void set_valid (bool v) { valid = v; }

        void reset ();

        unsigned int get_data_length () const { return data_length; }
        unsigned int get_data_length_max () const { return data_length_max; }
        const unsigned char *get_data_ptr () const { return data_ptr; }
        void set_data (const unsigned char *data, unsigned int length);

        void set_id (unsigned int id);
        void set_flags (unsigned int flags);
        void set_message (CANdbMessage *m);

        CANdbMessage* get_message () const { return message; }
        unsigned int get_id () const { return id; }
        unsigned int get_flags () const { return flags; }

        virtual void data_changed ();
        virtual void message_changed ();
      };


class CANdbMessageDataEntry {
        CANdbMessageData        *message_data;
        CANdbMessageDataEntry   *next,
                                *prev,
                                *next_hash;

      public:
        CANdbMessageDataEntry (CANdbMessageData *md);
        ~CANdbMessageDataEntry ();

        CANdbMessageDataEntry *get_next () const { return next; }
        CANdbMessageDataEntry *get_prev () const { return prev; }
        CANdbMessageDataEntry *get_next_hash () const { return next_hash; }
        CANdbMessageData *get_message_data () const { return message_data; }

        void set_next (CANdbMessageDataEntry *n) { next = n; }
        void set_prev (CANdbMessageDataEntry *p) { prev = p; }
        void set_next_hash (CANdbMessageDataEntry *h) { next_hash = h; }
        void set_message_data (CANdbMessageData *md) { message_data = md; }
      };


class CANdbMessageDataContainer {
        unsigned int            hash_table_size;
        CANdbMessageDataEntry   **hash_table;
        CANdbMessageDataEntry   *first_entry,
                                *last_entry;

      public:
        CANdbMessageDataContainer (unsigned int hash_table_size = 2048);
        ~CANdbMessageDataContainer ();

        void clear ();
        //CANdbMessageData *create_and_store_entry_for_id (unsigned int id);
        void insert_message_data (unsigned int id, CANdbMessageData *md);
        void reset_message_data ();
        CANdbMessageData *find_message_data_by_id (unsigned int id);
        CANdbMessageData *find_message_data_by_id_or_create (unsigned int id);
      };


class CANdbAttributeDefinition {

      private:
        static CANdbAttributeDefinitionDeletionCallbackType DeletionCallback;
        struct EnumEntry {
                 char *name;
                 int  value;
                 EnumEntry *next;

                 EnumEntry (const char *n, int v) {
                   name = NULL;
                   value = v;
                   next = NULL;
                   if (n) {
                     name = new char [strlen (n) + 1];
                     strcpy (name, n);
                   }
                 }

                 ~EnumEntry () {
                   if (name) delete [] name;
                   name = NULL;
                   next = NULL;
                 }
               };

        CANdbAttributeType      type;
        CANdbAttributeOwner     owner;
        char                    *name;
        CANdbAttributeDefinition *next; // next definition
        CANdb                   *candb; // owner database

        EnumEntry *first_enum;

        union {
          struct {
            EnumEntry   *first_enum_entry,
                        *last_enum_entry;
            int         default_value;
          } enumeration;

          struct {
            unsigned int min,
                        max,
                        default_value;
          } hex;

          struct {
            int         min,
                        max,
                        default_value;
          } integer;

          struct {
            double      min,
                        max,
                        default_value;
          } fp;

          struct {
            char       *default_value;
          } string;

        } property;

      public:
        CANdbAttributeDefinition ();
        ~CANdbAttributeDefinition ();

        static void register_deletion_callback(CANdbAttributeDefinitionDeletionCallbackType pFunc) {
          CANdbAttributeDefinition::DeletionCallback = pFunc;
        }

        void clear_property (void);

        void set_name (const char *n);
        void set_type (CANdbAttributeType t);
        void set_owner (CANdbAttributeOwner o) { owner = o; }
        void set_next (CANdbAttributeDefinition *n) { next = n; }

        CANdb *get_candb (void) const { return candb; }
        void set_candb (CANdb *_candb) { candb = _candb; }

        const char* get_name (void) const { return name; }
        CANdbAttributeType get_type (void) const { return type; }
        CANdbAttributeOwner get_owner (void) const { return owner; }
        CANdbAttributeDefinition *get_next (void) const { return next; }

        int get_first_enumeration (void);
        int get_next_enumeration (void);

        int get_enumeration_value_by_name (const char *name) const;
        const char *get_enumeration_name_by_value (int v) const;
        int add_enumeration (const char *name, int value);
        int delete_enumeration (const char *name, int value);
        int get_enumeration_default (void) const { return property.enumeration.default_value; }

        void set_enumeration_default (int d) { property.enumeration.default_value = d; }

        unsigned int get_hex_min (void) const { return property.hex.min; }
        unsigned int get_hex_max (void) const { return property.hex.max; }
        unsigned int get_hex_default (void) const { return property.hex.default_value; }
        void set_hex_min (unsigned int m) { property.hex.min = m; }
        void set_hex_max (unsigned int m) { property.hex.max = m; }
        void set_hex_default (unsigned int d) { property.hex.default_value = d; }

        int get_integer_min (void) const { return property.integer.min; }
        int get_integer_max (void) const { return property.integer.max; }
        int get_integer_default (void) const { return property.integer.default_value; }
        void set_integer_min (int m) { property.integer.min = m; }
        void set_integer_max (int m) { property.integer.max = m; }
        void set_integer_default (int d) { property.integer.default_value = d; }

        double get_float_min (void) const { return property.fp.min; }
        double get_float_max (void) const { return property.fp.max; }
        double get_float_default (void) const { return property.fp.default_value; }
        void set_float_min (double m) { property.fp.min = m; }
        void set_float_max (double m) { property.fp.max = m; }
        void set_float_default (double d) { property.fp.default_value = d; }

        const char *get_string_default (void) const { return property.string.default_value; }
        void set_string_default (const char *d);
      };


class CANdbAttribute {
        CANdbAttributeDefinition *definition;
        CANdbAttributeType       type;
        CANdbAttribute           *next; // next attribute in list

        union {
          char          *string;
          unsigned int  hex;
          int           integer;
          int           enumeration;
          double        fp;
        } value;

      public:
        // explicit
        CANdbAttribute (CANdbAttributeDefinition *def);
        ~CANdbAttribute ();

        static void register_deletion_callback(CANdbAttributeDeletionCallbackType pFunc) {
          CANdbAttribute::DeletionCallback = pFunc;
        }

        CANdbAttributeType get_type (void) const { return type; }

        const char *get_string_value (void) const { return value.string; }
        unsigned int get_hex_value (void) const { return value.hex; }
        int get_integer_value (void) const { return value.integer; }
        int get_enumeration_value (void) const  { return value.enumeration; }
        double get_float_value (void) const  { return value.fp; }
        CANdbAttributeDefinition *get_definition (void) const { return definition; }
        CANdbAttribute *get_next (void) const { return next; }

        void set_string_value (const char *s);
        void set_hex_value (unsigned int v) { value.hex = v; }
        void set_integer_value (int v) { value.integer = v; }
        void set_enumeration_value (int v) { value.enumeration = v; }
        void set_float_value (double v) { value.fp = v; }
        void set_next (CANdbAttribute *n) { next = n; }

      private:
        static CANdbAttributeDeletionCallbackType DeletionCallback;
      };


class CANdbAttributeList {
        CANdbAttribute  *first_attribute,
                        *last_attribute;

      public:
        CANdbAttributeList ();
        ~CANdbAttributeList ();

        void insert (CANdbAttribute *attr);
        void remove (CANdbAttribute *attr);
        CANdbAttribute *find_by_name (const char *name) const;
        CANdbAttribute *find_by_definition (CANdbAttributeDefinition *definition);
        CANdbAttribute *get_first_attribute (void) const { return first_attribute; }
      };


class CANdbMemoryRange {
        struct mr {
          char  *start,
                *end;
          mr    *next;
        };

        mr              *first_mr,
                        *last_mr,
                        *current_mr;
        unsigned int    object_count;

        __int64 dist (mr *r, char *start, char *end);
        void extend (mr *r, char *start, char *end);

      public:
        CANdbMemoryRange ();
        ~CANdbMemoryRange ();

        void collect (void);

        void add_range (void *start, void *end);

        void add_range (void *start, int len) { add_range (start, (char*)start + len); }

        int set_first_mr (void) {
          current_mr = first_mr;
          if (!current_mr) return -1;
          return 0;
        }

        int set_next_mr (void) {
          if (current_mr) current_mr = current_mr->next;
          if (!current_mr) return -1;
          return 0;
        }

        void *get_start_address (void) const {
          if (current_mr) return current_mr->start;
          return NULL;
        }

        void *get_end_address (void) const {
          if (current_mr) return current_mr->end;
          return NULL;
        }

        unsigned int get_size (void) const {
          if (current_mr) return (unsigned int)(current_mr->end - current_mr->start);
          return 0;
        }

        unsigned int get_object_count (void) const { return object_count; }
      };


class CANdbValue {
      public:
        CANdbSignalType type;

        union {
          int           i;
          unsigned int  u;
          double        d;
        } value;
      };


class CANdbSignal {
        struct Notification {
                 CANdbSignalCallback callback;
                 void           *context;
                 Notification   *next;
               };

        char            *name,
                        *comment,
                        *unit;
        double          min_val,  // The (suggested) extremes of the physical value
                        max_val,
                        offset,
                        factor;

        // Formatting variables.
        //bool            fmtOk;
        //int             fmtWidth, // Width of raw value, including sign if there could be one.
        //                fmtDecimals;
        //bool            fmtSign;  // True if there should be room for a sign
        //char            fmtString[16]; // Used as a last resort for floats/doubles (usually '\0').
        //int             fmtPhysWidth, // Total width of ascii representation, possibly including '.' and sign
        //                fmtPhysDecimals; // Nmbr of decimals in case of float/double value.
        //bool            fmtPhysSign;
        //char            fmtPhysString[16]; // Used as a last resort (usually '\0').

        // That start_bit is a bit difficult. For signals in Intel
        // format everything is easy. For Motorola signals we count the
        // start bit position in the same way as for signals in Intel format,
        // but the message expands in a different byte direction. For
        // messages fitting into one byte there is no difference between
        // Intel and Motorola. This behaviour is opposite to the Vector
        // format in the DBC files.
        int             start_bit, //!< Start position in Intel standard or Motorola forward LSB.
                        length,
                        mode;
        bool            mode_signal,
                        //mode_dependent,
                        motorola;
        CANdbSignalType type,
                        scaled_type;
        CANdbEnumValue  *first_value,
                        *last_value,
                        *current_value;
        CANdbSignal     *next,
                        *sign_signal; // Points to the external Sign bit, currently used for BMW
        Notification    *first_notification,
                        *last_notification;
        CANdbAttributeList attributes;
        CANdbMessage    *message;
        CANdbNodeEntry  *first_node,
                        *last_node;
        void            *context;

        //void fixFormat(void);
        int store_value_uint_internal (unsigned char *can_data, int dlc, uint64_t value);
        int get_value_uint_internal (const unsigned char* can_data, int dlc, bool sign_extend, uint64_t &value) const;

#ifdef OPT_CANDB_INT64
        int store_value_uint64_internal (unsigned char *can_data, int dlc, CANdbLargeUnsignedInteger value);
        int get_value_uint64_internal (const unsigned char* can_data, int dlc, bool sign_extend, CANdbLargeUnsignedInteger &value) const;
#endif

      protected:
        void clear ();
        void init_members ();
        void copy_contents (const CANdbSignal &signal);

      public:
        CANdbSignal ();
        CANdbSignal (const CANdbSignal &signal);

        ~CANdbSignal ();

        static void register_deletion_callback(CANdbSignalDeletionCallbackType pFunc) {
          CANdbSignal::DeletionCallback = pFunc;
        }

        CANdbSignal &operator= (const CANdbSignal &right_side_signal);

        int add_value (int value, const char *name);
        int remove_value (CANdbEnumValue *val);
        CANdbEnumValue *get_first_value (void);
        CANdbEnumValue *get_first_value (void) const { return first_value; }
        CANdbEnumValue *get_next_value (void);
        bool has_symbolic_values (void) const { return first_value != NULL; }

        void update_scaled_type ();

        char *get_qualified_name (char *buffer, int buflen) const;

        void extend_memory_range (CANdbMemoryRange& mr) const;

        void set_name (const char *name);
        void set_comment (const char *comment);
        void set_unit (const char *unit);
        void set_min_val (double mv) { min_val = mv; }
        void set_max_val (double mv) { max_val = mv; }
        void set_offset (double o) { offset = o; update_scaled_type (); }
        void set_factor (double f) { factor = f; update_scaled_type (); }
        void set_start_bit (int sb) { start_bit = sb; }
        void set_length (int l) { length = l; }
        void set_mode (int m) { mode = m; }
        void set_type (CANdbSignalType t) { type = t; update_scaled_type (); }
        void set_next (CANdbSignal *n) { next = n; }
        void set_sign_signal (CANdbSignal *ss) { sign_signal = ss; }
        void set_mode_signal (bool m) { mode_signal = m; }
        void set_motorola (bool m) { motorola = m; }
        void set_message (CANdbMessage *m) { message = m; }
        void set_context (void *c) { context = c; }

        char *get_name (void) const { return name; }
        char *get_comment (void) const { return comment; }
        const char *get_unit (void) const { return unit; }
        double get_min_val (void) const { return min_val; }
        double get_max_val (void) const { return max_val; }
        double get_offset (void) const { return offset; }
        double get_factor (void) const { return factor; }
        int get_start_bit (void) const { return start_bit; }
        int get_length (void) const { return length; }
        int get_mode (void) const { return mode; }
        CANdbSignalType get_type (void) const { return type; }
        CANdbSignalType get_scaled_type (void) const { return scaled_type; }
        CANdbSignal *get_next (void) const { return next; }
        CANdbSignal *get_sign_signal (void) const { return sign_signal; }
        CANdbMessage *get_message (void) const { return message; }
        CANdb *get_candb (void) const;
        void *get_context () const { return context; }

        bool is_mode_dependent (void) const { return mode != -1; }
        bool is_mode_signal (void) const { return mode_signal; }
        bool is_motorola (void) const { return motorola; }

        bool is_signal_too_large (size_t len) const;

        int store_value (unsigned char *can_data, int dlc, uint64_t value);
        int store_value (unsigned char *can_data, int dlc, double value);
        int store_value_raw (unsigned char *can_data, int dlc, uint64_t value);

        int get_value_int (const unsigned char* can_data, int dlc, int64_t &value) const;
        int get_value_uint (const unsigned char* can_data, int dlc, uint64_t &value) const;
        int get_value_float (const unsigned char* can_data, int dlc, double &value) const;
        int get_value_double (const unsigned char* can_data, int dlc, double &value) const;
        const char* get_value_string (int value) const;
        const char* get_value_string (const unsigned char* can_data, int dlc) const;
        int convert_enum_value_to_int (const char *s);

        double phys2raw (double phys) { if (factor !=0) return (phys - offset) / factor; else return 0; }
        double raw2phys (double raw) { return raw * factor + offset; }

        int insert_notification (void *context, CANdbSignalCallback proc);
        int remove_notification (void *context);
        int remove_all_notifications ();
        int handle_can_msg (unsigned char* can_data, int dlc, unsigned int arg) const;

        CANdbAttributeList *get_attributes (void) { return &attributes; }
        int insert_attribute (CANdbAttribute *attr);

        int get_start_bit_for_display (CANdbMessage *msg) const;
        void set_start_bit_from_display (CANdbMessage *msg, int sb);

        void add_receive_node (CANdbNode *node);
        void remove_receive_node (CANdbNode *node);
        bool contains_node (CANdbNode *node);
        CANdbNodeEntry *get_first_receive_node_entry () const { return first_node; }

      private:
        static CANdbSignalDeletionCallbackType DeletionCallback;
      };


class CANdbMessage {
      public:
        enum j1939Type {
          NOT_J1939,
          J1939_V1,
          J1939_V2
        };

        CANdbMessage ();
        ~CANdbMessage ();

        static void register_deletion_callback(CANdbMessageDeletionCallbackType pFunc) {
          CANdbMessage::DeletionCallback = pFunc;
        }

        void extend_memory_range (CANdbMemoryRange& mr) const;

        char *get_qualified_name (char *buffer, int buflen) const;

        void set_name (const char *name);
        void set_send_node (CANdbNode *sender);
        void set_comment (const char *comment);
        void set_id (unsigned int i) { id = i; }
        void set_dlc (int l) { dlc = l; }
        void set_extended (bool e) { extended = e; }
        void set_j1939() { set_j1939Type(J1939_V2); }
        void set_next (CANdbMessage *msg) { next = msg; }
        void set_default_message_data (CANdbDefaultMessageData *dmd) { default_message_data = dmd; }
        void set_candb (CANdb *_candb) { candb = _candb; }

        void         update_pgn_mask(void);
        void         set_pgn_mask (unsigned int to) { pgn_mask = to; }
        unsigned int get_pgn_mask ()                { return pgn_mask; }

        CANdbSignal *get_mode_signal (void) const;
        CANdbDefaultMessageData *get_default_message_data () const { return default_message_data; }

        int get_signal_count (void) const { return signal_count; }
        int get_dlc (void) const { return dlc; }
        char* get_name (void) const { return name; }
        CANdbNode *get_send_node (void) const { return send_node; }
        char *get_comment (void) const { return comment; }
        unsigned int get_id (void) const { return id; }
        unsigned int get_id_and_ext (void) const { return (extended? id | 0x80000000 : id); }
        bool is_extended (void) const { return extended; }
        CANdbMessage *get_next (void) const { return next; }
        CANdb *get_candb (void) const { return candb; }

        bool      is_j1939      () const {return (j1939 != NOT_J1939);}
        j1939Type get_j1939Type () {return j1939;}
        void      set_j1939Type (j1939Type to) {j1939 = to;}

        bool is_canfd (void) const;
        bool is_brs (void) const;

        CANdbSignal *find_signal_by_name (const char *name) const;

        int insert_signal (CANdbSignal *signal);
        int remove_signal (CANdbSignal *signal);
        int delete_signal (CANdbSignal *signal);

        CANdbSignal *get_first_signal (void);
        CANdbSignal *get_next_signal (void);
        CANdbAttributeList *get_attributes (void) { return &attributes; }
        const CANdbAttributeList *get_const_attributes (void) const { return &attributes; }

        void setup ();

      private:
        char                    *name,
                                *comment;
        CANdbNode               *send_node;
        unsigned int             id;
        CANdbSignal             *first_signal,
                                *last_signal,
                                *current_signal;
        int                      signal_count;
        int                      dlc;
        unsigned int             pgn_mask; //same type as id
        bool                     extended;
        CANdbAttributeList       attributes;
        CANdbMessage            *next;
        CANdbDefaultMessageData *default_message_data;
        CANdb                   *candb; // pointer to parent object
        j1939Type                j1939;
        static CANdbMessageDeletionCallbackType DeletionCallback;
     };


class CANdbNode {
        char               *name,
                           *comment;
        CANdbNode          *next;
        bool                master;
        CANdbAttributeList  attributes;
        CANdb              *candb; // pointer to parent object

      public:
        CANdbNode ();
        ~CANdbNode ();

        static void register_deletion_callback(CANdbNodeDeletionCallbackType pFunc) {
          CANdbNode::DeletionCallback = pFunc;
        }

        void set_candb (CANdb *_candb) { candb = _candb; }
        CANdb *get_candb (void) const { return candb; }

        void set_name (const char *name);
        void set_comment (const char *comment);
        void set_next (CANdbNode *n) { next = n; }
        void set_master (bool m) { master = m; }

        const char *get_name (void) const { return name; }
        const char *get_comment (void) const { return comment; }
        CANdbNode *get_next (void) const { return next; }
        bool is_master (void) const { return master; }

        int insert_attribute (CANdbAttribute *attr);

        CANdbAttributeList *get_attributes (void) { return &attributes; }

      private:
        static CANdbNodeDeletionCallbackType DeletionCallback;
      };


class CANdbNodeEntry {
        CANdbNode       *node;
        CANdbNodeEntry  *next;

      public:
        CANdbNodeEntry (CANdbNode *node);
        ~CANdbNodeEntry ();

        void set_next (CANdbNodeEntry *n) { next = n; }
        void set_node (CANdbNode *n) { node = n; }

        CANdbNodeEntry *get_next (void) const { return next; }
        CANdbNode *get_node (void) const { return node; }
      };


class CANdbEnvVariable {
        char            *name,
                        *comment,
                        *unit,
                        *dummy_node;
        CANdbEnvVarType type;
        CANdbEnvVarAccess access;
        //CANdbSignalType type; // Or maybe a special type
        double          min_val,
                        max_val,
                        start_value,
                        data;
        bool            data_flag; // Set to true if data is assigned
        unsigned        num_id;
        CANdbNodeEntry  *first_node,
                        *last_node;
        CANdbEnumValue  *first_value,
                        *last_value;
        CANdbEnvVariable *next;
        CANdb           *candb;

        CANdbAttributeList attributes;

      public:
        CANdbEnvVariable ();
        ~CANdbEnvVariable ();

        void set_name (const char *s) { candb_set_string (&name, s); }
        void set_comment (const char *s) { candb_set_string (&comment, s); }
        void set_unit (const char *s) { candb_set_string (&unit, s); }
        void set_min_val (double mv) { min_val = mv; }
        void set_max_val (double mv) { max_val = mv; }
        void set_start_value (double sv) { start_value = sv; }
        void set_access (CANdbEnvVarAccess a) { access = a;}
        void set_type (CANdbEnvVarType t) { type = t; }
        void set_num_id (unsigned n) { num_id = n; }
        void set_dummy_node (const char *s) { candb_set_string (&dummy_node, s); }
        void set_data(double v) { data = v; data_flag = true; }
        void set_candb (CANdb *_candb) { candb = _candb; }

        char *get_name (void) const { return name; }
        char *get_comment (void) const { return comment; }
        char *get_unit (void) const { return unit; }
        double get_min_val (void) const { return min_val; }
        double get_max_val (void) const { return max_val; }
        double get_start_value (void) { return start_value; }
        CANdbEnvVarAccess get_access (void) const { return access; }
        CANdbEnvVarType get_type (void) const { return type; }
        unsigned get_num_id (void) { return num_id; }
        char *get_dummy_node (void) { return dummy_node; }
        double get_data_flag (void) { return data_flag; }
        double get_data (void) { return data; }
        CANdb *get_candb (void) const { return candb; }

        //void add_receive_node (const char *name);
        void add_receive_node (CANdbNode *node);
        CANdbNodeEntry *get_first_receive_node_entry () const { return first_node; }

        int add_value (int value, const char *name);
        const char* get_value_string (int value) const;
        CANdbEnumValue *get_first_value (void) const { return first_value; }
        bool has_symbolic_values (void) const { return first_value != 0; }

        void set_next (CANdbEnvVariable *n) { next = n; }
        CANdbEnvVariable *get_next (void) { return next; }

        CANdbAttributeList *get_attributes (void) { return &attributes; }
        int insert_attribute (CANdbAttribute *attr);

        char* get_qualified_name (char *buf, int buflen) const;
      };


class CANdbScheduleTableEntry {
        CANdbMessage            *message;
        //CANdbMessageData        *message_data;
        double                  delay;
        CANdbScheduleTableEntry *next;

      public:
        CANdbScheduleTableEntry (CANdbMessage *m, double d);
        ~CANdbScheduleTableEntry ();

        CANdbMessage *get_message () const { return message; }
        //CANdbMessageData *get_message_data () const { return message_data; }
        double get_delay () const { return delay; }
        CANdbScheduleTableEntry *get_next () const { return next; }

        void set_message (CANdbMessage *m) { message = m; }
        //void set_message_data (CANdbMessageData *md) { message_data = md; }
        void set_delay (double d) { delay = d; }
        void set_next (CANdbScheduleTableEntry *n) { next = n; }
      };


class CANdbScheduleTable {
        char                    *name;
        CANdbScheduleTableEntry *first_entry,
                                *last_entry;
        CANdbScheduleTable      *next;
        int                     entry_count;

      public:
        CANdbScheduleTable ();
        ~CANdbScheduleTable ();

        void set_name (const char *name);
        void set_next (CANdbScheduleTable *n) { next = n; }

        void insert_message (CANdbMessage *message, double delay);
        void remove_message (CANdbMessage *message);

        CANdbScheduleTableEntry *get_first_entry () const { return first_entry; }

        const char *get_name () const { return name; }
        CANdbScheduleTable *get_next () const { return next; }
        int get_entry_count () const { return entry_count; }
      };


class CANdbSignalEncodingScale {
        friend class CANdbSignalEncoding;

        int             min_value,
                        max_value;

        double          factor,
                        offset;

        CANdbSignalEncodingScale        *next;

      public:
        CANdbSignalEncodingScale (int _min_value, int _max_value, double _factor, double _offset) {
          min_value = _min_value;
          max_value = _max_value;
          factor = _factor;
          offset = _offset;
          next = NULL;
        }

        int get_min_value () const { return min_value; }
        int get_max_value () const { return max_value; }
        double get_factor () const { return factor; }
        double get_offset () const { return offset; }

        CANdbSignalEncodingScale *get_next () const { return next; }
      };


class CANdbSignalEncoding {
        char            *unit,
                        *name;

        CANdbEnumValue  *first_value,
                        *last_value;

        CANdbSignalEncodingScale *first_scale,
                                 *last_scale;

        CANdbSignalEncoding      *next;

      public:
        CANdbSignalEncoding ();
        ~CANdbSignalEncoding ();

        void set_name (const char *name);
        void set_unit (const char *name);
        void set_next (CANdbSignalEncoding *n) { next = n; }

        const char *get_name () const { return name; }
        const char *get_unit () const { return unit; }
        CANdbSignalEncoding *get_next () const { return next; }

        void add_scale (CANdbSignalEncodingScale *scale);
        void add_scale (int min_value, int max_value, double factor, double offset);
        CANdbSignalEncodingScale *get_first_scale () const { return first_scale; }

        void add_value (CANdbEnumValue *value);
        void add_value (int value, const char *name);
        const char* get_value_string (int value) const;
        CANdbEnumValue *get_first_value (void) { return first_value; }
      };


class CANdb {

      public:
        enum j1939Type {
          NOT_J1939,
          J1939_V1,
          J1939_V2
        };

        CANdb ();
        ~CANdb ();

        void clear_hash_table ();
        void rebuild_hash_table ();

        void update_j1939_flag ();

        bool      is_j1939 ()              const { return (j1939 != NOT_J1939); }
        j1939Type get_j1939Type ()         const { return j1939; }
        void      set_j1939Type (j1939Type to)   { j1939 = to; };

        void set_name (const char *name);
        void set_filename (const char *filename);
        void set_comment (const char *comment);
        void set_next (CANdb *n) { next = n; }

        char *get_name (void) const { return name; }
        char *get_filename (void) const { return filename; }
        char *get_comment (void) const { return comment; }
        int get_message_count (void) const { return message_count; }
        int get_node_count (void) const { return node_count; }
        CANdb *get_next () const { return next; }

        void extend_memory_range (CANdbMemoryRange& mr) const;

        CANdbMessage* find_message_by_name (const char *name) const;
        CANdbMessage* find_message_by_id (unsigned int id) const;
        CANdbMessage* find_message_by_pgn (unsigned int id_and_ext); //matches against PGN, if the message is in j1939-format

        bool contains_message (CANdbMessage *message) const;

        CANdbNode* find_node_by_name (const char *name) const;
        CANdbNode* get_first_node (void);
        CANdbNode* get_next_node (void);
        int insert_node (CANdbNode *node);
        int remove_node (CANdbNode *node);

        int insert_message (CANdbMessage *msg);
        int delete_message (CANdbMessage *msg);
        int remove_message (CANdbMessage *msg);

        CANdbMessage *get_first_message ();
        CANdbMessage *get_next_message ();

        int insert_env_variable (CANdbEnvVariable *var);
        CANdbEnvVariable *get_first_env_variable(void) { return first_env_variable; }
        CANdbEnvVariable *find_env_variable_by_name (const char *name) const;

        CANdbAttributeList *get_attributes (void) { return &attributes; }

        int insert_attribute_definition (CANdbAttributeDefinition *attr_def);
        int delete_attribute_definition (CANdbAttributeDefinition *attr_def);
        CANdbAttributeDefinition *find_attribute_definition_by_name (const char *name);
        CANdbAttributeDefinition *get_first_attribute_definition (void) const { return first_attr_def; }


        int handle_can_msg (unsigned long id,
                            unsigned char* can_data,
                            int dlc,
                            unsigned int arg) const;

      // schedule tables:
        bool has_schedule_table () const { return schedule_table_count != 0; }
        int get_schedule_table_count () const { return schedule_table_count; }
        void insert_schedule_table (CANdbScheduleTable *st);
        CANdbScheduleTable *find_schedule_table_by_name (const char *name) const;
        CANdbScheduleTable *get_first_schedule_table () const { return first_schedule_table; }

      // signal encoding
        bool has_signal_encoding () const { return signal_encoding_count != 0; }
        int get_signal_encoding_count () const { return signal_encoding_count; }
        void insert_signal_encoding (CANdbSignalEncoding *se);
        CANdbSignalEncoding *find_signal_encoding_by_name (const char *name) const;
        CANdbSignalEncoding *get_first_signal_encoding () const { return first_signal_encoding; }

      // static functions:
        static char *get_short_comment (const char *comment, unsigned int max_len, char *buffer);

 private:
        class HashEntry {
              public:
                CANdbMessage   *message;
                HashEntry      *next;

                HashEntry () {
                  message = NULL;
                  next = NULL;
                }
              };

        int             message_count;
        int             node_count;
        char            *name,
                        *comment,
                        *filename;

        CANdbMessage    *first_message,
                        *last_message,
                        *current_message;

        CANdbEnvVariable *first_env_variable,
                         *last_env_variable;

        // hash table
        unsigned int    hash_table_size;
        HashEntry       *hash_table;

        CANdbNode       *first_node,
                        *last_node,
                        *current_node;

        CANdb           *next;

        CANdbAttributeDefinition *first_attr_def,
                                 *last_attr_def;

        CANdbAttributeList attributes;

        // schedule tables
        CANdbScheduleTable      *first_schedule_table,
                                *last_schedule_table;
        int                     schedule_table_count;

        // signal encoding
        CANdbSignalEncoding     *first_signal_encoding,
                                *last_signal_encoding;
        int                     signal_encoding_count;

        j1939Type               j1939;

      };


class CANdbCluster {
        CANdb           *first_db,
                        *last_db,
                        *current_db,
                        *current_db_node;
        //char            *default_name;

        unsigned int    candb_count;

      public:
        CANdbCluster ();
        ~CANdbCluster ();

        static void register_deletion_callback(CANdbClusterDeletionCallbackType pFunc) {
          CANdbCluster::DeletionCallback = pFunc;
        }

        unsigned int get_candb_count () const { return candb_count; }

        //void set_default_name (const char *);
        //const char *get_default_name () const { return default_name; }

        void add_db (CANdb *db);
        bool remove_db (CANdb *db);
        bool delete_db (CANdb *db);

        CANdb *get_first_candb ();
        CANdb *get_next_candb ();
        CANdb *get_current_candb ();

        CANdbMessage *get_first_message ();
        CANdbMessage *get_next_message ();
        CANdbMessage *find_message_by_name (const char *name) const;
        CANdbMessage *find_message_by_id (unsigned int id) const;
        CANdbMessage *find_message_by_pgn (unsigned int id); //matches against PGN, if the database says that the frame is in j1939-format

        CANdbNode *get_first_node ();
        CANdbNode *get_next_node ();
        CANdbNode *find_node_by_name (const char *name) const;

        CANdb* find_candb_by_message (CANdbMessage *message) const;

        bool is_j1939_message (const char *message_name) const;

        int handle_can_msg (unsigned long id,
                            unsigned char* can_data,
                            int dlc,
                            unsigned int arg) const;

      private:
        static CANdbClusterDeletionCallbackType DeletionCallback;
      };


class CANdbMessageLogger {
      public:
        CANdbMessageLogger ();
        virtual ~CANdbMessageLogger ();

        virtual void info (int line_number, const char *filename, const char *msg) = 0;
        virtual void error (int line_number, const char *filename, const char *msg) = 0;
        virtual void warning (int line_number, const char *filename, const char *msg) = 0;
      };


class CANdbFileIo {
        char                    *filename;
        bool                    read_ok;
        CANdbFileFormat         *file_format;
        CANdbMessageLogger      *message_logger;
        char                    *errorlog;

      protected:
        int lineno;

        virtual void info (const char *msg, ...);
        virtual void error (const char *msg, ...);
        virtual void warning (const char *msg, ...);

      public:
        CANdbFileIo (const char *filename);
        virtual ~CANdbFileIo ();

        void set_message_logger (CANdbMessageLogger *ml) { message_logger = ml; }

        bool is_read_ok () const { return read_ok; }
        void set_read_ok (bool r) { read_ok = r; }

        void set_filename (const char *filename);
        const char *get_filename () const { return filename; }

        const char *get_db_name () const;
        void set_dbname (CANdb *db);

        void set_file_format (CANdbFileFormat *ff);
        CANdbFileFormat *get_file_format () const { return file_format; }

        virtual int read_file (CANdb *db) = 0;
        virtual int save_file (CANdb *db) = 0;

        void set_errorlog(const char* msg);
        void append_errorlog(const char* msg);
        void get_errorlog(char *msg, unsigned int *buflen);
      };


class CANdbFileFormat {
        char                    *name,
                                *file_extension,
                                *description_short,
                                *description_long;

        CANdbFileIoBuildPtr     build_ptr;

        CANdbFileFormat         *next;

        unsigned int            flags;

      public:
        CANdbFileFormat (const char *name,
                         const char *description_short,
                         const char *description_long,
                         const char *file_extension,
                         unsigned long flags,
                         CANdbFileIoBuildPtr build_ptr);

        ~CANdbFileFormat ();

        CANdbFileFormat *get_next () const { return next; }

        const char *get_name () const { return name; }
        const char *get_file_extension () const { return file_extension; }

        const char *get_description_short () const { return description_short; }
        const char *get_description_long () const { return description_long; }
        unsigned int get_flags () const { return flags; }

      public:
        // Statische Funktionen
        static void register_candb_file_format (CANdbFileFormat *ff);
        static CANdbFileFormat* get_first_candb_file_format ();
        static CANdbFileFormat* get_candb_file_format_by_name (const char *name);
        static CANdbFileFormat* get_candb_file_format_by_extension (const char *extension);
        static CANdbFileIo* build (const char *name);
      };


// ****************************************************************************

#endif // CANDB_H

