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
 ** ---------------------------------------------------------------------------
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <stdarg.h>
#include <math.h>
#include <cstdint>
#include <inttypes.h>
#include <string.h>
#include <cstring>

#include "CANdb.h"

// ****************************************************************************

#define MAX_MSG          200         // Max length of error messages

#define EXT_FLAG         0x80000000  // If set in a message CAN id in file, it is an extended message.

// ****************************************************************************
//had to move these constants from CANdb.h because repo mdf2csv uses an older compiler
//that gives error C2864.

const char* Attribute_VFrameFormat      = "VFrameFormat";
const int   VFrameFormat_ExtendedCAN    = 2;
const int   VFrameFormat_J1939          = 3;
const int   VFrameFormat_StandardCAN_FD = 14;
const int   VFrameFormat_ExtendedCAN_FD = 15;

// Important for DIAdem stuff - don't remove it !!!
static int candb_first_variable = 1;

// ****************************************************************************

// Important for DIAdem stuff - don't remove it !!!
int global_dummy = 0;

// ****************************************************************************

static CANdbFileFormat *candb_file_format_first = NULL,
                       *candb_file_format_last = NULL;

// ****************************************************************************

// static deletion callback member variable declarations
CANdbClusterDeletionCallbackType CANdbCluster::DeletionCallback = NULL;
CANdbMessageDeletionCallbackType CANdbMessage::DeletionCallback = NULL;
CANdbSignalDeletionCallbackType CANdbSignal::DeletionCallback = NULL;
CANdbNodeDeletionCallbackType CANdbNode::DeletionCallback = NULL;
CANdbAttributeDeletionCallbackType CANdbAttribute::DeletionCallback = NULL;
CANdbAttributeDefinitionDeletionCallbackType CANdbAttributeDefinition::DeletionCallback = NULL;
CANdbEnumValueDeletionCallbackType CANdbEnumValue::DeletionCallback = NULL;

// Important for DIAdem stuff - don't remove it !!!
int candb_first ()
{
  global_dummy = 1;
  return 0;
} // candb_first


// ****************************************************************************


/* Frees if necessary, allocates memory for and assigns a string variable.
*/
void candb_set_string (char **var, const char *s)
{
  if (*var) delete [] *var;
  *var = NULL;
  if (!s) return;
  *var = new char [strlen (s) + 1];
  strcpy (*var, s);
} /* candb_set_string */


// ****************************************************************************


CANdbDefaultMessageData::CANdbDefaultMessageData ()
{
  //memset (data, 0, sizeof (data));
  message = NULL;
} // CANdbDefaultMessageData::CANdbDefaultMessageData


CANdbDefaultMessageData::~CANdbDefaultMessageData ()
{
  if (message) message->set_default_message_data (NULL);
  message = NULL;
} // CANdbDefaultMessageData::~CANdbDefaultMessageData


void CANdbDefaultMessageData::set_message (CANdbMessage *m)
{
  if (message == m) return;
  if (message) message->set_default_message_data (NULL);
  message = m;
  if (message) message->set_default_message_data (this);
  message_changed ();
} // CANdbDefaultMessageData::set_message


// ****************************************************************************


CANdbMessageData::CANdbMessageData (unsigned int _data_length_max)
{
  valid = false;
  message = NULL;
  id = 0xffffffff; // INVALID CAN-ID
  flags = 0;
  data_length = 0;
  data_length_max = _data_length_max;
  data_ptr = new unsigned char [data_length_max];
} // CANdbMessageData::CANdbMessageData


CANdbMessageData::~CANdbMessageData ()
{
  valid = false;
  delete [] data_ptr;
  data_ptr = NULL;
  data_length_max = 0;
  data_length = 0;
  message = NULL;
} // CANdbMessageData::~CANdbMessageData


void CANdbMessageData::reset ()
{
  set_valid (false);
  memset (data_ptr, 0, data_length_max);
  data_length = 0;
} // CANdbMessageData::reset


void CANdbMessageData::set_data (const unsigned char *data, unsigned int length)
{
  if (length > data_length_max) length = data_length_max;
  memcpy (data_ptr, data, length);
  data_length = length;
  set_valid (true);
  data_changed ();
} // CANdbMessageData::set_data


void CANdbMessageData::set_message (CANdbMessage *m)
{
  if (message == m) return;
  message = m;
  message_changed ();
} // CANdbMessageData::set_message


void CANdbMessageData::set_id (unsigned int new_id)
{
  if (id == new_id) return;
  id = new_id;
  message_changed ();
} // CANdbMessageData::set_id


void CANdbMessageData::set_flags (unsigned int new_flags)
{
  if (flags == new_flags) return;
  flags = new_flags;
  message_changed ();
} // CANdbMessageData::set_flags


void CANdbMessageData::data_changed ()
{
} // CANdbMessageData::data_changed


void CANdbMessageData::message_changed ()
{
} // CANdbMessageData::message_changed


// ****************************************************************************


CANdbMessageDataEntry::CANdbMessageDataEntry (CANdbMessageData *md)
{
  message_data = md;
  next = NULL;
  prev = NULL;
  next_hash = NULL;
} // CANdbMessageDataEntry::CANdbMessageDataEntry


CANdbMessageDataEntry::~CANdbMessageDataEntry ()
{
  if (message_data) delete message_data;
  message_data = NULL;
  next = NULL;
  prev = NULL;
  next_hash = NULL;
} // CANdbMessageDataEntry::~CANdbMessageDataEntry


// ****************************************************************************


CANdbMessageDataContainer::CANdbMessageDataContainer (unsigned int _hash_table_size)
{
  first_entry = NULL;
  last_entry = NULL;
  hash_table_size = _hash_table_size;
  hash_table = new CANdbMessageDataEntry* [hash_table_size];
  for (unsigned int i = 0; i < hash_table_size; ++i) {
    hash_table [i] = NULL;
  }
} // CANdbMessageDataContainer::CANdbMessageDataContainer


CANdbMessageDataContainer::~CANdbMessageDataContainer ()
{
  clear ();
  if (hash_table) delete [] hash_table;
  hash_table = NULL;
} // CANdbMessageDataContainer::~CANdbMessageDataContainer


void CANdbMessageDataContainer::clear ()
{
  CANdbMessageDataEntry *mde = first_entry;
  while (mde) {
    CANdbMessageDataEntry *nmde = mde->get_next ();
    delete mde;
    mde = nmde;
  }
  first_entry = NULL;
  last_entry = NULL;

  for (unsigned int i = 0; i < hash_table_size; ++i) {
    hash_table [i] = NULL;
  }
} // CANdbMessageDataContainer::clear


void CANdbMessageDataContainer::reset_message_data ()
{
  CANdbMessageDataEntry *mde = first_entry;
  while (mde) {
    CANdbMessageData *md = mde->get_message_data ();
    if (md) md->reset ();
    mde = mde->get_next ();
  }
} // CANdbMessageDataContainer::reset_message_data


void CANdbMessageDataContainer::insert_message_data (unsigned int id, CANdbMessageData *md)
{
  md->set_id (id); // ????

  CANdbMessageDataEntry *mde = new CANdbMessageDataEntry (md);
  unsigned int index = id % hash_table_size;
  mde->set_next_hash (hash_table [index]);
  hash_table [index] = mde;
  if (first_entry && last_entry) {
    last_entry->set_next (mde);
    last_entry = mde;
  }
  else first_entry = last_entry = mde;
} // CANdbMessageDataContainer::insert_message_data


CANdbMessageData *CANdbMessageDataContainer::find_message_data_by_id (unsigned int id)
{
  unsigned int index = id % hash_table_size;
  CANdbMessageData *md = NULL;
  CANdbMessageDataEntry *mde = hash_table [index];
  while (mde) {
    md = mde->get_message_data ();
    if (md) {
      CANdbMessage *m = md->get_message ();
      if (m && (m->get_id () == id)) break;
      if (m->get_id () == id) break;
    }
    mde = mde->get_next_hash ();
    md = NULL;
  }
  return md;
} // CANdbMessageDataContainer::find_message_data_by_id


CANdbMessageData *CANdbMessageDataContainer::find_message_data_by_id_or_create (unsigned int id)
{
  CANdbMessageData *md = find_message_data_by_id (id);
  if (!md) {
    md = new CANdbMessageData ();
    md->set_id (id);
    insert_message_data (id, md);
  }
  return md;
} // CANdbMessageDataContainer::find_message_data_by_id_or_create


// ****************************************************************************


//#define PAGE_SIZE       16384
#define MAX_DIST            256

CANdbMemoryRange::CANdbMemoryRange ()
{
  object_count = 0;
  first_mr = NULL;
  last_mr = NULL;
  current_mr = NULL;
} // CANdbMemoryRange::CANdbMemoryRange


CANdbMemoryRange::~CANdbMemoryRange ()
{
  mr *m = first_mr;
  while (m) {
    mr *nm = m->next;
    delete m;
    m = nm;
  }
  first_mr = NULL;
  last_mr = NULL;
  current_mr = NULL;
} // CANdbMemoryRange::~CANdbMemoryRange




__int64 CANdbMemoryRange::dist (mr *r, char *start, char *end)
{
  if (end < r->start) return (__int64) (r->start - end);
  if (start > r->end) return (__int64) (start - r->end);
  return 0;
} // CANdbMemoryRange::dist


void CANdbMemoryRange::extend (mr *r, char *start, char *end)
{
  if (start < r->start) r->start = start;
  if (end > r->end) r->end = end;
} // CANdbMemoryRange::extend


void CANdbMemoryRange::collect (void)
{
  mr *m = first_mr;
  while (m) {
    mr *n = m->next;
    if (n && (dist (m, n->start, n->end) < MAX_DIST)) {
      extend (m, n->start, n->end);
      m->next = n->next;
      if (n == last_mr) last_mr = m;
      delete n;
      //n = m->next;
    }
    else m = n;
  }
} // CANdbMemoryRange::collect


void CANdbMemoryRange::add_range (void *start, void *end)
{
  ++object_count;

  mr *m = first_mr,
     *min = NULL;
  __int64 min_dist = 0;

  while (m) {
    if (!min) {
      min = m;
      min_dist = dist (min, (char*)start, (char*)end);
    }
    else {
      if (dist (m, (char*)start, (char*)end) < min_dist) {
        min = m;
        min_dist = dist (min, (char*)start, (char*)end);
        if (min_dist == 0) break;
      }
    }
    m = m->next;
  }

  if (min && (min_dist < MAX_DIST)) {
    extend (min, (char*)start, (char*)end);
    return;
  }

  m = new mr;
  m->start = (char*)start;
  m->end = (char*)end;
  m->next = NULL;

  if (!first_mr) first_mr = last_mr = m;
  else {
    mr *c = first_mr,
       *p = NULL;

    while (c) {
      if (m->start > c->start) { // insert
        p = c;
        c = c->next;
      }
      else {
        if (p) {
          p->next = m;
          m->next = c;
        }
        else {
          m->next = c;
          first_mr = m;
        }
        break;
      }
    }
    if (!c) { // append at end
      last_mr->next = m;
      last_mr = m;
    }
  }
} // CANdbMemoryRange::add_range


// ****************************************************************************


CANdbScheduleTableEntry::CANdbScheduleTableEntry (CANdbMessage *m, double d)
{
  //message_data = NULL;
  message = m;
  delay = d;
  next = NULL;
} // CANdbScheduleTableEntry::CANdbScheduleTableEntry


CANdbScheduleTableEntry::~CANdbScheduleTableEntry ()
{
  //message_data = NULL;
  message = NULL;
  delay = 0;
  next = NULL;
} // CANdbScheduleTableEntry::~CANdbScheduleTableEntry


// ****************************************************************************


CANdbScheduleTable::CANdbScheduleTable ()
{
  name = NULL;
  first_entry = NULL;
  last_entry = NULL;
  next = NULL;
  entry_count = 0;
} // CANdbScheduleTable::CANdbScheduleTable


CANdbScheduleTable::~CANdbScheduleTable ()
{
  set_name (NULL);
  first_entry = NULL;
  last_entry = NULL;
  next = NULL;
  entry_count = 0;
} // CANdbScheduleTable::~CANdbScheduleTable


void CANdbScheduleTable::set_name (const char *n)
{
  if (name) delete [] name;
  name = NULL;
  if (n) {
    name = new char [strlen (n) + 1];
    strcpy (name, n);
  }
} // CANdbScheduleTable::set_name


void CANdbScheduleTable::insert_message (CANdbMessage *message, double delay)
{
  CANdbScheduleTableEntry *ste = new CANdbScheduleTableEntry (message, delay);
  if (first_entry && last_entry) {
    last_entry->set_next (ste);
    last_entry = ste;
    ++entry_count;
  }
  else {
    first_entry = last_entry = ste;
    entry_count = 1;
  }
} // CANdbScheduleTable::insert_message


void CANdbScheduleTable::remove_message (CANdbMessage *message)
{
  CANdbScheduleTableEntry *ste = first_entry,
                          *pste = NULL;
  while (ste) {
    if (ste->get_message () == message) {
      if (pste) {
        pste->set_next (ste->get_next ());
        if (ste == last_entry) last_entry = pste;
      }
      else {
        first_entry = ste->get_next ();
        if (ste == last_entry) last_entry = NULL;
      }
      entry_count--;
    }
    pste = ste;
    ste = ste->get_next ();
  }
} // CANdbScheduleTable::remove_message


// ****************************************************************************


CANdbSignalEncoding::CANdbSignalEncoding ()
{
  unit = NULL;
  name = NULL;
  first_value = NULL;
  last_value = NULL;
  first_scale = NULL;
  last_scale = NULL;
  next = NULL;
} // CANdbSignalEncoding::CANdbSignalEncoding


CANdbSignalEncoding::~CANdbSignalEncoding ()
{
  CANdbEnumValue *v = first_value;
  while (v) {
    CANdbEnumValue *nv = v->get_next ();
    delete v;
    v = nv;
  }
  first_value = last_value = NULL;

  CANdbSignalEncodingScale *s = first_scale;
  while (s) {
    CANdbSignalEncodingScale *ns = s->get_next ();
    delete s;
    s = ns;
  }
  first_scale = last_scale = NULL;

  set_name (NULL);
  set_unit (NULL);
  next = NULL;
} // CANdbSignalEncoding::~CANdbSignalEncoding


void CANdbSignalEncoding::set_name (const char *_name)
{
  if (name) delete [] name;
  name = NULL;
  if (_name) {
    name = new char [strlen (_name) + 1];
    strcpy (name, _name);
  }
} // CANdbSignalEncoding::set_name


void CANdbSignalEncoding::set_unit (const char *_unit)
{
  if (unit) delete [] unit;
  unit = NULL;
  if (_unit) {
    unit = new char [strlen (_unit) + 1];
    strcpy (unit, _unit);
  }
} // CANdbSignalEncoding::set_unit


void CANdbSignalEncoding::add_scale (CANdbSignalEncodingScale *scale)
{
  scale->next = NULL;
  if (first_scale && last_scale) {
    last_scale->next = scale;
    last_scale = scale;
  }
  else first_scale = last_scale = scale;
} // CANdbSignalEncoding::add_scale


void CANdbSignalEncoding::add_scale (int min_value, int max_value, double factor, double offset)
{
  CANdbSignalEncodingScale *scale =
    new CANdbSignalEncodingScale (min_value, max_value, factor, offset);
  add_scale (scale);
} // CANdbSignalEncoding::add_scale


void CANdbSignalEncoding::add_value (CANdbEnumValue *value)
{
  value->set_next (NULL);
  if (first_value && last_value) {
    last_value->set_next (value);
    last_value = value;
  }
  else first_value = last_value = value;
} // CANdbSignalEncoding::add_value


void CANdbSignalEncoding::add_value (int value, const char *name)
{
  CANdbEnumValue *enum_value = new CANdbEnumValue (value, name);
  add_value (enum_value);
} // CANdbSignalEncoding::add_value


const char* CANdbSignalEncoding::get_value_string (int value) const
{
  CANdbEnumValue *v = first_value;
  while (v) {
    if (v->get_value () == value) return v->get_name ();
    v = v->get_next ();
  }
  return NULL;
} // CANdbSignalEncoding::get_value_string


// ****************************************************************************


CANdbEnumValue::CANdbEnumValue (int _value, const char *_name)
{
  name = NULL;
  next = NULL;
  value = _value;
  set_name (_name);
} // CANdbEnumValue::CANdbEnumValue


CANdbEnumValue::~CANdbEnumValue ()
{
  set_name (NULL);
  next = NULL;
  if (DeletionCallback != NULL) {
    DeletionCallback(this);
  }
} // CANdbEnumValue::~CANdbEnumValue


void CANdbEnumValue::set_name (const char*_name)
{
  if (name) delete [] name;
  name = NULL;
  if (_name) {
    name = new char [strlen (_name) + 1];
    strcpy (name, _name);
  }
} // CANdbEnumValue::set_name


void CANdbEnumValue::set_value (int new_value)
{
  value = new_value;
} // CANdbEnumValue::set_value


void CANdbEnumValue::extend_memory_range (CANdbMemoryRange& mr) const
{
  mr.add_range ((void*)this, sizeof (CANdbEnumValue));
  if (name) mr.add_range (name, (int) (strlen (name) + 1));
} // CANdbEnumValue::extend_memory_range


// ****************************************************************************


CANdbAttributeDefinition::CANdbAttributeDefinition ()
{
  type = CANDB_ATTR_TYPE_INVALID;
  owner = CANDB_ATTR_OWNER_INVALID;
  name = NULL;
  next = NULL;
} // CANdbAttributeDefinition::CANdbAttributeDefinition


CANdbAttributeDefinition::~CANdbAttributeDefinition ()
{
  set_name (NULL);
  clear_property ();
  if (DeletionCallback != NULL) {
    DeletionCallback(this);
  }

} // CANdbAttributeDefinition::~CANdbAttributeDefinition


void CANdbAttributeDefinition::clear_property (void)
{
  if (type == CANDB_ATTR_TYPE_ENUMERATION) {
    EnumEntry *e = property.enumeration.first_enum_entry,
              *ne = NULL;
    while (e) {
      ne = e->next;
      delete e;
      e = ne;
    }
    property.enumeration.first_enum_entry = NULL;
    property.enumeration.last_enum_entry = NULL;
  }
  else if (type == CANDB_ATTR_TYPE_STRING) {
    if (property.string.default_value) delete [] property.string.default_value;
    property.string.default_value = NULL;
  }
  type = CANDB_ATTR_TYPE_INVALID;
} // CANdbAttributeDefinition::clear_property


void CANdbAttributeDefinition::set_name (const char *n)
{
  if (name) delete [] name;
  name = NULL;
  if (n) {
    name = new char [strlen (n) + 1];
    strcpy (name, n);
  }
} // CANdbAttributeDefinition::set_name


void CANdbAttributeDefinition::set_type (CANdbAttributeType t)
{
  clear_property ();
  type = t;
  switch (type) {

    case CANDB_ATTR_TYPE_INTEGER:
         property.integer.min = 0;
         property.integer.max = 0;
         property.integer.default_value = 0;
         break;

    case CANDB_ATTR_TYPE_HEX:
         property.hex.min = 0;
         property.hex.max = 0;
         property.hex.default_value = 0;
         break;

    case CANDB_ATTR_TYPE_FLOAT:
         property.fp.min = 0;
         property.fp.max = 0;
         property.fp.default_value = 0;
         break;

    case CANDB_ATTR_TYPE_ENUMERATION:
         property.enumeration.first_enum_entry = NULL;
         property.enumeration.last_enum_entry = NULL;
         property.enumeration.default_value = 0;
         break;

    case CANDB_ATTR_TYPE_STRING:
         property.string.default_value = NULL;
         break;

    case CANDB_ATTR_TYPE_INVALID:
         break;
  }
} // CANdbAttributeDefinition::set_type

int CANdbAttributeDefinition::get_first_enumeration (void)
{
  if (type != CANDB_ATTR_TYPE_ENUMERATION) return -1;
  EnumEntry *e = property.enumeration.first_enum_entry;

  //printf ("get_first_enumeration 1\n");
  while (e) {
    if (e->name) {
      //printf ("get_first_enumeration 2 %s\n", e->name);
      if (strcmp ("reserved", e->name) == 0) {
	e = e->next;
      } else {
	first_enum = e->next;
	return e->value;
      }
    }
  }
  return -1;
} // CANdbAttributeDefinition::get_first_enumeration

int CANdbAttributeDefinition::get_next_enumeration (void)
{
  if (type != CANDB_ATTR_TYPE_ENUMERATION) return -1;
  EnumEntry *e = first_enum;

  //printf ("get_next _enumeration 1\n");
  while (e) {
    if (e->name) {
      //printf ("get_next_enumeration 2 %s\n", e->name);
      if (strcmp ("reserved", e->name) == 0) {
	e = e->next;
      } else {
	first_enum = e->next;
	return e->value;
      }
    }
  }
  return -1;
} // CANdbAttributeDefinition::get_first_enumeration

int CANdbAttributeDefinition::get_enumeration_value_by_name (const char *name) const
{
  if (type != CANDB_ATTR_TYPE_ENUMERATION) return -1;
  EnumEntry *e = property.enumeration.first_enum_entry;

  //printf ("get_enumeration_value_by_name 1 %s\n", name);
  while (e) {
    if (e->name) {
      //printf ("get_enumeration_value_by_name 2 %s\n", e->name);
      if (strcmp (name, e->name) == 0) {
	return e->value;
      }
    }
    e = e->next;
  }
  return -1;
} // CANdbAttributeDefinition::get_enumeration_value_by_name


const char *CANdbAttributeDefinition::get_enumeration_name_by_value (int v) const
{
  if (type != CANDB_ATTR_TYPE_ENUMERATION) return NULL;
  EnumEntry *e = property.enumeration.first_enum_entry;

  //printf ("get_enumeration_name_by_value 1 %d\n", v);
  while (e) {
    //printf ("get_enumeration_name_by_value 2 %d\n", v);
    if (e->value == v) return e->name;
    e = e->next;
  }
  return NULL;
} // CANdbAttributeDefinition::get_enumeration_name_by_value


int CANdbAttributeDefinition::add_enumeration (const char *name, int value)
{
  if (type != CANDB_ATTR_TYPE_ENUMERATION || !name || value < 0) {
    return -1;
  }

  EnumEntry *entry;

  //printf ("add_enumeration 1 %s  %d\n", name, value);

  if (value > 0) {
    if (NULL == property.enumeration.first_enum_entry) {
      property.enumeration.first_enum_entry = new EnumEntry("reserved", 0);
      first_enum = property.enumeration.first_enum_entry;
    }
    EnumEntry *cur_entry = property.enumeration.first_enum_entry;
    for (int i = 1; i < value; ++i) {
      if (NULL == cur_entry) {
        return -1;
      }
      if (NULL == cur_entry->next) {
        cur_entry->next = new EnumEntry("reserved", i);
      }
      cur_entry = cur_entry->next;
    }

    // What if cur_entry->next->next isn't==NULL
    EnumEntry* next_next = NULL;
    if (cur_entry->next != NULL) {
      next_next = cur_entry->next->next;
      delete cur_entry->next;
    }
    cur_entry->next = new EnumEntry(name, value);
    entry = cur_entry->next;
    entry->next = next_next;
  }
  else {
    EnumEntry* next_next = NULL;
    if (NULL != property.enumeration.first_enum_entry) {
      next_next = property.enumeration.first_enum_entry->next;
      delete property.enumeration.first_enum_entry;
    }
    property.enumeration.first_enum_entry = new EnumEntry(name, 0);
    entry = property.enumeration.first_enum_entry;
    entry->next = next_next;
  }

  if (NULL == entry) {
    return -1;
  }

  if (NULL == property.enumeration.last_enum_entry || entry->value > property.enumeration.last_enum_entry->value) {
    property.enumeration.last_enum_entry = entry;
  }

  return 0;
} // CANdbAttributeDefinition::add_enumeration


int CANdbAttributeDefinition::delete_enumeration (const char *name, int value)
{
  if (type != CANDB_ATTR_TYPE_ENUMERATION) return -1;
  EnumEntry *e = property.enumeration.first_enum_entry,
            *pe = NULL;

  //printf ("delete_enumeration 1 %s  %d\n", name, value);

  while (e) {
    if ((!e->name || (strcmp (e->name, name) == 0)) &&
        ((value == -1) || (value == e->value))) {

      if (pe) {
	pe->next = e->next;
      } else {
	property.enumeration.first_enum_entry = e->next;
      }

      if (first_enum == e) {
	get_next_enumeration();
      }

      if (property.enumeration.last_enum_entry == e) property.enumeration.last_enum_entry = pe;
      if (!property.enumeration.first_enum_entry) property.enumeration.last_enum_entry = NULL;

      delete e;

      return 0;
    }
    pe = e;
    e = e->next;
  }
  return -1;
} // CANdbAttributeDefinition::delete_enumeration


void CANdbAttributeDefinition::set_string_default (const char *d)
{
  if (property.fp.default_value) delete [] property.string.default_value;
  property.string.default_value = NULL;
  if (d) {
    property.string.default_value = new char [strlen (d) + 1];
    strcpy (property.string.default_value, d);
  }
} // CANdbAttributeDefinition::set_string_default


// ****************************************************************************


CANdbAttribute::CANdbAttribute (CANdbAttributeDefinition *def)
{
  definition = def;
  type = CANDB_ATTR_TYPE_INVALID;
  next = NULL;
  if (definition) type = definition->get_type ();
  if (type == CANDB_ATTR_TYPE_STRING) value.string = NULL;
} // CANdbAttribute::CANdbAttribute


CANdbAttribute::~CANdbAttribute ()
{
  if ((type == CANDB_ATTR_TYPE_STRING) && value.string) {
    delete [] value.string;
    value.string = NULL;
  }
  if (DeletionCallback != NULL) {
    DeletionCallback(this);
  }

} // CANdbAttribute::~CANdbAttribute


void CANdbAttribute::set_string_value (const char *s)
{
  if (type != CANDB_ATTR_TYPE_STRING) return;
  if (value.string) delete [] value.string;
  value.string = NULL;
  if (s) {
    value.string = new char [strlen (s) + 1];
    strcpy (value.string, s);
  }
} // CANdbAttribute::set_string_value


// ****************************************************************************


CANdbAttributeList::CANdbAttributeList ()
{
  first_attribute = NULL;
  last_attribute = NULL;
} // CANdbAttributeList::CANdbAttributeList


CANdbAttributeList::~CANdbAttributeList ()
{
  CANdbAttribute *a = first_attribute;
  while (a) {
    CANdbAttribute *na = a->get_next ();
    delete a;
    a = na;
  }
  first_attribute = NULL;
  last_attribute = NULL;
} // CANdbAttributeList::~CANdbAttributeList


void CANdbAttributeList::insert (CANdbAttribute *attr)
{
  if (!attr) return;
  if (!first_attribute) first_attribute = last_attribute = attr;
  else {
    last_attribute->set_next (attr);
    last_attribute = attr;
  }
} // CANdbAttributeList::insert


void CANdbAttributeList::remove (CANdbAttribute *attr)
{
  CANdbAttribute *a = first_attribute;
  CANdbAttribute *pa = NULL;
  while (a) {
    if (a == attr) {
      if (pa) {
        pa->set_next (a->get_next ());
        if (a == last_attribute) last_attribute = pa;
      }
      else {
        first_attribute = a->get_next ();
        if (a == last_attribute) last_attribute = NULL;
      }
    }
    pa = a;
    a = a->get_next ();
  }
} // CANdbAttributeList::remove


CANdbAttribute *CANdbAttributeList::find_by_name (const char *name) const
{
  CANdbAttribute *a = first_attribute;
  while (a) {
    CANdbAttributeDefinition *ad = a->get_definition ();
    if (ad->get_name () && (strcmp (ad->get_name (), name) == 0)) break;
    a = a->get_next ();
  }
  return a;
} // CANdbAttributeList::find_by_name


CANdbAttribute *CANdbAttributeList::find_by_definition (CANdbAttributeDefinition *definition)
{
  CANdbAttribute *a = first_attribute;
  while (a) {
    if (definition == a->get_definition ()) break;
    a = a->get_next ();
  }
  return a;
} // CANdbAttributeList::find_by_definition


// ****************************************************************************


CANdbSignal::CANdbSignal ()
{
  init_members ();
} // CANdbSignal::CANdbSignal


CANdbSignal &CANdbSignal::operator= (const CANdbSignal &right_side_signal)
{
  copy_contents (right_side_signal);
  return *this;
} // CANdbSignal::operator=


CANdbSignal::~CANdbSignal ()
{
  clear ();
  if (DeletionCallback != NULL) {
    DeletionCallback(this);
  }

} // CANdbSignal::~CANdbSignal


void CANdbSignal::init_members ()
{
  name = NULL;
  comment = NULL;
  unit = NULL;
  min_val = 0;
  max_val = 0;
  offset = 0;
  factor = 1.0;
  start_bit = 0;
  length = 0;
  mode = -1;
  motorola = false;
  //mode_dependent = false;
  mode_signal = false;
  type = CANDB_INVALID;
  first_value = NULL;
  last_value = NULL;
  first_notification = NULL;
  last_notification = NULL;
  next = NULL;
  sign_signal = NULL;
  message = NULL;
  first_node = NULL;
  last_node = NULL;
  context = 0;
} // CANdbSignal::init_members


void CANdbSignal::copy_contents (const CANdbSignal &signal)
{
  clear ();

  set_name (signal.get_name ());
  set_comment (signal.get_comment ());
  set_unit (signal.get_unit ());

  set_min_val (signal.get_min_val ());
  set_max_val (signal.get_max_val ());
  set_offset (signal.get_offset ());
  set_factor (signal.get_factor ());
  set_start_bit (signal.get_start_bit ());
  set_length (signal.get_length ());
  set_mode (signal.get_mode ());
  set_type (signal.get_type ());
  set_motorola (signal.is_motorola ());

  /*CANdbEnumValue*/
  //CANdbEnumValue *v = first_value;
  //while (v) {
  //  CANdbEnumValue *nv = v->next;
  //  delete v;
  //  v = nv;
  //}
  //first_value = NULL;
  //last_value = NULL;

  CANdbEnumValue *v = signal.get_first_value ();
  while (v) {
    add_value (v->get_value (), v->get_name ());
    v = v->get_next ();
  }

  /*Notifications*/
  //remove_all_notifications ();
  //Notification *n = signal.first_notification;
  //while (n) {
  //  this->insert_notification (n->context, n->callback);
  //  n = n->next;
  //}

  /*CANdbNodeEntry*/
  //CANdbNodeEntry *node = first_node;
  //while (node) {
  //  CANdbNodeEntry *nnode = node->get_next ();
  //  delete node;
  //  node = nnode;
  //}
  //first_node = NULL;
  //last_node = NULL;

  CANdbNodeEntry *node = signal.get_first_receive_node_entry ();
  while (node) {
    add_receive_node (node->get_node ());
    node = node->get_next ();
  }
} // CANdbSignal::copy_contents


void CANdbSignal::clear ()
{
  type = CANDB_INVALID;
  next = NULL;
  sign_signal = NULL;
  if (comment) delete [] comment;
  comment = NULL;
  if (name) delete [] name;
  name = NULL;
  if (unit) delete [] unit;
  unit = NULL;
  CANdbEnumValue *v = first_value;
  while (v) {
    CANdbEnumValue *nv = v->next;
    delete v;
    v = nv;
  }
  first_value = NULL;
  last_value = NULL;

  Notification *n = first_notification;
  while (n) {
    Notification *nn = n->next;
    delete n;
    n = nn;
  }
  first_notification = NULL;
  last_notification = NULL;

  CANdbNodeEntry *node = first_node;
  while (node) {
    CANdbNodeEntry *nnode = node->get_next ();
    delete node;
    node = nnode;
  }
  first_node = NULL;
  last_node = NULL;
} // CANdbSignal::clear


char *CANdbSignal::get_qualified_name (char *buffer, int buflen) const
{
  if (name && buffer && (buflen > 0)) {
    strcpy (buffer, "");
    buflen -= (int) strlen (name);
    buflen -= 1; // for the dot.
    if (message) {
      message->get_qualified_name (buffer, buflen);
      buflen -= (int) strlen (buffer);
      if ((buflen > 0) && ((int)strlen (buffer) > 0)) {
        strcat (buffer, ".");
        strcat (buffer, name);
      }
      else strcpy (buffer, "");
    }
  }
  return buffer;
} // CANdbSignal::get_qualified_name


void CANdbSignal::add_receive_node (CANdbNode *node)
{
  if (!node) return;

  // first check if node is already in the list
  CANdbNodeEntry *ne = first_node;
  while (ne) {
    if (ne->get_node () == node) return;
    ne = ne->get_next ();
  }

  // insert node...
  ne = new CANdbNodeEntry (node);
  if (!first_node) first_node = last_node = ne;
  else {
    last_node->set_next (ne);
    last_node = ne;
  }
} // CANdbSignal::add_receive_node


void CANdbSignal::remove_receive_node (CANdbNode *node)
{
  CANdbNodeEntry *ne = first_node,
                 *pne = NULL;
  while (ne) {
    if (ne->get_node () == node) {
      if (pne) {
        pne->set_next (ne->get_next ());
        if (ne == last_node) last_node = pne;
      }
      else {
        first_node = ne->get_next ();
        if (ne == last_node) last_node = NULL;
      }
      return;
    }
    pne = ne;
    ne = ne->get_next ();
  }
} // CANdbSignal::remove_receive_node


bool CANdbSignal::contains_node (CANdbNode *node)
{
  if (!node) return false;

  // check if node is in the list
  CANdbNodeEntry *ne = first_node;
  while (ne) {
    if (ne->get_node () == node) return true;
    ne = ne->get_next ();
  }

  // not found...
  return false;
} // CANdbSignal::contains_node

CANdbEnumValue *CANdbSignal::get_first_value (void)
{
  current_value = first_value;
  return current_value;
} // CANdbSignal::get_first_value

CANdbEnumValue *CANdbSignal::get_next_value (void)
{
  if (current_value) current_value = current_value->get_next ();
  return current_value;
} // CANdbSignal::get_next_signal

int CANdbSignal::add_value (int val, const char *name)
{
  if (!name) return -1;
  if (get_value_string (val)) return -1;
  CANdbEnumValue *v = new CANdbEnumValue (val, name);
  if (!first_value) first_value = last_value = v;
  else {
    last_value->next = v;
    last_value = v;
  }
  return 0;
}

int CANdbSignal::remove_value (CANdbEnumValue *val)
{
  CANdbEnumValue *pv = NULL,
                 *v = first_value;
  while (v) {
    if (v == val) {
      if (pv) {
        pv->set_next (v->get_next ());
        if (last_value == v) last_value = pv;
      }
      else {
        first_value = v->get_next ();
        if (!first_value) last_value = NULL;
      }
      v->set_next (NULL);
      delete val;
      return 0;
    }
    pv = v;
    v = v->get_next ();
  }
  return -1;
} // CANdbSignal::remove_value

void CANdbSignal::extend_memory_range (CANdbMemoryRange& mr) const
{
  mr.add_range ((void*)this, sizeof (CANdbSignal));
  if (name) mr.add_range (name, (int) strlen (name) + 1);
  if (comment) mr.add_range (comment, (int) strlen (comment) + 1);

  CANdbEnumValue *v = first_value;
  while (v) {
    v->extend_memory_range (mr);
    v = v->next;
  }

  Notification *n = first_notification;
  while (n) {
    mr.add_range (n, sizeof (Notification));
    n = n->next;
  }
} // CANdbSignal::extend_memory_range


void CANdbSignal::set_comment (const char *c)
{
  if (comment) delete [] comment;
  comment = NULL;
  if (!c) return;
  comment = new char [strlen (c) + 1];
  strcpy (comment, c);
} // CANdbSignal::set_comment


void CANdbSignal::set_name (const char *n)
{
  if (name) delete [] name;
  name = NULL;
  if (!n) return;
  name = new char [strlen (n) + 1];
  strcpy (name, n);
} // CANdbSignal::set_name


void CANdbSignal::set_unit (const char *u)
{
  if (unit) delete [] unit;
  unit = NULL;
  if (!u) return;
  unit = new char [strlen (u) + 1];
  strcpy (unit, u);
} // CANdbSignal::set_unit


static unsigned char byte_mask [] = {
         0x01, 0x03, 0x07, 0x0f, 0x1f, 0x3f, 0x7f, 0xff,
         0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
         0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
         0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
         0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
         0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
         0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
         0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
       };


static uint64_t sign_mask [] = {
         0xffffffffffffffff, 0xfffffffffffffffe, 0xfffffffffffffffc, 0xfffffffffffffff8,
         0xfffffffffffffff0, 0xffffffffffffffe0, 0xffffffffffffffc0, 0xffffffffffffff80,
         0xffffffffffffff00, 0xfffffffffffffe00, 0xfffffffffffffc00, 0xfffffffffffff800,
         0xfffffffffffff000, 0xffffffffffffe000, 0xffffffffffffc000, 0xffffffffffff8000,
         0xffffffffffff0000, 0xfffffffffffe0000, 0xfffffffffffc0000, 0xfffffffffff80000,
         0xfffffffffff00000, 0xffffffffffe00000, 0xffffffffffc00000, 0xffffffffff800000,
         0xffffffffff000000, 0xfffffffffe000000, 0xfffffffffc000000, 0xfffffffff8000000,
         0xfffffffff0000000, 0xffffffffe0000000, 0xffffffffc0000000, 0xffffffff80000000,
         0xffffffff00000000, 0xfffffffe00000000, 0xfffffffc00000000, 0xfffffff800000000,
         0xfffffff000000000, 0xffffffe000000000, 0xffffffc000000000, 0xffffff8000000000,
         0xffffff0000000000, 0xfffffe0000000000, 0xfffffc0000000000, 0xfffff80000000000,
         0xfffff00000000000, 0xffffe00000000000, 0xffffc00000000000, 0xffff800000000000,
         0xffff000000000000, 0xfffe000000000000, 0xfffc000000000000, 0xfff8000000000000,
         0xfff0000000000000, 0xffe0000000000000, 0xffc0000000000000, 0xff80000000000000,
         0xff00000000000000, 0xfe00000000000000, 0xfc00000000000000, 0xf800000000000000,
         0xf000000000000000, 0xe000000000000000, 0xc000000000000000, 0x8000000000000000,
       };


// Store a value as an integer of some size.
// The data type of the signal is not considered, just the bit pattern
// is inserted in the CAN message.
int CANdbSignal::store_value_uint_internal (unsigned char *can_data, int dlc, uint64_t value)
{
  int sb = start_bit,
      len = length,
      // mlen = dlc *8,
      byte_pos = sb / 8,
      bit = sb % 8;

  if ((byte_pos < 0) || (byte_pos >= dlc) || (len > 64)) return -1;

  while (len > 0) {
    unsigned char mask = byte_mask [len - 1];
    int plen = 8 - bit;
    if (plen > len) plen = len;
    mask = (mask << bit) & 0xff;
    unsigned char v = (unsigned char) ((value << bit) & 0xff);
    value = value >> plen;
    len -= plen;
    can_data [byte_pos] = (can_data [byte_pos] & ~mask) | (v & mask);
    bit = 0;
    if (motorola) {
      --byte_pos;
      if (byte_pos < 0) break;
    }
    else { // Intel
      ++byte_pos;
      if (byte_pos >= dlc) break;
    }
  }
  return 0;
} // CANdbSignal::store_value_uint_internal


#ifdef OPT_CANDB_INT64
int CANdbSignal::store_value_uint64_internal (unsigned char *can_data, int dlc, CANdbLargeUnsignedInteger value)
{
  int sb = start_bit,
      len = length,
      //mlen = dlc *8,
      byte_pos = sb / 8,
      bit = sb % 8;

  if ((byte_pos < 0) || (byte_pos >= dlc) || (len > 64)) return -1;

  while (len > 0) {
    unsigned char mask = byte_mask [len - 1];
    int plen = 8 - bit;
    if (plen > len) plen = len;
    mask = (mask << bit) & 0xff;
    unsigned int v = (unsigned int) ((value << bit) & 0xff);
    value = value >> plen;
    len -= plen;
    can_data [byte_pos] = (can_data [byte_pos] & ~mask) | (v & mask);
    bit = 0;
    if (motorola) {
      --byte_pos;
      if (byte_pos < 0) break;
    }
    else { // Intel
      ++byte_pos;
      if (byte_pos >= dlc) break;
    }
  }
  return 0;
} // CANdbSignal::store_value_uint64_internal
#endif


// Store value as is. Regardless of type.
int CANdbSignal::store_value_raw (unsigned char *can_data, int dlc, uint64_t value)
{
  if (!can_data) return -1;

  return store_value_uint_internal (can_data, dlc, value);
} // CANdbSignal::store_value_raw


// Store a value as an integer of some size.
// If called for a float or double signal, the call is passed on
// store_value (unsigned char *, int, double) instead.
int CANdbSignal::store_value (unsigned char *can_data, int dlc, uint64_t value)
{
  if (!can_data) return -1;

  if (type == CANDB_FLOAT || type == CANDB_DOUBLE) {
    return store_value (can_data, dlc, (double) value);
  }
  else {
    // There is no need to handle the sign in any way.
    // If value is negative, but small enough to fit in the bits
    // available, there will be 1's in the extra positions to the left.
    return store_value_uint_internal (can_data, dlc, value);
  }
} // CANdbSignal::store_value


// Store a float or double value.
int CANdbSignal::store_value (unsigned char *can_data, int dlc, double value)
{
  if (!can_data) return -1;

  if (type == CANDB_DOUBLE) {
    if (length != 64) {
      return -1; // Error!
    }
    return store_value_uint_internal (can_data, dlc, *((uint64_t*) &value));
  }
  else if (type == CANDB_FLOAT) {
    float f = (float) value;
#ifdef OPT_CN
    CompilerAssert (sizeof(f) == 4);
#endif
    // memcpy(&iValue, &f, sizeof(unsigned long));
    unsigned int iValue = *((unsigned int*) &f);
    return store_value_uint_internal (can_data, dlc, iValue);
  }
  else {
    // Straight cast from negative double to unsigned is implementation defined.
    // It returns 0 on gcc arm, passing it via signed int is portable.
    return store_value (can_data, dlc, (uint64_t)(int64_t)value); // Just cast it to an integer.
  }
} // CANdbSignal::store_value


// Return the value as an uint regardless of the actual type.
// No sign extension is made.
int CANdbSignal::get_value_uint_internal (const unsigned char* can_data,
                                          int dlc,
                                          bool sign_extend,
                                          uint64_t &value) const
{
  int len = length,
      sb = start_bit,
      bit = sb % 8,
      byte_pos = sb / 8,
      wpos = 0,
      retval = 0;

  value = 0;

  if ((sb < 0) || (len < 1) || (len > 64)) return -1;

  if (motorola) {
    while (len > 0) {
      unsigned char mask = byte_mask [len - 1];

      if ((byte_pos >= dlc) || (byte_pos < 0)) { // out of range
        value = 0;
        retval = -1;
        break;
      }

      value |= (uint64_t)((can_data [byte_pos] >> bit) & mask) << wpos;
      int plen = 8 - bit;
      if (plen > len) plen = len;
      len -= plen;
      wpos += plen;
      --byte_pos;
      bit = 0;
    }
  }
  else {
    while (len > 0) {
      unsigned char mask = byte_mask [len - 1];

      if (byte_pos >= dlc) {  // out of range
        value = 0;
        retval = -1;
        break;
      }

      value |= (uint64_t)((can_data [byte_pos] >> bit) & mask) << wpos;
      int plen = 8 - bit;
      if (plen > len) plen = len;
      len -= plen;
      wpos += plen;
      ++byte_pos;
      bit = 0;
    }
  }

  if (sign_extend &&
      (retval == 0) &&
      (value & ((uint64_t)1 << (wpos - 1)))) { // negative number: extend sign
    value |= sign_mask [wpos - 1];
  }

  return retval;
} // CANdbSignal::get_value_uint_internal


#ifdef OPT_CANDB_INT64
// Return the value as an uint64 regardless of the actual type.
// No sign extension is made.
int CANdbSignal::get_value_uint64_internal (const unsigned char* can_data,
                                            int dlc,
                                            bool sign_extend,
                                            CANdbLargeUnsignedInteger &value) const
{
  int len = length,
      sb = start_bit,
      bit = sb % 8,
      byte_pos = sb / 8,
      wpos = 0,
      retval = 0;

  value = 0;

  if ((sb < 0) || (len < 1) || (len > 64)) return -1;

  if (motorola) {
    while (len > 0) {
      unsigned int mask = byte_mask [len - 1];

      if ((byte_pos >= dlc) || (byte_pos < 0)) { // out of range
        value = 0;
        retval = -1;
        break;
      }

      value |= (CANdbLargeUnsignedInteger) ((can_data [byte_pos] >> bit) & mask) << wpos;
      int plen = 8 - bit;
      if (plen > len) plen = len;
      len -= plen;
      wpos += plen;
      --byte_pos;
      bit = 0;
    }
  }
  else {
    while (len > 0) {
      unsigned int mask = byte_mask [len - 1];

      if (byte_pos >= dlc) {  // out of range
        value = 0;
        retval = -1;
        break;
      }

      value |= (CANdbLargeUnsignedInteger) ((can_data [byte_pos] >> bit) & mask) << wpos;
      int plen = 8 - bit;
      if (plen > len) plen = len;
      len -= plen;
      wpos += plen;
      ++byte_pos;
      bit = 0;
    }
  }

  if (sign_extend &&
      (retval == 0) &&
      (value & ((CANdbLargeUnsignedInteger) 1 << (wpos - 1)))) { // negative number: extend sign
    value |= sign_mask [wpos - 1];
  }

  return retval;
} // CANdbSignal::get_value_uint64_internal
#endif


int CANdbSignal::get_value_int (const unsigned char* can_data, int dlc, int64_t &value) const
{
#ifdef QDEBUG
  if (type != CANDB_SIGNED)
    QDEBUG(("Called %s.get_value_int() for data type %d", name, type));
#endif
  uint64_t uvalue = 0;
  int retval = get_value_uint_internal (can_data, dlc, true, uvalue);
  value = uvalue;
  return retval;
} // CANdbSignal::get_value_int


int CANdbSignal::get_value_uint (const unsigned char* can_data, int dlc, uint64_t &value) const
{
#ifdef QDEBUG
  if (type != CANDB_UNSIGNED)
    QDEBUG(("Called %s.get_value_uint() for data type %d", name, type));
#endif
  int retval = get_value_uint_internal (can_data, dlc, false, value);
  return retval;
} // CANdbSignal::get_value_uint


// Interpret the value as a float number.
// Floats are stored in 32 bits, so we start by extracting a ulong.
// If the bitcount is different than 32, it is really an error, but we
// leave this check to dbe, doing our best here which is to fill with
// zeroes at the left
int CANdbSignal::get_value_float (const unsigned char* can_data, int dlc, double &value) const
{
#ifdef QDEBUG
  if (type != CANDB_FLOAT)
    QDEBUG(("Called %s.get_value_float() for data type %d", name, type));
#endif

  value = 0;

  uint64_t uvalue = 0;
  int retval = get_value_uint_internal (can_data, dlc, false, uvalue);

  if (retval == 0) {
    float v_f = 0;
    * ((unsigned int*) &v_f) = (unsigned int) uvalue;
    value = v_f;
  }

  return retval;
} // CANdbSignal::get_value_float


// Interpret the value as a double number.
// Doubles are stored in 64 bits.
// Should this not be the case, we return -1.
int CANdbSignal::get_value_double (const unsigned char* can_data, int /* dlc */, double &value) const
{
#ifdef QDEBUG
  if (type != CANDB_DOUBLE)
    QDEBUG(("Called %s.get_value_double() for data type %d", name, type));
#endif
  if (length != (sizeof (value) * 8)) return -1; // Error!

  value = 0;
  uint64_t uvalue = 0;
  int retval = get_value_uint_internal (can_data, 8, false, uvalue);

  if (retval == 0) {
    double dvalue;
    *((uint64_t*)&dvalue) = uvalue;
    value = dvalue;
  }

  return retval;
} // CANdbSignal::get_value_double


// Get a description associated with a value
// (Used e.g. for enumeration types.)
const char* CANdbSignal::get_value_string (int value) const
{
  CANdbEnumValue *v = first_value;
  while (v) {
    if (v->value == value) return v->name;
    v = v->next;
  }
  return NULL;
} // CANdbSignal::get_value_string


// Get a description associated with a value in the message
const char* CANdbSignal::get_value_string (const unsigned char* can_data, int dlc) const
{
  int64_t ivalue = 0;
  int res = get_value_int (can_data, dlc, ivalue);
  if (res == 0) return get_value_string ((int)ivalue);
  return NULL;
} // CANdbSignal::get_value_string


int CANdbSignal::convert_enum_value_to_int (const char *s)
{
  CANdbEnumValue *v = first_value;
  while (v) {
    if (strcmp (v->name, s) == 0) return v->value;
    v = v->next;
  }
  return (int)_atoi64 (s);
} // CANdbSignal::convert_enum_value_to_int


int CANdbSignal::insert_notification (void *context, CANdbSignalCallback proc)
{
  Notification *n = new Notification;
  n->context = context;
  n->callback = proc;
  n->next = NULL;
  if (!first_notification) first_notification = last_notification = n;
  else {
    last_notification->next = n;
    last_notification = n;
  }
  return 0;
} // CANdbSignal::insert_notification


int CANdbSignal::remove_notification (void *context)
{
  Notification *n = first_notification,
               *pn = NULL;
  while (n) {
    if (n->context == context) {
      Notification *nn = n->next;
      if (pn) pn->next = nn;
         else first_notification = nn;
      if (n == last_notification) last_notification = pn;
      if (!first_notification) last_notification = NULL;
      delete n;
      n = nn;
    }
    else {
      pn = n;
      n = n->next;
    }
  }
  return 0;
} // CANdbSignal::remove_notification


int CANdbSignal::remove_all_notifications ()
{
  Notification *n = first_notification;
  while (n) {
    Notification *nn = n->next;
    delete n;
    n = nn;
  }
  first_notification = NULL;
  last_notification = NULL;
  return 0;
} // CANdbSignal::remove_all_notifications


int CANdbSignal::handle_can_msg (unsigned char* can_data, int dlc, unsigned int arg) const
{
  if (!first_notification) return 0; // Do not build the value if there are no one to notify.

  CANdbValue value;
  int res = 0;

#ifdef CANDB_SCALE_VALUES
  value.type = get_scaled_type ();
#else
  value.type = get_type ();
#endif

  if (type == CANDB_SIGNED) {
    if (sign_signal) {
      uint64_t uvalue = 0;
      res = get_value_uint_internal (can_data, dlc, false, uvalue);
      value.value.i = (int) uvalue;
      if (res == 0) {
        int64_t ivalue;
        int sign = 0;
        res = sign_signal->get_value_int (can_data, dlc, ivalue);
        sign = (int) ivalue;
        if ((res == 0) && sign) value.value.i *= -1;
      }
    }
    else {
      int64_t ivalue;
      res = get_value_int (can_data, dlc, ivalue);
      value.value.i = (int) ivalue;
    }

#ifdef CANDB_SCALE_VALUES
    if (value.type == CANDB_DOUBLE) {
      value.value.d = (double) (value.value.i * get_factor () + get_offset ());
    }
    else {
      value.value.i = (int) (value.value.i * get_factor () + get_offset ());
    }
#endif
  }
  else if (type == CANDB_UNSIGNED) {
    uint64_t uvalue = 0;
    res = get_value_uint (can_data, dlc, uvalue);
    value.value.u = (unsigned int) uvalue;

#ifdef CANDB_SCALE_VALUES
    if (value.type == CANDB_DOUBLE) {
      value.value.d = (double) (value.value.u * get_factor () + get_offset ());
    }
    else {
      value.value.u = (unsigned int) (value.value.u * get_factor () + get_offset ());
    }
#endif
  }

  else if (type == CANDB_FLOAT) {
    res = get_value_float (can_data, dlc, value.value.d);

#ifdef CANDB_SCALE_VALUES
    value.value.d = (value.value.d * get_factor ()) + get_offset ();
#endif
  }

  else if (type == CANDB_DOUBLE) {
    res = get_value_double (can_data, dlc, value.value.d);

#ifdef CANDB_SCALE_VALUES
    value.value.d = (value.value.d * get_factor ()) + get_offset ();
#endif
  }
  if (res == 0) {
    Notification *n = first_notification;
    while (n) {
      n->callback (n->context, &value, arg);
      n = n->next;
    }
  }
  return res;
} // CANdbSignal::handle_can_msg


int CANdbSignal::get_start_bit_for_display (CANdbMessage *msg) const
{
  if (is_motorola ()) {
    int bit = start_bit % 8,
        byte = start_bit / 8;
    return (msg->get_dlc () - 1 - byte) * 8 + bit;
  }
  else return start_bit;
} // CANdbSignal::get_start_bit_for_display


void CANdbSignal::set_start_bit_from_display (CANdbMessage *msg, int sb)
{
  if (is_motorola ()) {
    int bit = sb % 8,
        byte = msg->get_dlc () - 1 - sb / 8;
    start_bit = byte * 8 + bit;
  }
  else start_bit = sb;
} // CANdbSignal::set_start_bit_from_display


// Calculates the type of the signal, taking offset and factor into
// accound (a signal with integer representation can have real
// values e.g if the factor is 0.1221)
void CANdbSignal::update_scaled_type ()
{
  switch (get_type ()) {
    case CANDB_UNSIGNED:
    case CANDB_SIGNED:
         if ((floor (get_offset ()) != get_offset ()) ||
             (floor (get_factor ()) != get_factor ())) {
           scaled_type = CANDB_DOUBLE;
         }
         else {
           if ((get_type () == CANDB_UNSIGNED) &&
               ((get_factor () < 0.0) || (get_min_val () < 0.0))) {
             scaled_type = CANDB_SIGNED;
           }
           else {
             scaled_type = get_type ();
           }
         }
         break;

    case CANDB_FLOAT:
    case CANDB_DOUBLE:
         scaled_type = get_type ();
         break;

    case CANDB_INVALID:
         break;
  }
} // CANdbSignal::update_scaled_type


CANdb *CANdbSignal::get_candb (void) const
{
  if (message) return message->get_candb ();
  return NULL;
} // CANdbSignal::get_candb


bool CANdbSignal::is_signal_too_large (size_t len) const
{
  int sb = start_bit;
  // if motorola byte order, translate start bit as if it were intel byte order
  if (motorola)
    sb = start_bit - (2 * (start_bit / 8) - (int(len) - 1)) * 8;
  if (sb < 0)
    return true;
  return (int)(len * 8) < sb + length;
  } // CANdbSignal::has_valid_data_length

// ****************************************************************************


CANdbEnvVariable::CANdbEnvVariable ()
{
  name = NULL;
  comment = NULL;
  unit = NULL;
  candb = NULL;
  dummy_node = NULL;
  min_val = 0;
  max_val = 0;
  start_value = 0;
  type = CANDB_EV_INVALID;
  access = CANDB_EV_INVALID_ACCESS;
  data = 0;
  data_flag = false;
  num_id = 0;
  first_node = NULL;
  last_node = NULL;
  first_value = NULL;
  last_value = NULL;
  next = NULL;
} // CANdbEnvVariable::CANdbEnvVariable


CANdbEnvVariable::~CANdbEnvVariable ()
{
  type = CANDB_EV_INVALID;
  if (comment) delete [] comment;
  comment = NULL;
  if (name) delete [] name;
  name = NULL;
  if (unit) delete [] unit;
  unit = NULL;
  if (dummy_node) delete [] dummy_node;
  dummy_node = NULL;

  CANdbNodeEntry *node = first_node;
  while (node) {
    CANdbNodeEntry *nnode = node->get_next ();
    delete node;
    node = nnode;
  }
  first_node = NULL;
  last_node = NULL;

  CANdbEnumValue *v = first_value;
  while (v) {
    CANdbEnumValue *nv = v->get_next ();
    delete v;
    v = nv;
  }
  first_value = NULL;
  last_value = NULL;

  next = NULL;
} // CANdbEnvVariable::~CANdbEnvironmentVariable


void CANdbEnvVariable::add_receive_node (CANdbNode *node)
{
  if (!node) return;
  CANdbNodeEntry *ne = new CANdbNodeEntry (node);
  if (!first_node) first_node = last_node = ne;
  else {
    last_node->set_next (ne);
    last_node = ne;
  }
} // CANdbEnvVariable::add_receive_node


int CANdbEnvVariable::add_value (int val, const char *n)
{
  if (!n) return -1;
  if (get_value_string (val)) return -1;
  CANdbEnumValue *v = new CANdbEnumValue (val, n);
  if (!first_value) first_value = last_value = v;
  else {
    last_value->set_next (v);
    last_value = v;
  }
  return 0;
} // CANdbEnvVariable::add_value


const char* CANdbEnvVariable::get_value_string (int value) const
{
  CANdbEnumValue *v = first_value;
  while (v) {
    if (v->get_value () == value) return v->get_name ();
    v = v->get_next ();
  }
  return NULL;
} // CANdbEnvVariable::get_value_string


char* CANdbEnvVariable::get_qualified_name (char *buffer, int buflen) const
{
  if (name && buffer && (buflen > 0)) {
    strcpy (buffer, "");
    buflen -= (int) strlen (name);
    buflen -= 2; // for the dot & termination
    if (candb && candb->get_name () && ((int)strlen (candb->get_name ()) <= buflen)) {
      strcpy (buffer, candb->get_name ());
      buflen -= (int) strlen (candb->get_name ());
      strcat (buffer, ".");
      strcat (buffer, name);
    }
  }
  return buffer;
} // CANdbEnvVariable::get_qualified_name


int CANdbEnvVariable::insert_attribute (CANdbAttribute *attr)
{
  if (attr) {
    attributes.insert (attr);
    return 1;
  }
  else return 0;
} // CANdbEnvVariable::insert_attribute


// ****************************************************************************


CANdbMessage::CANdbMessage ()
{
  send_node = NULL;
  name = NULL;
  comment = NULL;
  id = 0;
  first_signal = NULL;
  last_signal = NULL;
  current_signal = NULL;
  signal_count = 0;
  dlc = 0;
  next = NULL;
  default_message_data = NULL;
  extended = false;
  candb = NULL;

  set_pgn_mask(0xFFFFFFFF);
  set_j1939Type(NOT_J1939);
} // CANdbMessage::CANdbMessage


CANdbMessage::~CANdbMessage ()
{
  if (default_message_data) default_message_data->set_message (NULL);
  default_message_data = NULL;

  if (comment) delete [] comment;
  comment = NULL;
  send_node = NULL;
  if (name) delete [] name;
  name = NULL;

  CANdbSignal *s = first_signal;
  while (s) {
    CANdbSignal *ns = s->get_next ();
    delete s;
    s = ns;
  }
  first_signal = current_signal = last_signal = NULL;
  if (DeletionCallback != NULL) {
    DeletionCallback(this);
  }

} // CANdbMessage::~CANdbMessage

void CANdbMessage::extend_memory_range (CANdbMemoryRange& mr) const
{
  mr.add_range ((void*)this, sizeof (CANdbMessage));
  if (name) mr.add_range (name, (int) strlen (name) + 1);
  if (comment) mr.add_range (comment, (int) strlen (comment) + 1);
  // if (send_node) send_node->extend_memory_range (mr);

  CANdbSignal *s = first_signal;
  while (s) {
    s->extend_memory_range (mr);
    s = s->get_next ();
  }
} // CANdbMessage::extend_memory_range

//NOTE if you edit this one, make sure get_qualified_name for signals still works..
char* CANdbMessage::get_qualified_name (char *buffer, int buflen) const
{
  if (name && buffer && (buflen > 0)) {
    strcpy (buffer, "");
    buflen -= (int) strlen (name);
    buflen -= 2; // for the dot & termination
    if (candb && candb->get_name () && ((int)strlen (candb->get_name ()) <= buflen)) {
      strcpy (buffer, candb->get_name ());
      buflen -= (int) strlen (candb->get_name ());
      strcat (buffer, ".");
      strcat (buffer, name);
    }
  }
  return buffer;
} // CANdbMessage::get_qualified_name


void CANdbMessage::set_name (const char *n)
{
  if (name) delete [] name;
  name = NULL;
  if (!n) return;
  name = new char [strlen (n) + 1];
  strcpy (name, n);
} // CANdbMessage::set_name


void CANdbMessage::set_send_node (CANdbNode *s)
{
  send_node = s;
} // CANdbMessage::set_send_node


void CANdbMessage::set_comment (const char *c)
{
  if (comment) delete [] comment;
  comment = NULL;
  if (!c) return;
  comment = new char [strlen (c) + 1];
  strcpy (comment, c);
} // CANdbMessage::set_comment


CANdbSignal *CANdbMessage::get_mode_signal (void) const
{
  CANdbSignal *s = first_signal;
  while (s) {
    if (s->is_mode_signal ()) return s;
    s = s->get_next ();
  }
  return NULL;
} // CANdbMessage::get_mode_signal


CANdbSignal *CANdbMessage::find_signal_by_name (const char *name) const
{
  CANdbSignal *s = first_signal;
  while (s) {
    if (!strcmp (s->get_name (), name)) return s;
    s = s->get_next ();
  }
  return NULL;
} // CANdbMessage::find_signal_by_name


int CANdbMessage::insert_signal (CANdbSignal *signal)
{
  signal->set_next (NULL);
  signal->set_message (this);
  if (!first_signal || !last_signal) first_signal = last_signal = signal;
  else {
    last_signal->set_next (signal);
    last_signal = signal;
  }
  signal_count++;
  return 0;
} // CANdbMessage::insert_signal


int CANdbMessage::remove_signal (CANdbSignal *signal)
{
  if (signal->get_message () == this) signal->set_message (NULL);

  CANdbSignal *ps = NULL,
              *s = first_signal;
  while (s) {
    if (s == signal) {
      if (ps) {
        ps->set_next (s->get_next ());
        if (last_signal == s) last_signal = ps;
      }
      else {
        first_signal = s->get_next ();
        if (!first_signal) last_signal = NULL;
      }
      s->set_next (NULL);
      --signal_count;
      return 0;
    }
    ps = s;
    s = s->get_next ();
  }
  return -1;
} // CANdbMessage::remove_signal


int CANdbMessage::delete_signal (CANdbSignal *signal)
{
  if (remove_signal (signal) == 0) {
    if (current_signal == signal) current_signal = current_signal->get_next ();
    delete signal;
    return 0;
  }
  return -1;
} // CANdbMessage::delete_signal


CANdbSignal *CANdbMessage::get_first_signal (void)
{
  current_signal = first_signal;
  return current_signal;
} // CANdbMessage::get_first_signal


CANdbSignal *CANdbMessage::get_next_signal (void)
{
  if (current_signal) current_signal = current_signal->get_next ();
  return current_signal;
} // CANdbMessage::get_next_signal

void CANdbMessage::update_pgn_mask(void)
{
  uint32_t pdu_fmt;
  if (get_j1939Type() == J1939_V1) {
    pdu_fmt = (id & 0x0000ff00) >> 8;
  }
  else {
    pdu_fmt = (id & 0x00ff0000) >> 16;
  }

  if (pdu_fmt < 240){
    set_pgn_mask(0x03ff0000 | EXT_FLAG);  // PDU1
  }
  else {
    set_pgn_mask(0x03ffff00 | EXT_FLAG);  // PDU2
  }
}

void CANdbMessage::setup (void)
{
  CANdbSignal *signal = get_first_signal ();
  while (signal) {
    const CANdbAttributeList *al = signal->get_attributes ();
    CANdbSignal *new_sign_signal = NULL;
    if (al && signal->get_name ()) {
      CANdbAttribute *attr = al->find_by_name ("VorzeichenSignal");
      if (!attr) attr = al->find_by_name ("SignSignal");
      if (attr &&
          attr->get_string_value () &&
          (attr->get_type () == CANDB_ATTR_TYPE_STRING)) {

        //printf ("Sgn found: msg=%s, sig=%s, aval=%s\n",
        //        get_name (),
        //        signal->get_name (),
        //        attr->get_string_value ());

        new_sign_signal = get_first_signal ();
        while (new_sign_signal) {
          if (new_sign_signal &&
              new_sign_signal->get_name () &&
              (strcmp (new_sign_signal->get_name (),
                       attr->get_string_value ()) == 0)) break;
          new_sign_signal = new_sign_signal->get_next ();
        }
      }
    }
    // if (new_sign_signal) printf ("  -> Sign=%s\n", new_sign_signal->get_name ());
    signal->set_sign_signal (new_sign_signal);
    if (new_sign_signal) {
      if (signal->get_type () == CANDB_UNSIGNED) {
        signal->set_type (CANDB_SIGNED);
        if (signal->get_min_val () == 0) signal->set_min_val (- signal->get_max_val ());
      }
    }
    signal = signal->get_next ();
  }

  //set pgn-mask and j1939
  set_j1939Type(NOT_J1939);

  if (get_candb()->get_j1939Type() == CANdb::J1939_V1) { //j1939 v1 database
    set_j1939Type(J1939_V1);
    update_pgn_mask();

  } else  { //j1939 v2 database or a non-j1939 database
    const CANdbAttributeDefinition *ad = get_candb()->find_attribute_definition_by_name(Attribute_VFrameFormat);
    if (ad && ad->get_type() == CANDB_ATTR_TYPE_ENUMERATION) {
      const CANdbAttribute     *attr;
      const CANdbAttributeList *aList = get_attributes();

      if (aList) {
        attr = aList->find_by_name(Attribute_VFrameFormat); //get attribute VFrameFormat
        if (attr && attr->get_type () == CANDB_ATTR_TYPE_ENUMERATION) {
          if (attr->get_enumeration_value () == VFrameFormat_J1939) {
            update_pgn_mask();
            set_j1939Type(J1939_V2);
            set_extended(true);
          } else if (attr->get_enumeration_value() == VFrameFormat_ExtendedCAN_FD || attr->get_enumeration_value() == VFrameFormat_ExtendedCAN) {
            set_extended(true);
          }
        } else { //no attribute found. try the database default value for VFrameFormat
          if (ad->get_enumeration_default() == VFrameFormat_J1939) {
            update_pgn_mask();
            set_j1939Type(J1939_V2);
            set_extended(true);
          }
        }
      }
    } else if (get_candb()->get_j1939Type() == CANdb::J1939_V2) {
      if (is_extended()) {
        update_pgn_mask();
        set_j1939Type(J1939_V2);
      }
    }
  }
} // CANdbMessage::setup

bool CANdbMessage::is_canfd (void) const
{
  const CANdbAttributeList *al = get_const_attributes();

  if (al) {
    const CANdbAttribute *a = al->find_by_name (Attribute_VFrameFormat);

    if (a) { // Get attribute value
      if (a->get_type () == CANDB_ATTR_TYPE_ENUMERATION) {
        if (a->get_enumeration_value() == VFrameFormat_StandardCAN_FD ||
            a->get_enumeration_value() == VFrameFormat_ExtendedCAN_FD) {
          return true;
        }
      }
    }
    else { // If attribute value not found try to get the default value
      const CANdbAttributeDefinition *ad = get_candb()->find_attribute_definition_by_name(Attribute_VFrameFormat);
      if (ad && ad->get_type() == CANDB_ATTR_TYPE_ENUMERATION) {
        if (ad->get_enumeration_default() == VFrameFormat_StandardCAN_FD ||
            ad->get_enumeration_default() == VFrameFormat_ExtendedCAN_FD) {
          return true;
        }
      }
    }
  }

  return false;
} // CANdbMessage::is_canfd

bool CANdbMessage::is_brs (void) const
{
  const char* DbcCanfdBrs = "CANFD_BRS";

  const CANdbAttributeList *al = get_const_attributes();

  if (al) {
    const CANdbAttribute *a = al->find_by_name (DbcCanfdBrs);

    if (a) { // Get attribute value
      if (a->get_type() == CANDB_ATTR_TYPE_ENUMERATION) {
        if (a->get_enumeration_value() == 1) {
          return true;
        }
      }
    }
    else { // If attribute value not found try to get the default value
      const CANdbAttributeDefinition *ad = get_candb()->find_attribute_definition_by_name(DbcCanfdBrs);
      if (ad && ad->get_type() == CANDB_ATTR_TYPE_ENUMERATION) {
        if (ad->get_enumeration_default() == 1) {
          return true;
        }
      }
    }
  }

  return false;
} // CANdbMessage::is_brs

// ****************************************************************************


CANdbNode::CANdbNode ()
{
  name = NULL;
  comment = NULL;
  next = NULL;
  master = false;
} // CANdbNode::CANdbNode


CANdbNode::~CANdbNode ()
{
  set_name (NULL);
  set_comment (NULL);
  if (DeletionCallback != NULL) {
    DeletionCallback(this);
  }

} // CANdbNode::~CANdbNode


void CANdbNode::set_name (const char *n)
{
  if (name) delete [] name;
  name = NULL;
  if (n) {
    name = new char [strlen (n) + 1];
    strcpy (name, n);
  }
} // CANdbNode::set_name


void CANdbNode::set_comment (const char *c)
{
  if (comment) delete [] comment;
  comment = NULL;
  if (c) {
    comment = new char [strlen (c) + 1];
    strcpy (comment, c);
  }
} // CANdbNode::set_comment


int CANdbNode::insert_attribute (CANdbAttribute *attr)
{
  if (attr) {
    attributes.insert(attr);
    return 1;
  }
  else return 0;
} // CANdbMessage::insert_attribute


// ****************************************************************************


CANdbNodeEntry::CANdbNodeEntry (CANdbNode *n)
{
  node = n;
  next = NULL;
} // CANdbNodeEntry::CANdbNodeEntry


CANdbNodeEntry::~CANdbNodeEntry ()
{
  node = NULL;
  next = NULL;
} // CANdbNodeEntry::~CANdbNodeEntry


// ****************************************************************************


CANdb::CANdb ()
{
  name = NULL;
  comment = NULL;
  filename = NULL;
  message_count = 0;
  node_count = 0;
  hash_table_size = 2048;
  hash_table = new HashEntry [hash_table_size];
  first_message = NULL;
  first_schedule_table = NULL;
  last_schedule_table = NULL;
  schedule_table_count = 0;
  first_signal_encoding = NULL;
  last_signal_encoding = NULL;
  signal_encoding_count = 0;
  last_message = NULL;
  current_message = NULL;
  first_env_variable = NULL;
  last_env_variable = NULL;
  first_attr_def = NULL;
  last_attr_def = NULL;
  first_node = NULL;
  last_node = NULL;
  current_node = NULL;
  next = NULL;
  set_j1939Type(NOT_J1939);
} // CANdb::CANdb


CANdb::~CANdb ()
{
  clear_hash_table ();
  if (hash_table) delete [] hash_table;
  hash_table = NULL;

  if (comment) delete [] comment;
  comment = NULL;

  if (name) delete [] name;
  name = NULL;

  if (filename) delete [] filename;
  filename = NULL;

  // schedule tables
  CANdbScheduleTable *st = first_schedule_table;
  while (st) {
    CANdbScheduleTable *nst = st->get_next ();
    delete st;
    st = nst;
  }
  first_schedule_table = last_schedule_table = NULL;
  schedule_table_count = 0;

  // signal encoding
  CANdbSignalEncoding *se = first_signal_encoding;
  while (se) {
    CANdbSignalEncoding *nse = se->get_next ();
    delete se;
    se = nse;
  }
  first_signal_encoding = last_signal_encoding = NULL;
  signal_encoding_count = 0;

  CANdbMessage *m = first_message;
  while (m) {
    CANdbMessage *nm = m->get_next ();
    delete m;
    m = nm;
  }
  first_message = last_message = current_message = NULL;

  CANdbEnvVariable *var = first_env_variable;
  while (var) {
    CANdbEnvVariable *nextVar = var->get_next ();
    delete var;
    var = nextVar;
  }

  // Delete attribute list
  CANdbAttributeDefinition *ad = first_attr_def;
  while (ad) {
    CANdbAttributeDefinition *nad = ad->get_next ();
    delete ad;
    ad = nad;
  }
  first_attr_def = last_attr_def = NULL;

  CANdbNode *n = first_node;
  while (n) {
    CANdbNode *nn = n->get_next ();
    delete n;
    n = nn;
  }
  first_node = last_node = current_node = NULL;
} // CANdb::~CANdb


void CANdb::clear_hash_table ()
{
  for (unsigned int i = 0; i < hash_table_size; ++i) {
    HashEntry *h = hash_table [i] .next;
    while (h) {
      HashEntry *nh = h->next;
      delete h;
      h = nh;
    }
    hash_table [i] .next = NULL;
    hash_table [i] .message = NULL;
  }
} // CANdb::clear_hash_table


void CANdb::rebuild_hash_table ()
{
  CANdbMessage *message = first_message;
  while (message) {
    HashEntry *h = &hash_table [message->get_id_and_ext () % hash_table_size];
    if (!h->message) h->message = message;
    else {
      HashEntry *nh = new HashEntry;
      nh->message = message;
      nh->next = h->next;
      h->next = nh;
    }
    message = message->get_next ();
  }
} // CANdb::rebuild_hash_table


void CANdb::update_j1939_flag ()
{
  const CANdbAttribute           *attr;
  const CANdbAttributeList       *aList = get_attributes();
  const CANdbAttributeDefinition *ad;

  set_j1939Type(NOT_J1939);

  if (aList) {
    attr = aList->find_by_name ("J1939PGN");
    if (attr && attr->get_type () == CANDB_ATTR_TYPE_INTEGER) {
      if (attr->get_integer_value () == 1) {
	set_j1939Type(J1939_V1);
	return;
      }
    } else {
      ad = find_attribute_definition_by_name("J1939PGN");
      if (ad && ad->get_type () == CANDB_ATTR_TYPE_INTEGER) {
	if (ad->get_integer_default () == 1) {
	  set_j1939Type(J1939_V1);
	  return;
	}
      }
    }

    attr = aList->find_by_name ("ProtocolType");
    if (attr && attr->get_type () == CANDB_ATTR_TYPE_STRING) {
      const char *attr_val = attr->get_string_value ();
      if (strcmp (attr_val, "J1939") == 0) {
	set_j1939Type(J1939_V2);
	return;
      }
    } else {
      ad = find_attribute_definition_by_name("ProtocolType");
      if (ad && ad->get_type () == CANDB_ATTR_TYPE_STRING) {
	const char *attr_val = ad->get_string_default ();
	if (strcmp (attr_val, "J1939") == 0) {
	  set_j1939Type(J1939_V2);
	  return;
	}
      }
    }
  }
} // CANdb::update_j1939_flag


// EXT_FLAG set in the id means extended CAN
int CANdb::handle_can_msg (unsigned long id, unsigned char* can_data, int dlc, unsigned int arg) const
{
  //CANdbMessage *message = find_message_by_id (id);
  //if (!message) return -1;
  // Don't use find_message_by_id() to search for the ID. Search the hash table
  // by our own here in order to find all message IDs.

#ifndef CANDB_NO_J1939_DECODING
  if (j1939 && (id & EXT_FLAG)) { // decode the 8-byte J1939 messages
    unsigned long pf;
    pf = (id & 0x0ff0000) >> 16;
    if (pf < 0xF0) id &= 0x1ff0000; // PDU1 format
              else id &= 0x1ffff00; // PDU2 format
    id >>= 8;
    id |= EXT_FLAG;
  }
#endif

  HashEntry *h = &hash_table [id % hash_table_size];
  while (h) {
    if (h->message && (h->message->get_id_and_ext () == id)) {
      CANdbMessage *message = h->message;
      CANdbSignal *mode_signal = NULL;
      mode_signal = message->get_mode_signal ();
      int current_mode = -1;
      int mode_signal_res = 0;
      if (mode_signal) {
        uint64_t uvalue = 0;
        mode_signal_res = mode_signal->get_value_uint (can_data, dlc, uvalue);
        current_mode = (int) uvalue;
      }

      CANdbSignal *signal = message->get_first_signal ();
      while (signal) {
        if (!mode_signal ||
            !signal->is_mode_dependent () ||
           ((signal->get_mode () == current_mode) && (mode_signal_res == 0))) {
          signal->handle_can_msg (can_data, dlc, arg);
        }

        signal = signal->get_next ();
      }
    }
    h = h->next;
  }
  return 0;
} // CANdb::handle_can_msg


CANdbMessage *CANdb::get_first_message ()
{
  current_message = first_message;
  return current_message;
} // CANdb::get_first_message


CANdbMessage *CANdb::get_next_message ()
{
  if (current_message) current_message = current_message->get_next ();
  return current_message;
} // CANdb::get_next_message


CANdbNode *CANdb::get_first_node ()
{
  current_node = first_node;
  return current_node;
} // CANdb::get_first_node


CANdbNode *CANdb::get_next_node ()
{
  if (current_node) current_node = current_node->get_next ();
  return current_node;
} // CANdb::get_next_node


void CANdb::set_name (const char *n)
{
  if (name) delete [] name;
  name = NULL;
  if (!n) return;
  name = new char [strlen (n) + 1];
  strcpy (name, n);
} // CANdb::set_name


void CANdb::set_comment (const char *c)
{
  if (comment) delete [] comment;
  comment = NULL;
  if (!c) return;
  comment = new char [strlen (c) + 1];
  strcpy (comment, c);
} // CANdb::set_comment


void CANdb::set_filename (const char *fn)
{
  if (filename) delete [] filename;
  filename = NULL;
  if (!fn) return;
  filename = new char [strlen (fn) + 1];
  strcpy (filename, fn);
} // CANdb::set_filename


// Inserts a new node. Returns 0 if success.
int CANdb::insert_node (CANdbNode *node)
{
  if (!node) return -1;
  if (!first_node) first_node = last_node = node;
  else {
    last_node->set_next (node);
    last_node = node;
  }
  node->set_candb (this);
  ++node_count;
  return 0;
} // CANdb::insert_node


// Removes a node. Returns 0 if success.
// All messages and signals where sender refers to the node gets sender cleared.
int CANdb::remove_node (CANdbNode *node)
{
  if (!node) return -1;

  // Remove all references to it from the messages.
  CANdbMessage *m = first_message;
  while (m) {
    CANdbSignal *s = m->get_first_signal ();
    while (s) {
      s->remove_receive_node (node);
      s = s->get_next ();
    }
    if (m->get_send_node () == node) m->set_send_node (NULL);
    m = m->get_next ();
  }

  // Unlink the node from the list
  CANdbNode *n = first_node,
            *pn = NULL;
  while (n) {
    if (n == node) {
      if (pn) {
        pn->set_next (n->get_next ());
        if (n == last_node) last_node = pn;
      }
      else {
        first_node = n->get_next ();
        if (n == last_node) last_node = NULL;
      }
      node_count--;
      return 0;
    }
    pn = n;
    n = n->get_next ();
  }

  return -1;
} // CANdb::remove_node


CANdbNode* CANdb::find_node_by_name (const char *name) const
{
  CANdbNode *node = first_node;

  while (node) {
    if (node->get_name ()) {
      if (!strcmp (node->get_name (), name)) {
	return node;
      }
    }
    node = node->get_next ();
  }
  return NULL;
} // CANdb::find_node_by_name


CANdbMessage* CANdb::find_message_by_name (const char *n) const
{
  CANdbMessage *msg = first_message;
  while (msg) {
    if (msg->get_name ()) {
      if (!strcmp (msg->get_name (), n)) {
	return msg;
      }
    }
    msg = msg->get_next ();
  }
  return NULL;
} // CANdb::find_message_by_name


// EXT_FLAG is set in the id for extended CAN
CANdbMessage* CANdb::find_message_by_id (unsigned int id) const
{
  HashEntry *h = &hash_table [id % hash_table_size];
  while (h) {
    if (h->message &&
        (h->message->get_id_and_ext () == id)) {
      return h->message;
    }
    h = h->next;
  }
  return NULL;
} // CANdb::find_message_by_id

CANdbMessage* CANdb::find_message_by_pgn (unsigned int id_and_ext)
{
  unsigned int  mask;
  unsigned int pgn;
  CANdbMessage *msg = first_message;
  while (msg) {

    if (msg->get_j1939Type() == CANdbMessage::J1939_V1){

      mask = msg->get_pgn_mask();

      // CAN Id in a V1 database is actually PGN
      pgn = (id_and_ext & (mask & ~EXT_FLAG)) >> 8;

      // Only pgn in dbc V1 will have EXT bit set, CAN messages will not.
      if (id_and_ext & EXT_FLAG) {
        pgn |= EXT_FLAG;
      }

      if (pgn == msg->get_id_and_ext()) {
        if ((msg->get_id_and_ext() != 0xC0000000)) { // EXT | VECTOR__INDEPENDENT_SIG_MSG
          return msg;
        }
      }
    }
    else if (msg->get_j1939Type() == CANdbMessage::J1939_V2) {
      mask = msg->get_pgn_mask();
      if ((msg->get_id_and_ext() & mask) == (id_and_ext & mask)) {
        if ((msg->get_id_and_ext() != 0xC0000000)) { // EXT | VECTOR__INDEPENDENT_SIG_MSG
          return msg;
        }
      }
    }
    msg = msg->get_next ();
  }
  return NULL;
} // CANdb::find_message_by_pgn

bool CANdb::contains_message (CANdbMessage *message) const
{
  CANdbMessage *msg = first_message;
  while (msg) {
    if (msg == message) return true;
    msg = msg->get_next ();
  }
  return false;
} // CANdb::contains_message


int CANdb::insert_attribute_definition (CANdbAttributeDefinition *attr_def)
{
  if (!first_attr_def) first_attr_def = last_attr_def = attr_def;
  else {
    last_attr_def->set_next (attr_def);
    last_attr_def = attr_def;
  }
  return 0;
} // CANdb::insert_attribute_definition

int CANdb::delete_attribute_definition (CANdbAttributeDefinition *attr_def)
{
  CANdbAttributeDefinition *a      = first_attr_def;
  CANdbAttributeDefinition *prev_a = first_attr_def;

  while (a) {
    if (a == attr_def) {
      if (a == first_attr_def) {
	first_attr_def = a->get_next ();
      } else if (a == last_attr_def) {
	last_attr_def = prev_a;
	prev_a->set_next(NULL);
      } else {
	prev_a->set_next(a->get_next ());
      }
      delete a;
      return 0;
    } else {
      prev_a = a;
      a = a->get_next ();
    }
  }
  return -1;
} // CANdb::delete_attribute_definition


CANdbAttributeDefinition *CANdb::find_attribute_definition_by_name (const char *n)
{
  CANdbAttributeDefinition *a = first_attr_def;
  while (a) {
    if (strcmp (n, a->get_name ()) == 0) break;
    a = a->get_next ();
  }
  return a;
} // CANdb::find_attribute_definition_by_name


int CANdb::insert_message (CANdbMessage *msg)
{
  if (!msg) return -1;
  HashEntry *h = &hash_table [msg->get_id_and_ext () % hash_table_size];
  if (!h->message) h->message = msg;
  else {
    HashEntry *nh = new HashEntry;
    nh->message = msg;
    nh->next = h->next;
    h->next = nh;
  }

  msg->set_next (NULL);
  if (!first_message) first_message = last_message = msg;
  else {
    last_message->set_next (msg);
    last_message = msg;
  }

  msg->set_candb (this);
  ++message_count;
  return 0;
} // CANdb::insert_message


int CANdb::delete_message (CANdbMessage *msg)
{
  if (remove_message (msg) == 0) {
    delete msg;
    return 0;
  }
  return -1;
} // CANdb::delete_message


int CANdb::remove_message (CANdbMessage *msg)
{
  if (msg->get_candb () == this) msg->set_candb (NULL);

  for (unsigned int i = 0; i < hash_table_size; ++i) {
    HashEntry *h = &hash_table [i],
              *ph = NULL;
    bool found = false;
    while (h) {
      if (h->message == msg) {
        h->message = NULL;
        if (ph) {
          ph->next = h->next;
          delete h;
        }
        found = true;
        break;
      }
      ph = h;
      h = h->next;
    }
    if (found) break;
  }

  // delete from schedule tables ....
  CANdbScheduleTable *st = first_schedule_table;
  while (st) {
    st->remove_message (msg);
    st = st->get_next ();
  }

  CANdbMessage *m = first_message,
               *pm = NULL;
  while (m) {
    if (m == msg) {
      if (pm) {
        pm->set_next (m->get_next ());
        if (m == last_message) last_message = pm;
      }
      else {
        first_message = m->get_next ();
        if (m == last_message) last_message = NULL;
      }
      message_count--;
      return 0;
    }
    pm = m;
    m = m->get_next ();
  }

  if (m == current_message) current_message = NULL;

  return -1;
} // CANdb::remove_message


int CANdb::insert_env_variable (CANdbEnvVariable *var)
{
  if (!var)  return -1;

  var->set_candb (this);

  if (!first_env_variable) {
    first_env_variable = last_env_variable = var;
  }
  else {
    last_env_variable->set_next (var);
    last_env_variable = var;
  }
  return 0;
} // CANdb::insert_env_variable


CANdbEnvVariable* CANdb::find_env_variable_by_name (const char *name) const
{
  CANdbEnvVariable *v = first_env_variable;
  while (v) {
    if (strcmp (v->get_name (), name) == 0) break;
    v = v->get_next ();
  }
  return v;
} // CANdb::find_node_by_name


void CANdb::insert_schedule_table (CANdbScheduleTable *st)
{
  st->set_next (NULL);
  if (first_schedule_table && last_schedule_table) {
    last_schedule_table->set_next (st);
    last_schedule_table = st;
    ++schedule_table_count;
  }
  else {
    first_schedule_table = last_schedule_table = st;
    schedule_table_count = 1;
  }
} // CANdb::insert_schedule_table


CANdbScheduleTable *CANdb::find_schedule_table_by_name (const char *name) const
{
  CANdbScheduleTable *st = first_schedule_table;
  while (st) {
    if (strcmp (st->get_name (), name) == 0) break;
    st = st->get_next ();
  }
  return st;
} // CANdb::find_schedule_table_by_name


void CANdb::insert_signal_encoding (CANdbSignalEncoding *se)
{
  se->set_next (NULL);
  if (first_signal_encoding && last_signal_encoding) {
    last_signal_encoding->set_next (se);
    last_signal_encoding = se;
    ++signal_encoding_count;
  }
  else {
    first_signal_encoding = last_signal_encoding = se;
    signal_encoding_count = 1;
  }
} // CANdb::insert_signal_encoding


CANdbSignalEncoding *CANdb::find_signal_encoding_by_name (const char *name) const
{
  CANdbSignalEncoding *se = first_signal_encoding;
  while (se) {
    if (strcmp (se->get_name (), name) == 0) break;
    se = se->get_next ();
  }
  return se;
} // CANdb::find_signal_encoding_by_name


// ****************************************************************************


CANdbCluster::CANdbCluster ()
{
  first_db = NULL;
  last_db = NULL;
  current_db = NULL;
  current_db_node = NULL;
  //default_name = NULL;
  candb_count = 0;
} // CANdbCluster::CANdbCluster


CANdbCluster::~CANdbCluster ()
{
  CANdb *db =  first_db;
  while (db) {
    CANdb *ndb = db->get_next ();
    delete db;
    db = ndb;
  }
  first_db = last_db = current_db = current_db_node = NULL;

  //if (default_name) delete [] default_name;
  //default_name = NULL;

  candb_count = 0;
  if (DeletionCallback != NULL) {
    DeletionCallback(this);
  }

} // CANdbCluster::~CANdbCluster


//void CANdbCluster::set_default_name (const char *n)
//{
//  if (default_name) delete [] default_name;
//  default_name = NULL;
//  if (n) {
//    default_name = new char [strlen (n) + 1];
//    if (default_name) strcpy (default_name, n);
//  }
//} // CANdbCluster::set_default_name


void CANdbCluster::add_db (CANdb *db)
{
  db->set_next (NULL);
  if (first_db && last_db) {
    last_db->set_next (db);
    last_db = db;
  }
  else first_db = last_db = db;

  candb_count += 1;
} // CANdbCluster::add_db


bool CANdbCluster::remove_db (CANdb *rdb)
{
  if (rdb && (current_db == rdb)) current_db = current_db->get_next ();
  if (rdb && (current_db_node == rdb)) current_db_node = current_db_node->get_next ();
  CANdb *db = first_db,
        *pdb = NULL;
  while (db) {
    if (rdb == db) { // found CANdb entry
      if (pdb) pdb->set_next (db->get_next ());
          else first_db = db->get_next ();
      if (db == last_db) last_db = pdb;
      db->set_next (NULL);
      candb_count -= 1;
      return true;
    }
    pdb = db;
    db = db->get_next ();
  }
  return false;
} // CANdbCluster::remove_db


bool CANdbCluster::delete_db (CANdb *db)
{
  bool ret = remove_db (db);
  if (ret) delete db;
  return ret;
} // CANdbCluster::delete_db


CANdb *CANdbCluster::get_first_candb ()
{
  current_db = first_db;
  return current_db;
} // CANdbCluster::get_first_candb


CANdb *CANdbCluster::get_next_candb ()
{
  if (current_db) current_db = current_db->get_next ();
  return current_db;
} // CANdbCluster::get_next_candb


CANdb *CANdbCluster::get_current_candb ()
{
  return current_db;
} // CANdbCluster::get_current_candb


CANdbNode *CANdbCluster::get_first_node ()
{
  CANdbNode *n = NULL;
  current_db_node = first_db;
  while (!n && current_db_node) {
    n = current_db_node->get_first_node ();
    if (!n) current_db_node = current_db_node->get_next ();
  }
  return n;
} // CANdbCluster::get_first_node


CANdbNode *CANdbCluster::get_next_node ()
{
  if (!current_db_node) return NULL;
  CANdbNode *n = current_db_node->get_next_node ();
  while (!n && current_db_node) {
    current_db_node = current_db_node->get_next ();
    if (current_db_node) n = current_db_node->get_first_node ();
  }
  return n;
} // CANdbCluster::get_next_node


CANdbNode* CANdbCluster::find_node_by_name (const char *name) const
{
  CANdbNode *n = NULL;
  CANdb *db = first_db;
  while (!n && db) {
    n = db->find_node_by_name (name);
    if (!n) db = db->get_next ();
  }
  return n;
} // CANdbCluster::find_node_by_name


CANdbMessage *CANdbCluster::get_first_message ()
{
  CANdbMessage *m = NULL;
  current_db = first_db;
  while (!m && current_db) {
    m = current_db->get_first_message ();
    if (!m) current_db = current_db->get_next ();
  }
  return m;
} // CANdbCluster::get_first_message


CANdbMessage *CANdbCluster::get_next_message ()
{
  if (!current_db) return NULL;
  CANdbMessage *m = current_db->get_next_message ();
  while (!m && current_db) {
    current_db = current_db->get_next ();
    if (current_db) m = current_db->get_first_message ();
  }
  return m;
} // CANdbCluster::get_next_message


CANdbMessage* CANdbCluster::find_message_by_name (const char *name) const
{
  CANdbMessage *m = NULL;
  CANdb *db = first_db;
  while (!m && db) {
    m = db->find_message_by_name (name);
    if (!m) db = db->get_next ();
  }
  return m;
} // CANdbCluster::find_message_by_name


CANdbMessage* CANdbCluster::find_message_by_id (unsigned int id) const
{
  CANdbMessage *m = NULL;
  CANdb *db = first_db;
  while (!m && db) {
    m = db->find_message_by_id (id);
    if (!m) db = db->get_next ();
  }
  return m;
} // CANdbCluster::find_message_by_id

CANdbMessage* CANdbCluster::find_message_by_pgn (unsigned int id)
{
  CANdbMessage *m = NULL;
  CANdb *db = first_db;
  while (!m && db) {
    m = db->find_message_by_pgn (id);
    if (!m) db = db->get_next ();
  }
  return m;
} // CANdbCluster::find_message_by_pgn

CANdb* CANdbCluster::find_candb_by_message (CANdbMessage *message) const
{
  CANdb *db = first_db;
  while (db) {
    if (db->contains_message (message)) break;
    db = db->get_next ();
  }
  return db;
} // CANdbCluster::find_candb_by_message


bool CANdbCluster::is_j1939_message (const char *message_name) const
{
  CANdbMessage *message = find_message_by_name (message_name);
  if (message) return (message->is_j1939 ());
  return false;
} // CANdbCluster::is_j1939_message


int CANdbCluster::handle_can_msg (unsigned long id,
                                  unsigned char* can_data,
                                  int dlc,
                                  unsigned int arg) const
{
  CANdb *db = first_db;
  while (db) {
    db->handle_can_msg (id, can_data, dlc, arg);
    db = db->get_next ();
  }
  return 0;
} // CANdbCluster::handle_can_msg


// ****************************************************************************
// Static CANdb functions:

char *CANdb::get_short_comment (const char *comment, unsigned int max_len, char *buffer)
{
  if (!comment || !buffer || (max_len < 4)) return NULL;
  while ((*comment == ' ') ||
         (*comment == '\t') ||
         (*comment == '\n') ||
         (*comment == '\r')) ++comment;
  strncpy (buffer, comment, max_len - 1);
  buffer[max_len - 1] = '\0';
  char *nl = strchr (buffer, '\n');
  if (nl) *nl = '\0';
  unsigned int l = (int) strlen (buffer);
  if (l < strlen (comment)) {
    if ((l + 3) >= max_len) l = max_len - 4;
    buffer [l] = '\0';
    strcat (buffer, "...");
  }
  return buffer;
} // CANdb::get_short_comment



static int candb_last_variable = 1;

void CANdb::extend_memory_range (CANdbMemoryRange& mr) const
{
  mr.add_range ((void*) &candb_first_variable, (void*) &candb_last_variable);
  mr.add_range ((void*)this, sizeof (CANdb));
  if (name) mr.add_range (name, (int) strlen (name) + 1);
  if (comment) mr.add_range (comment, (int) strlen (comment) + 1);
  if (hash_table) mr.add_range (hash_table, sizeof (HashEntry) * hash_table_size);

  for (unsigned int i = 0; i < hash_table_size; ++i) {
    HashEntry *h = &hash_table [i];
    while (h) {
      if (h->message) h->message->extend_memory_range (mr);
      h = h->next;
      if (h) mr.add_range (h, sizeof (HashEntry));
    }
  }
} // CANdb::extend_memory_range


// ****************************************************************************


CANdbFileIo::CANdbFileIo (const char *fn)
{
  errorlog = NULL;
  read_ok = false;
  filename = NULL;
  set_filename (fn);
  lineno = 1;
  message_logger = NULL;
} // CANdbFileIo::CANdbFileIo


CANdbFileIo::~CANdbFileIo ()
{
  set_errorlog(NULL);
  set_filename (NULL);
} // CANdbFileIo::~CANdbFileIo


void CANdbFileIo::set_filename (const char *fn)
{
  if (filename) delete [] filename;
  filename = NULL;
  if (!fn) return;
  filename = new char [strlen (fn) + 1];
  strcpy (filename, fn);
} // CANdbFileIo::set_filename


void CANdbFileIo::set_file_format (CANdbFileFormat *ff)
{
  file_format = ff;
} // CANdbFileIo::set_file_format


const char *CANdbFileIo::get_db_name () const
{
#define MAX_DB_NAME     100

  static char name [MAX_DB_NAME + 1];

  if (filename) {
    const char *fn = filename,
               *start = fn;
    while (*fn) {
      if ((*fn == '\\') || (*fn == '/')) start = fn + 1;
      ++fn;
    }
    int len = 0;
    const char *x = start;
    while (*x && (len < MAX_DB_NAME)) {
      if (*x == '.') break;
      ++x;
      ++len;
    }
    strncpy (name, start, len);
    name [len] = '\0';
  }
  else strcpy (name, "unkown");

  return name;
} // CANdbFileIo::get_db_name


void CANdbFileIo::set_dbname (CANdb *db)
{
  //if (get_filename ()) {
  //  char tmpname [MAX_PATH + 1];
  //  strncpy (tmpname, get_filename (), MAX_PATH);
  //  tmpname [MAX_PATH] = '\0';
  //
  //  char *x = tmpname,
  //       *cname = tmpname;
  //  while (*x) {
  //    if ((*x == '\\') || (*x == '/')) cname = x + 1;
  //    ++x;
  //  }
  //
  //  char *ext = strstr (cname, ".dbc");
  //  if (!ext) ext = strstr (cname, ".DBC");
  //  if (!ext) ext = strstr (cname, ".");
  //  if (ext) *ext = '\0';
  //
  //  db->set_name (cname);
  //}
  //else db->set_name ("unknown");
  db->set_name (get_db_name ());
} // CANdbFileIo::set_dbname

void CANdbFileIo::get_errorlog(char *msg, unsigned int *buflen)
{
  if (!errorlog){
    set_errorlog("");
  }

  // IF buflen too short returns the first errors. The last would be better.
  unsigned int errlen = (unsigned int) strlen(errorlog);
  if (errlen > *buflen - 1){
    strncpy(msg, errorlog, *buflen - 1);
    *buflen = errlen + 1;
  } else {
    strcpy(msg, errorlog);
  }
}

void CANdbFileIo::set_errorlog(const char *msg)
{
  if (errorlog) delete [] errorlog;
  errorlog = NULL;
  if (!msg) return;
  errorlog = new char[strlen(msg) + 1];
  strcpy(errorlog, msg);
}

void CANdbFileIo::append_errorlog(const char *msg)
{
  if(!errorlog){
    set_errorlog(msg);
    return;
  }

  char* newlog = new char[strlen(errorlog) + strlen(msg) + 1];
  strcpy(newlog, errorlog);
  strcat(newlog, msg);

  delete [] errorlog;
  errorlog = newlog;
}

void CANdbFileIo::error (const char *fmt, ...)
{
  char tmp [MAX_MSG + 1];
  va_list ap;
  va_start (ap, fmt);
  vsnprintf (tmp, MAX_MSG, fmt, ap);
  va_end (ap);

  char new_message [MAX_MSG + 31];
  sprintf (new_message, "Error (line %d): %s\n", lineno, tmp);

  // if (message_logger) message_logger->error (lineno, filename, tmp);
  // else printf ("Error (line %d): %s\n", lineno, tmp);

  append_errorlog(new_message);

} // CANdbFileIo::error


void CANdbFileIo::warning (const char *fmt, ...)
{
  char tmp [MAX_MSG + 1];
  va_list ap;
  va_start (ap, fmt);
  vsnprintf (tmp, MAX_MSG, fmt, ap);
  va_end (ap);
  if (message_logger) message_logger->warning (lineno, filename, tmp);
  else printf ("Warning (line %d): %s\n", lineno, tmp);
} // CANdbFileIo::warning


void CANdbFileIo::info (const char *fmt, ...)
{
  char tmp [MAX_MSG + 1];
  va_list ap;
  va_start (ap, fmt);
  vsnprintf (tmp, MAX_MSG, fmt, ap);
  va_end (ap);
  if (message_logger) message_logger->info (lineno, filename, tmp);
  else printf ("%s\n", tmp);
} // CANdbFileIo::info


// ****************************************************************************


CANdbFileFormat::CANdbFileFormat (const char *fname,
                                  const char *descr_short,
                                  const char *descr_long,
                                  const char *extension,
                                  unsigned long f,
                                  CANdbFileIoBuildPtr b_ptr)
{
  flags = f;
  build_ptr = b_ptr;
  next = NULL;

  name = new char [strlen (fname) + 1];
  strcpy (name, fname);

  description_short = NULL;
  if (descr_short) {
    description_short = new char [strlen (descr_short) + 1];
    strcpy (description_short, descr_short);
  }

  description_long = NULL;
  if (descr_long) {
    description_long = new char [strlen (descr_long) + 1];
    strcpy (description_long, descr_long);
  }

  file_extension = NULL;
  if (extension) {
    file_extension = new char [strlen (extension) + 1];
    strcpy (file_extension, extension);
  }

  register_candb_file_format (this);
} // CANdbFileFormat::CANdbFileFormat


CANdbFileFormat::~CANdbFileFormat ()
{
  if (description_long) delete [] description_long;
  if (description_short) delete [] description_short;
  if (name) delete [] name;
  if (file_extension) delete [] file_extension;
  name = NULL;
  file_extension = NULL;
  description_short = NULL;
  description_long = NULL;
} // CANdbFileFormat::~CANdbFileFormat ()


// Statische Funktionen
void CANdbFileFormat::register_candb_file_format (CANdbFileFormat *clf)
{
  clf->next = NULL;
  if (!candb_file_format_first || !candb_file_format_last) {
    candb_file_format_first = candb_file_format_last = clf;
  }
  else {
    candb_file_format_last->next = clf;
    candb_file_format_last = clf;
  }
} // CANdbFileFormat::register_candb_file_format


CANdbFileFormat* CANdbFileFormat::get_first_candb_file_format ()
{
  return candb_file_format_first;
} // CANdbFileFormat::get_first_candb_file_format ()


CANdbFileFormat* CANdbFileFormat::get_candb_file_format_by_extension (const char *extension)
{
  if (!extension) return NULL;

  CANdbFileFormat *clf = candb_file_format_first;
  while (clf) {
    if (clf->file_extension && (_stricmp (extension, clf->file_extension) == 0)) break;
    clf = clf->next;
  }
  return clf;
} // CANdbFileFormat::get_candb_file_format_by_extension


CANdbFileFormat* CANdbFileFormat::get_candb_file_format_by_name (const char *name)
{
  if (!name) return NULL;

  CANdbFileFormat *clf = candb_file_format_first;
  while (clf) {
    if (clf->name && (strcmp (name, clf->name) == 0)) break;
    clf = clf->next;
  }
  return clf;
} // CANdbFileFormat::get_candb_file_format_by_name


CANdbFileIo* CANdbFileFormat::build (const char *name)
{
  CANdbFileIo *fio = NULL;

  if (!name) return NULL;

  CANdbFileFormat *ff = get_candb_file_format_by_name ((const char *) name);
  if (!ff) return NULL;

  if (ff->build_ptr) {
    fio = ff->build_ptr ();
    fio->set_file_format (ff);
  }

  return fio;
} // CANdbFileFormat::build


// ****************************************************************************


CANdbMessageLogger::CANdbMessageLogger ()
{
} // CANdbMessageLogger::CANdbMessageLogger


CANdbMessageLogger::~CANdbMessageLogger ()
{
} // CANdbMessageLogger::~CANdbMessageLogger


// ****************************************************************************


// Important for DIAdem stuff - don't remove it !!!
int candb_last ()
{
  global_dummy = 2;
  return 0;
} // candb_last


// ****************************************************************************
