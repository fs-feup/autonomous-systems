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
** Description: Handle errors and throw exceptions
** -----------------------------------------------------------------------------
*/

#include "errorhandler.h"
#include "kvdebug.h"

static char ErrorMessage[XML_ERROR_MESSAGE_LENGTH] = {0};
static KvaXmlStatus ErrorCode = KvaXmlStatusOK;

void set_error_status (const KvaXmlStatus status)
{
  ErrorCode = status;
}

KvaXmlStatus get_error_status (void)
{
  return ErrorCode;
}

const char *  get_error_message (void)
{
  return ErrorMessage;
}


void set_error_message ( const char * format, ... )
{
  va_list args;
  va_start (args, format);
  vsnprintf(ErrorMessage, sizeof(ErrorMessage), format, args);
  va_end (args);
}

void set_error ( const KvaXmlStatus status, const char * format, ... )
{
  ErrorCode = status;
  va_list args;
  va_start (args, format);
  vsnprintf(ErrorMessage, sizeof(ErrorMessage), format, args);
  va_end (args);
}

void set_error_and_throw ( const KvaXmlStatus status, const char * format, ... )
{
  ErrorCode = status;
  va_list args;
  va_start (args, format);
  vsnprintf(ErrorMessage, sizeof(ErrorMessage), format, args);
  va_end (args);
  throw(ErrorCode);
}

// ---------------------------------------------------------------------------
void throw_nullpointer (const char* fcn_name)
{
  sprintf(ErrorMessage,"Function '%s()' called with NULL.\n", fcn_name);
  ErrorCode = KvaXmlStatusERR_INTERNAL;
  throw(ErrorCode);
}
// ---------------------------------------------------------------------------
void throw_general_failure (const char* err_msg)
{
  if (!err_msg) throw_nullpointer(__FUNCTION__);
  sprintf(ErrorMessage,"Error: %s\n", err_msg);
  ErrorCode = KvaXmlStatusFail;
  throw(ErrorCode);
}

// ---------------------------------------------------------------------------
void throw_attribute_not_found (xmlNode *a_node, const char* attr_name)
{
  long line = 0;
  int len = 0;
  if (!a_node || !attr_name) throw_nullpointer(__FUNCTION__);

  line = xmlGetLineNo(a_node);
  if (line > 0) len = sprintf(ErrorMessage, "Error in XML element '%s' on line %ld:\n", a_node->name, line);
  sprintf(&ErrorMessage[len],"  Expected to find attribute '%s' in element '%s'.\n",
    attr_name, a_node->name);
  ErrorCode = KvaXmlStatusERR_ATTR_NOT_FOUND;
  throw(ErrorCode);
}

// ---------------------------------------------------------------------------
void throw_attribute_value (xmlNode *a_node, const char* attr_name, xmlChar *attr_value)
{
  long line = 0;
  int len = 0;
  if (!a_node || !attr_name || !attr_value) throw_nullpointer(__FUNCTION__);
  line = xmlGetLineNo(a_node);
  if (line > 0) len = sprintf(ErrorMessage, "Error in XML element '%s' on line %ld:\n", a_node->name, line);
  sprintf(&ErrorMessage[len],"  Illegal attribute value '%s' for attribute '%s'.\n",
    attr_value, attr_name);
  xmlFree(attr_value);
  ErrorCode = KvaXmlStatusERR_ATTR_VALUE;
  throw(ErrorCode);
}

// ---------------------------------------------------------------------------
void throw_element_value (xmlNode *a_node, xmlChar *attr_value)
{
  long line = 0;
  int len = 0;
  if (!a_node || !attr_value) throw_nullpointer(__FUNCTION__);
  line = xmlGetLineNo(a_node);
  if (line > 0) len = sprintf(ErrorMessage, "Error in XML element '%s' on line %ld:\n", a_node->name, line);
  sprintf(&ErrorMessage[len],"  Illegal value '%s'.\n",
    attr_value);
  xmlFree(attr_value);
  ErrorCode = KvaXmlStatusERR_ATTR_VALUE;
  throw(ErrorCode);
}

// ---------------------------------------------------------------------------
void throw_element_not_found (xmlNode *a_node, const char* element_name)
{
  long line = 0;
  int len = 0;
  if (!a_node || !element_name) throw_nullpointer(__FUNCTION__);

  line = xmlGetLineNo(a_node);
  if (line > 0) len = sprintf(ErrorMessage, "Error in XML element '%s' on line %ld:\n", a_node->name, line);
  sprintf(&ErrorMessage[len],"  Expected to find element '%s' below element '%s'.\n",
    element_name, a_node->name);
  ErrorCode = KvaXmlStatusERR_ELEM_NOT_FOUND;
  throw(ErrorCode);
}

// ---------------------------------------------------------------------------
void throw_wrong_element_count (xmlNode *a_node, const char* element_name, int count, int min, int max)
{
  long line = 0;
  int len = 0;
  if (!a_node || !element_name) throw_nullpointer(__FUNCTION__);

  line = xmlGetLineNo(a_node);
  if (line > 0) len = sprintf(ErrorMessage, "Error in XML element '%s' on line %ld:\n", a_node->name, line);
  sprintf(&ErrorMessage[len],"  Found %d element(s) of type '%s' in element '%s'."
    " Expected to find between %d and %d elements(s).\n",
    count, element_name, a_node->name, min, max);
  ErrorCode = KvaXmlStatusERR_ELEM_NOT_FOUND;
  throw(ErrorCode);
}

// ---------------------------------------------------------------------------
void throw_wrong_element_order (xmlNode *a_node, const char* first, const char* second,
                                       const char* parent)
{
  long line = 0;
  int len = 0;
  if (!a_node || !first || !second) throw_nullpointer(__FUNCTION__);

  line = xmlGetLineNo(a_node);
  if (line > 0) len = sprintf(ErrorMessage, "Error in XML element '%s' on line %ld:\n", a_node->name, line);
  sprintf(&ErrorMessage[len],"  Expected element '%s' before element '%s' in '%s'.\n",
    first, second, parent);
  ErrorCode = KvaXmlStatusERR_ELEM_NOT_FOUND;
  throw(ErrorCode);
}

// ---------------------------------------------------------------------------
void throw_value_range (xmlNode *a_node, const char* name, const int value,
                               const int lower_limit, const int upper_limit)
{
  long line = 0;
  int len = 0;
  if (!a_node || !name) throw_nullpointer(__FUNCTION__);

  line = xmlGetLineNo(a_node);
  if (line > 0) len = sprintf(ErrorMessage, "Error in XML element '%s' on line %ld:\n", a_node->name, line);
  sprintf(&ErrorMessage[len],"  The value %d for '%s' must be within the range from %d to %d.\n",
    value, name, lower_limit, upper_limit);
  ErrorCode = KvaXmlStatusERR_VALUE_RANGE;
  throw(ErrorCode);
}

// ---------------------------------------------------------------------------
void throw_value_unique (xmlNode *a_node, const char* name, const int value)
{
  long line = 0;
  int len = 0;
  if (!a_node || !name) throw_nullpointer(__FUNCTION__);

  line = xmlGetLineNo(a_node);
  if (line > 0) len = sprintf(ErrorMessage, "Error in XML element '%s' on line %ld:\n", a_node->name, line);
  sprintf(&ErrorMessage[len],"  The value for %d for '%s' must be unique within it's element type.\n",
    value, name);
  ErrorCode = KvaXmlStatusERR_VALUE_UNIQUE;
  throw(ErrorCode);
}

// ---------------------------------------------------------------------------
void throw_string_unique (xmlNode *a_node, const char* name, std::string& str)
{
  long line = 0;
  int len = 0;
  if (!a_node || !name) throw_nullpointer(__FUNCTION__);

  line = xmlGetLineNo(a_node);
  if (line > 0) len = sprintf(ErrorMessage, "Error in XML element '%s' on line %ld:\n", a_node->name, line);
  sprintf(&ErrorMessage[len],"  The string '%s' for attribute '%s' must be unique within it's element type.\n",
    str.c_str(), name);
  ErrorCode = KvaXmlStatusERR_VALUE_UNIQUE;
  throw(ErrorCode);
}

// ---------------------------------------------------------------------------
void throw_value_consecutive (xmlNode *a_node, const char* name)
{
  long line = 0;
  int len = 0;
  if (!a_node || !name) throw_nullpointer(__FUNCTION__);

  line = xmlGetLineNo(a_node);
  if (line > 0) len = sprintf(ErrorMessage, "Error in XML element '%s' on line %ld:\n", a_node->name, line);
  sprintf(&ErrorMessage[len],"  The values for '%s' must be consecutive within it's element type.\n", name);
  ErrorCode = KvaXmlStatusERR_VALUE_CONSECUTIVE;
  throw(ErrorCode);
}

// ---------------------------------------------------------------------------
void throw_invalid_expression (xmlNode *a_node)
{
  long line = 0;
  int len = 0;
  xmlChar *expr = NULL;
  if (!a_node) throw_nullpointer(__FUNCTION__);

  line = xmlGetLineNo(a_node);
  if (line > 0) len = sprintf(ErrorMessage, "Error in XML element '%s' on line %ld:\n", a_node->name, line);
  expr = xmlNodeGetContent(a_node);
  sprintf(&ErrorMessage[len],"  The trigger expression '%s' is not valid.\n",
    expr ? (const char*) expr : " ");
  xmlFree(expr);
  ErrorCode = KvaXmlStatusERR_EXPRESSION;
  throw(ErrorCode);
}

// ---------------------------------------------------------------------------
void warning_element_unknown (xmlNode *a_node)
{
  long line = 0;
  if (!a_node) throw_nullpointer(__FUNCTION__);

  if (a_node->type == XML_ELEMENT_NODE) {
    line = xmlGetLineNo(a_node);
    if (line >= 0) {
    PRINTF(("***** Warning: Unknown XML element '%s' on line %ld. *****", a_node->name, line));
  }
}
}

// ---------------------------------------------------------------------------
void throw_xml_writer_failure (const char* err_msg)
{
  if (!err_msg) throw_nullpointer(__FUNCTION__);
  sprintf(ErrorMessage,"Error: %s\n", err_msg);
  ErrorCode = KvaXmlStatusFail;
  throw(ErrorCode);
}

// ---------------------------------------------------------------------------
void throw_xml_writer_element_failure (const char* element)
{
  if (!element) throw_nullpointer(__FUNCTION__);
  sprintf(ErrorMessage,"Error: Could not write Element %s\n", element);
  ErrorCode = KvaXmlStatusFail;
  throw(ErrorCode);
}

// ---------------------------------------------------------------------------
void throw_xml_writer_attribute_failure (const char* element, const char* attribute)
{
  if (!element || attribute) throw_nullpointer(__FUNCTION__);
  sprintf(ErrorMessage,"Error: Could not write Attribute %s in Element %s\n", attribute, element);
  ErrorCode = KvaXmlStatusFail;
  throw(ErrorCode);
}

// ---------------------------------------------------------------------------
void throw_xml_writer_comment_failure (const char* comment)
{
  if (!comment) throw_nullpointer(__FUNCTION__);
  sprintf(ErrorMessage,"Error: Could not write Comment \"%s\"\n", comment);
  ErrorCode = KvaXmlStatusFail;
  throw(ErrorCode);
}

// ---------------------------------------------------------------------------
// Error texts
// ---------------------------------------------------------------------------

typedef struct {
    int errcode;
    const char* msg;
} errmsg_t;

static errmsg_t errmsg_table[] = {
  {KvaXmlStatusOK,                    "OK"},
  {KvaXmlStatusFail,                  "Generic error"},
  {KvaXmlStatusERR_ATTR_NOT_FOUND,    "Failed to find an attribute in a node"},
  {KvaXmlStatusERR_ATTR_VALUE,        "The attribute value is not correct, e.g. whitespace after a number"},
  {KvaXmlStatusERR_ELEM_NOT_FOUND,    "Could not find a required element"},
  {KvaXmlStatusERR_VALUE_RANGE,       "The value is outside the allowed range"},
  {KvaXmlStatusERR_VALUE_UNIQUE,      "The value is not unique; usually idx attributes"},
  {KvaXmlStatusERR_VALUE_CONSECUTIVE, "The values are not consecutive; usually idx attributes"},
  {KvaXmlStatusERR_EXPRESSION,        "The trigger expression could not be parsed"},
  {KvaXmlStatusERR_XML_PARSER,        "The XML settings contain syntax errors"},
  {KvaXmlStatusERR_DTD_VALIDATION,    "The XML settings do not follow the DTD"},
  {KvaXmlStatusERR_SCRIPT_ERROR,      "t-script related errors, e.g. file not found"},
  {KvaXmlStatusERR_INTERNAL,          "Internal errors, e.g. null pointers"},

  {KvaXmlValOffset + KvaXmlValidationStatusOK,                         "OK"},
  {KvaXmlValOffset + KvaXmlValidationStatusFail,                       "Generic error"},
  {KvaXmlValOffset + KvaXmlValidationStatusERR_ABORT,                  "Too many errors, validation aborted"},
  {KvaXmlValOffset + KvaXmlValidationStatusERR_SILENT_TRANSMIT,        "Transmit lists used in silent mode"},
  {KvaXmlValOffset + KvaXmlValidationStatusERR_UNDEFINED_TRIGGER,      "An undefined trigger is used in an expression"},
  {KvaXmlValOffset + KvaXmlValidationStatusERR_MULTIPLE_EXT_TRIGGER,   "There are more than one external trigger defined"},
  {KvaXmlValOffset + KvaXmlValidationStatusERR_MULTIPLE_START_TRIGGER, "There are more than one start up trigger defined"},
  {KvaXmlValOffset + KvaXmlValidationStatusERR_DISK_FULL_STARTS_LOG,   "A trigger on disk full starts the logging"},
  {KvaXmlValOffset + KvaXmlValidationStatusERR_NUM_OUT_OF_RANGE,       "A numerical value is out of range"},
  {KvaXmlValOffset + KvaXmlValidationStatusERR_SCRIPT_NOT_FOUND,       "A t-script file could not be opened"},
  {KvaXmlValOffset + KvaXmlValidationStatusERR_SCRIPT_TOO_LARGE,       "A t-script is too large for the configuration"},
  {KvaXmlValOffset + KvaXmlValidationStatusERR_SCRIPT_TOO_MANY,        "Too many active t-scripts for selected device"},
  {KvaXmlValOffset + KvaXmlValidationStatusERR_SCRIPT_CONFLICT,        "More than one active script is set as 'primary'"},
  {KvaXmlValOffset + KvaXmlValidationStatusERR_ELEMENT_COUNT,          "Too many or too few elements of this type"},

  {KvaXmlValOffset + KvaXmlValidationStatusWARN_ABORT,                  "Too many warnings, validation aborted"},
  {KvaXmlValOffset + KvaXmlValidationStatusWARN_NO_ACTIVE_LOG,          "No active logging detected"},
  {KvaXmlValOffset + KvaXmlValidationStatusWARN_DISK_FULL_AND_FIFO,     "A trigger on disk full used with FIFO mode"},
  {KvaXmlValOffset + KvaXmlValidationStatusWARN_IGNORED_ELEMENT,        "This XML element was ignored"},
  {KvaXmlValOffset + KvaXmlValidationStatusWARN_MULTIPLE_EXT_TRIGGER,   "Using more than one external trigger requires firmware version 3.7 or better."}
};

#define ERRMSG_TABLE_LEN  sizeof(errmsg_table)/sizeof(errmsg_table[0])

bool get_error_text(int code, const char* &msg)
{
  msg = NULL;
  for (unsigned int i=0; i<ERRMSG_TABLE_LEN; i++) {
    if (errmsg_table[i].errcode == code) {
      msg = errmsg_table[i].msg;
      return true;
    }
  }
  return false;
}
