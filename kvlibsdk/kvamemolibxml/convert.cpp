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
** Description: Convert XML strings to values
** -----------------------------------------------------------------------------
*/

#define NOMINMAX
#include <algorithm>
#include <string>
#include <cstring>
#include <stdexcept>
#include "convert.h"
#include "kvdebug.h"
#include "errorhandler.h"
#include "xmlref.h"
#include "util.h"
#include "memolibxml_feature_definitions.h"
using namespace std;

  #include <inttypes.h>
  #define __int64 int64_t

// ---------------------------------------------------------------------------
uint8_t num_to_uint8 (xmlNode *a_node, const char *attr_name)
{
  xmlChar *attr_value = NULL;
  char *p = NULL;
  uint8_t  tmp = 0;

  if (!a_node || !attr_name) throw_nullpointer(__FUNCTION__);

  attr_value = xmlGetProp(a_node, (const xmlChar *) attr_name);
  if (!attr_value) throw_attribute_not_found(a_node, attr_name);
  tmp = (uint8_t) strtoul((const char *) attr_value, &p, 0);
  if ((*p) != '\0') throw_attribute_value(a_node, attr_name, attr_value);
  xmlFree(attr_value);
  return tmp;
}

// ---------------------------------------------------------------------------
uint8_t num_to_uint8_or_default (xmlNode *a_node, const char *attr_name, uint8_t val)
{
  xmlChar *attr_value = NULL;
  char *p = NULL;
  uint8_t  tmp = val;

  if (!a_node || !attr_name) throw_nullpointer(__FUNCTION__);

  attr_value = xmlGetProp(a_node, (const xmlChar *) attr_name);
  if (attr_value) {
    tmp = (uint8_t) strtoul((const char *) attr_value, &p, 0);
    if ((*p) != '\0') {
      throw_attribute_value(a_node, attr_name, attr_value);
    }
    xmlFree(attr_value);
  }
  return tmp;
}

// ---------------------------------------------------------------------------
uint16_t num_to_uint16 (xmlNode *a_node, const char *attr_name)
{
  xmlChar *attr_value = NULL;
  char *p = NULL;
  uint16_t  tmp = 0;

  if (!a_node || !attr_name) throw_nullpointer(__FUNCTION__);

  attr_value = xmlGetProp(a_node, (const xmlChar *) attr_name);
  if (!attr_value) throw_attribute_not_found(a_node, attr_name);
  tmp = (uint16_t) strtoul((const char *) attr_value, &p, 0);
  if ((*p) != '\0') throw_attribute_value(a_node, attr_name, attr_value);
  xmlFree(attr_value);
  return tmp;
}

// ---------------------------------------------------------------------------
uint32_t num_to_uint32 (xmlNode *a_node, const char *attr_name)
{
  xmlChar *attr_value = NULL;
  char *p = NULL;
  uint32_t  tmp = 0;

  if (!a_node || !attr_name) throw_nullpointer(__FUNCTION__);

  attr_value = xmlGetProp(a_node, (const xmlChar *) attr_name);
  if (!attr_value) throw_attribute_not_found(a_node, attr_name);
  tmp = (uint32_t) strtoul((const char *) attr_value, &p, 0);
  if ((*p) != '\0') throw_attribute_value(a_node, attr_name, attr_value);
  xmlFree(attr_value);
  return tmp;
}

// ---------------------------------------------------------------------------
uint32_t num_to_uint32_or_default (xmlNode *a_node, const char *attr_name, uint32_t val)
{
  xmlChar *attr_value = NULL;
  char *p = NULL;
  uint32_t  tmp = 0;

  if (!a_node || !attr_name) throw_nullpointer(__FUNCTION__);

  attr_value = xmlGetProp(a_node, (const xmlChar *) attr_name);
  if (attr_value) {
    tmp = (uint32_t) strtoul((const char *) attr_value, &p, 0);
    if ((*p) != '\0') throw_attribute_value(a_node, attr_name, attr_value);
    xmlFree(attr_value);
  }
  else {
    tmp = val;
  }
  return tmp;
}

// ---------------------------------------------------------------------------
uint8_t yes_to_uint8 (xmlNode *a_node, const char *attr_name)
{
  xmlChar *attr_value = NULL;
  uint8_t tmp = 0xFF;

  if (!a_node || !attr_name) throw_nullpointer(__FUNCTION__);

  attr_value = xmlGetProp(a_node, (const xmlChar*) attr_name);
  if (!attr_value) throw_attribute_not_found(a_node, attr_name);

  if (!xmlStrcmp(attr_value,(const xmlChar*) "YES")) {
    tmp = 1;
  } else
  if (!xmlStrcmp(attr_value, (const xmlChar*)"NO")) {
    tmp = 0;
  }
  if (0xFF == tmp) throw_attribute_value(a_node, attr_name, attr_value);

  xmlFree(attr_value);
  return tmp;
}

// ---------------------------------------------------------------------------
uint8_t yes_to_uint8_or_default (xmlNode *a_node, const char *attr_name, uint8_t val)
{
  xmlChar *attr_value = NULL;
  uint8_t tmp = 0xFF;

  if (!a_node || !attr_name) throw_nullpointer(__FUNCTION__);

  attr_value = xmlGetProp(a_node, (const xmlChar*) attr_name);
  if (attr_value) {
    if (!xmlStrcmp(attr_value,(const xmlChar*) "YES")) {
    tmp = 1;
    } else
    if (!xmlStrcmp(attr_value, (const xmlChar*)"NO")) {
    tmp = 0;
    }
    if (0xFF == tmp) throw_attribute_value(a_node, attr_name, attr_value);
    xmlFree(attr_value);
  }
  else {
    tmp = val;
  }

  return tmp;
}

// ---------------------------------------------------------------------------
uint8_t protocol_to_uint8_or_default (xmlNode *a_node)
{
  xmlChar *attr_value = NULL;
  uint8_t tmp = HLP_TYPE_NONE;

  if (!a_node) throw_nullpointer(__FUNCTION__);

  // Note that this function is used for filters as well
  attr_value = xmlGetProp(a_node, (const xmlChar*) XML_TRIGGERS_ATTR_ID_PROT);
  if (attr_value) {
    if (!xmlStrcmp(attr_value,(const xmlChar*) PROTOCOL_J1939)) {
      tmp = HLP_TYPE_J1939;
    } else if (!xmlStrcmp(attr_value, (const xmlChar*) PROTOCOL_NONE)) {
      tmp = HLP_TYPE_NONE;
    } else {
      // Error handler will free attr_value
      throw_attribute_value(a_node, XML_TRIGGERS_ATTR_ID_PROT, attr_value);
    }
    xmlFree(attr_value);
  }
  return (tmp & HLP_TYPE_MASK);
}

// ---------------------------------------------------------------------------
uint8_t protocol_msk_to_uint8_or_default (xmlNode *a_node)
{
  xmlChar *attr_value = NULL;
  uint8_t tmp = 0;

  if (!a_node) throw_nullpointer(__FUNCTION__);

  attr_value = xmlGetProp(a_node, (const xmlChar*) XML_TRIGGERS_ATTR_ID_MSK);

  if (attr_value) {
    if (xmlStrstr(attr_value,(const xmlChar*) ID_MSK_SRC)) {
      tmp |= J1939_ID_SOURCE_ADDRESS;
    }
    if (xmlStrstr(attr_value, (const xmlChar*) ID_MSK_DST)) {
      tmp |= J1939_ID_DEST_ADDRESS;
    }
    if (xmlStrstr(attr_value, (const xmlChar*) ID_MSK_PGN)) {
      tmp |= J1939_ID_PGN_NO;
    }
    xmlFree(attr_value);
  }
  return (tmp & HLP_SPEC_MASK);
}

// ---------------------------------------------------------------------------
string uint8_to_protocol_msk(uint8_t msk)
{
  string str;
  if (msk & J1939_ID_PGN_NO) {
    str.append(ID_MSK_PGN);
  }

  if (msk & J1939_ID_SOURCE_ADDRESS) {
    if (str.size()) {
      str.append(", ");
    }
    str.append(ID_MSK_SRC);
  }

  if (msk & J1939_ID_DEST_ADDRESS) {
    if (str.size()) {
      str.append(", ");
    }
    str.append(ID_MSK_DST);
  }
  return str;
}

// ---------------------------------------------------------------------------
string uint8_to_protocol(uint8_t msk)
{
  string str;
  if (msk & HLP_TYPE_J1939) {
    str.append(PROTOCOL_J1939);
  }
  else {
    str.append(PROTOCOL_NONE);
  }
  return str;
}

// ---------------------------------------------------------------------------
bool getAttributeString (xmlNode *a_node, const char *attr_name, char * buf, unsigned int len)
{
  xmlChar *attr_value = NULL;
  bool found = false;

  if (!a_node || !attr_name || !buf || !len) throw_nullpointer(__FUNCTION__);

  attr_value = xmlGetProp(a_node, (const xmlChar*) attr_name);
  if (attr_value) {
    found = true;
    strncpy(buf, (const char*) attr_value, len);
    xmlFree(attr_value);
  }
  return found;
}

// ---------------------------------------------------------------------------
bool getAttributeString (xmlNode *a_node, const char *attr_name, std::string& str)
{
  xmlChar *attr_value = NULL;
  bool found = false;

  if (!a_node || !attr_name ) throw_nullpointer(__FUNCTION__);

  attr_value = xmlGetProp(a_node, (const xmlChar*) attr_name);
  if (attr_value) {
    found = true;
    str.append((const char*) attr_value);
    xmlFree(attr_value);
  }
  return found;
}


// ---------------------------------------------------------------------------
uint8_t node_name_to_uint8 (xmlNode *a_node, char const * const string_list[],
                            const int* value_list, unsigned int len)
{
  unsigned int idx = 0;

  if (!a_node || !string_list || !value_list || !len)
    throw_nullpointer(__FUNCTION__);
  for (idx=0; idx < len; idx++) if (!xmlStrcmp(a_node->name, (const xmlChar *) string_list[idx])) break;

  if (idx == len) throw_attribute_value(a_node, (const char*)a_node->name, (xmlChar*)"list of TRIGGERS or ACTIONS");
  PRINTF(("node_name_to_uint8 (%s) -> %d\n", a_node->name, value_list[idx]));
  return (uint8_t) value_list[idx];
}

// ---------------------------------------------------------------------------
const char *uint8_to_name (uint8_t val, char const * const string_list[],
                           const int* value_list, unsigned int len)
{
  unsigned int idx = 0;

  if (!string_list || !value_list || !len)
    throw_nullpointer(__FUNCTION__);

  for (idx=0; idx < len; idx++) {
    if (val == value_list[idx]) break;
  }

  if (idx == len) {
    throw_xml_writer_failure("Unknown type in list of TRIGGERS, FILTERS or ACTIONS");
  }

  PRINTF(("uint8_to_name (%d) -> %s\n", val, string_list[idx]));

  return string_list[idx];
}

// ---------------------------------------------------------------------------
uint8_t string_to_uint8 (xmlNode *a_node, const char *attr_name,
                         char const * const string_list[],
                         const int* value_list, unsigned int len)
{
  xmlChar *attr_value = NULL;
  unsigned int idx = 0;

  if (!a_node || !attr_name || !string_list || !value_list || !len)
    throw_nullpointer(__FUNCTION__);

  attr_value = xmlGetProp(a_node, (const xmlChar *) attr_name);
  if (!attr_value) throw_attribute_not_found(a_node, attr_name);

  for (idx=0; idx < len; idx++) if (!xmlStrcmp(attr_value, (const xmlChar *) string_list[idx])) break;

  if (idx == len) throw_attribute_value(a_node, attr_name, attr_value);

  xmlFree(attr_value);
  return (uint8_t) value_list[idx];
}

// ---------------------------------------------------------------------------
bool postfixUsesTrigger (uint8_t *postFixExpr, const uint8_t triggerIdx)
{
  for (int j=0; postFixExpr[j] != EXPR_OP_NONE; j++) {
    if (IS_VARIABLE(postFixExpr[j]) && triggerIdx == postFixExpr[j]) {
      return true;
    }
  }
  return false;
}

// ---------------------------------------------------------------------------
bool postfixUsesAnyTrigger (uint8_t *postFixExpr, const uint8_t *triggerIdxList, int cnt)
{
  for (int k = 0; k < cnt; k++) {
    for (int j=0; postFixExpr[j] != EXPR_OP_NONE; j++) {
      if (IS_VARIABLE(postFixExpr[j]) && triggerIdxList[k] == postFixExpr[j]) {
        return true;
      }
    }
  }
  return false;
}

// ---------------------------------------------------------------------------
int postfixToTriggerList (uint8_t *postFixExpr, uint8_t *buf, int len)
{
  int count = 0;
  for (int j=0; postFixExpr[j] != EXPR_OP_NONE; j++) {
    if (IS_VARIABLE(postFixExpr[j]) && count < len) {
      buf[count++] = postFixExpr[j];
    }
  }
  return count;
}

//-----------------------------------------------------------------------------
// Parse (binary) postfix expression and return expression as an (infix) string
std::string getInfixExpr(uint8_t *postfixExpr) {
  std::string infixString, symbol1, symbol2, theOperator;
  std::vector<std::string> symbolStack;

  try {
    for (int j = 0; postfixExpr[j] != EXPR_OP_NONE; j++) {
      if ( (postfixExpr[j] != EXPR_OP_AND) &&
           (postfixExpr[j] != EXPR_OP_OR) ) {
        symbolStack.push_back(intToString(postfixExpr[j]));
      } else {
        if (postfixExpr[j] == EXPR_OP_AND) {
          theOperator = " AND ";
        } else {
          theOperator = " OR ";
        }

        if (symbolStack.size() >= 2) {
          symbol1 = symbolStack.back();
          symbolStack.pop_back();
          symbol2 = symbolStack.back();
          symbolStack.pop_back();
          symbolStack.push_back("(" + symbol2 + theOperator + symbol1 + ")");
        } else {
          set_error_and_throw(KvaXmlStatusERR_EXPRESSION,
              "getInfixExpr: Malformed postfix expression; to few symbols on stack.");
        }
      }
    }
  } catch (std::out_of_range&) {
    set_error_and_throw(KvaXmlStatusERR_EXPRESSION,
        "getInfixExpr: Malformed postfix expression; index out of range.");
  }

  infixString = symbolStack.back();
  symbolStack.pop_back();

  if (!symbolStack.empty()) {
    set_error_and_throw(KvaXmlStatusERR_EXPRESSION,
        "getInfixExpr: Malformed postfix expression; stack not empty.");
  }

  return infixString;
}

// ---------------------------------------------------------------------------
uint16_t filterflags_to_uint16(xmlNode *a_node)
{
  uint16_t flags = 0;
  if (!a_node) throw_nullpointer(__FUNCTION__);
  if (yes_to_uint8(a_node, "flag_std"))         flags |=  FILTER_FLAG_STD;
  if (yes_to_uint8(a_node, "flag_ext"))         flags |=  FILTER_FLAG_EXT;
  if (yes_to_uint8(a_node, "flag_errorframe"))  flags |=  FILTER_FLAG_ERRORFRAME;
  return flags;
}

// ---------------------------------------------------------------------------
uint8_t datatype_to_uint8(xmlNode *a_node)
{
  uint8_t type = 0;

  xmlChar *attr_value = NULL;

  if (!a_node) throw_nullpointer(__FUNCTION__);

  attr_value = xmlGetProp(a_node, (const xmlChar*) XML_TRIGGERS_ATTR_DATATYPE);

  if (attr_value) {
    if (!xmlStrcmp(attr_value,(const xmlChar*) DATATYPE_UNSIGNED)) {
      // Default; no mask
    } else if (!xmlStrcmp(attr_value, (const xmlChar*) DATATYPE_SIGNED)) {
      type |=  DE_FORMAT_SIGN_BIT;
    } else {
      // Error handler will free attr_value
      throw_attribute_value(a_node, XML_TRIGGERS_ATTR_DATATYPE, attr_value);
    }
  }

  attr_value = xmlGetProp(a_node, (const xmlChar*) XML_TRIGGERS_ATTR_ENDIAN);

  if (attr_value) {
    if (!xmlStrcmp(attr_value,(const xmlChar*) ENDIAN_LITTLE)) {
      // Default; no mask
    } else if (!xmlStrcmp(attr_value, (const xmlChar*) ENDIAN_BIG)) {
      type |=  DE_FORMAT_BIGENDIAN_BIT;
    } else {
      // Error handler will free attr_value
      throw_attribute_value(a_node, XML_TRIGGERS_ATTR_ENDIAN, attr_value);
    }
  }

  return type;
}

// ---------------------------------------------------------------------------
std::string datatype_to_string(uint8_t type)
{
  std::string str;
  if ( type & DE_FORMAT_SIGN_BIT ) {
    str.append(DATATYPE_SIGNED);
  } else {
    // Default
    str.append(DATATYPE_UNSIGNED);
  }
  return str;
}

// ---------------------------------------------------------------------------
std::string endian_to_string(uint8_t type)
{
  std::string str;
  if ( type & DE_FORMAT_BIGENDIAN_BIT ) {
    str.append(ENDIAN_BIG);
  } else {
    // Default
    str.append(ENDIAN_LITTLE);
  }
  return str;
}

// ---------------------------------------------------------------------------
uint8_t pass_to_uint8 (xmlNode *a_node, const char *attr_name)
{
  xmlChar *attr_value = NULL;
  uint8_t tmp = 0xFF;

  if (!a_node || !attr_name) throw_nullpointer(__FUNCTION__);

  attr_value = xmlGetProp(a_node, (const xmlChar*) attr_name);
  if (!attr_value) throw_attribute_not_found(a_node, attr_name);

  if (!xmlStrcmp(attr_value,(const xmlChar*) "PASS")) {
    tmp = FILTER_INTERNAL_FLAG_PASS;
  } else
  if (!xmlStrcmp(attr_value, (const xmlChar*)"STOP")) {
    tmp = FILTER_INTERNAL_FLAG_STOP;
  }
  if (0xFF == tmp) throw_attribute_value(a_node, attr_name, attr_value);

  xmlFree(attr_value);
  return tmp;
}

// ---------------------------------------------------------------------------
uint8_t filtertype_bin_to_xml(uint8_t bin_type)
{
  uint8_t xml_type;

  if (bin_type & FILTER_INTERNAL_FLAG_PASS) {
    if (bin_type & FILTER_INTERNAL_FLAG_ID) {
      xml_type = MESSAGE_PASS_TYPE;
    }
    else if (bin_type & FILTER_INTERNAL_FLAG_SIGNAL) {
      xml_type = SIGNAL_PASS_TYPE;
    }
    else {
      xml_type = FLAGS_PASS_TYPE;
    }
  } else {
    if (bin_type & FILTER_INTERNAL_FLAG_ID) {
      xml_type = MESSAGE_STOP_TYPE;
    }
    else if (bin_type & FILTER_INTERNAL_FLAG_SIGNAL) {
      xml_type = SIGNAL_STOP_TYPE;
    }
    else {
      xml_type = FLAGS_STOP_TYPE;
    }
  }

  if (bin_type & FILTER_INTERNAL_FLAG_CNTR) {
    switch (xml_type) {
      case MESSAGE_PASS_TYPE : xml_type = MESSAGE_COUNTING_PASS_TYPE; break;
      case SIGNAL_PASS_TYPE  : xml_type = SIGNAL_COUNTING_PASS_TYPE; break;
      case FLAGS_PASS_TYPE   : xml_type = FLAGS_COUNTING_PASS_TYPE; break;
    }
  }

  return xml_type;
}

// ---------------------------------------------------------------------------
size_t getElementString(xmlNode *a_node, char *buf, size_t len)
{
  xmlChar *attr_string = NULL;

  if (!a_node || !buf) throw_nullpointer(__FUNCTION__);

  attr_string = xmlNodeGetContent(a_node);
  if (!attr_string) return 0;

  strncpy(buf, (const char*) attr_string, len);
  xmlFree(attr_string);
  return strnlen(buf, len);
}

// ---------------------------------------------------------------------------
size_t getElementString(xmlNode *a_node, std::string & str)
{
  xmlChar *attr_string = NULL;

  if (!a_node) throw_nullpointer(__FUNCTION__);

  attr_string = xmlNodeGetContent(a_node);
  if (!attr_string) return 0;

  str.append((const char*) attr_string);
  xmlFree(attr_string);
  return str.size();
}


// ---------------------------------------------------------------------------
long int element_string_to_long (xmlNode *element_node)
{
  xmlChar *attr_string = NULL;
  std::string str;
  long int res = 0;

  if (!element_node) throw_nullpointer(__FUNCTION__);

  attr_string = xmlNodeGetContent(element_node);
  if (!attr_string) {
    throw_element_not_found(element_node, (const char*) element_node->name);
  }
  str.append((const char*) attr_string);

 try {
   // This is C++ 11, but it is convenient...
  res =  std::stoi(str, NULL);
 }
 catch (int err) {
   (void) err;
   throw_element_value(element_node, attr_string);
 }

  xmlFree(attr_string);
  return res;
}

// ---------------------------------------------------------------------------
void getBusPar(xmlNode *bus_node, uint32_t *bitrate, uint8_t *tseg1,
                      uint8_t *tseg2, uint8_t *sjw)
{
  if (!bus_node) throw_nullpointer(__FUNCTION__);

  *bitrate = num_to_uint32(bus_node, XML_BUSPARAM_ATTR_BITRATE);
  if (!*bitrate) {
    throw_value_range(bus_node, XML_BUSPARAM_ATTR_BITRATE, *bitrate,
      KVASER_MIN_BITRATE, KVASER_MAX_BITRATE);
  }

  *tseg1 = num_to_uint8(bus_node, XML_BUSPARAM_ATTR_TSEG1);
  if (*tseg1 < TSEG1_MIN || *tseg1 > TSEG1_MAX) {
    throw_value_range(bus_node, XML_BUSPARAM_ATTR_TSEG1, *tseg1, TSEG1_MIN, TSEG1_MAX);
  }

  *tseg2 = num_to_uint8(bus_node, XML_BUSPARAM_ATTR_TSEG2);
  if (*tseg2 < TSEG2_MIN || *tseg2 > TSEG2_MAX) {
    throw_value_range(bus_node, XML_BUSPARAM_ATTR_TSEG2, *tseg2, TSEG2_MIN, TSEG2_MAX);
  }

  *sjw = num_to_uint8(bus_node, XML_BUSPARAM_ATTR_SJW);
  if (*sjw < SJW_MIN || *sjw > SJW_MAX) {
    throw_value_range(bus_node, XML_BUSPARAM_ATTR_SJW, *sjw, SJW_MIN, SJW_MAX);
  }
}

// ---------------------------------------------------------------------------
void getBusParFd(xmlNode *bus_node, uint32_t *bitrate, uint16_t *tseg1,
                      uint16_t *tseg2, uint8_t *sjw,
                      uint32_t *bitrate_brs, uint16_t *tseg1_brs,
                      uint16_t *tseg2_brs, uint8_t *sjw_brs)
{
  if (!bus_node) throw_nullpointer(__FUNCTION__);

  // Arbitration phase
  *bitrate = num_to_uint32(bus_node, XML_BUSPARAM_ATTR_BITRATE);
  if (*bitrate < KVASER_MIN_BITRATE || *bitrate > KVASER_MAX_BITRATE) {
    throw_value_range(bus_node, XML_BUSPARAM_ATTR_BITRATE, *bitrate,
      KVASER_MIN_BITRATE, KVASER_MAX_BITRATE);
  }

  *tseg1 = num_to_uint16(bus_node, XML_BUSPARAM_ATTR_TSEG1);
  if (*tseg1 < TSEG1_ARB_MIN || *tseg1 > TSEG1_ARB_MAX) {
    throw_value_range(bus_node, XML_BUSPARAM_ATTR_TSEG1, *tseg1, TSEG1_ARB_MIN,
      TSEG1_ARB_MAX);
  }

  *tseg2 = num_to_uint16(bus_node, XML_BUSPARAM_ATTR_TSEG2);
  if (*tseg2 < TSEG2_ARB_MIN || *tseg2 > TSEG2_ARB_MAX) {
    throw_value_range(bus_node, XML_BUSPARAM_ATTR_TSEG2, *tseg2, TSEG2_ARB_MIN,
      TSEG2_ARB_MAX);
  }

  *sjw = num_to_uint8(bus_node, XML_BUSPARAM_ATTR_SJW);
  if (*sjw < SJW_ARB_MIN || *sjw > SJW_ARB_MAX) {
    throw_value_range(bus_node, XML_BUSPARAM_ATTR_SJW, *sjw, SJW_ARB_MIN,
      SJW_ARB_MAX);
  }
  // Data phase with bitrate switch (BRS)
  *bitrate_brs = num_to_uint32(bus_node, XML_BUSPARAM_ATTR_BITRATE_BRS);
  if (*bitrate_brs < KVASER_MIN_BITRATE_FD || *bitrate_brs > KVASER_MAX_BITRATE_FD) {
    throw_value_range(bus_node, XML_BUSPARAM_ATTR_BITRATE_BRS, *bitrate_brs,
      KVASER_MIN_BITRATE_FD, KVASER_MAX_BITRATE_FD);
  }

  *tseg1_brs = num_to_uint16(bus_node, XML_BUSPARAM_ATTR_TSEG1_BRS);
  if (*tseg1_brs < TSEG1_DATA_MIN || *tseg1_brs > TSEG1_DATA_MAX) {
    throw_value_range(bus_node, XML_BUSPARAM_ATTR_TSEG1_BRS, *tseg1_brs, TSEG1_DATA_MIN,
      TSEG1_DATA_MAX);
  }

  *tseg2_brs = num_to_uint16(bus_node, XML_BUSPARAM_ATTR_TSEG2_BRS);
  if (*tseg2_brs < TSEG2_DATA_MIN || *tseg2_brs > TSEG2_DATA_MAX) {
    throw_value_range(bus_node, XML_BUSPARAM_ATTR_TSEG2_BRS, *tseg2_brs, TSEG2_DATA_MIN,
      TSEG2_DATA_MAX);
  }

  *sjw_brs = num_to_uint8(bus_node, XML_BUSPARAM_ATTR_SJW_BRS);
  if (*sjw_brs < SJW_DATA_MIN || *sjw_brs > SJW_DATA_MAX) {
    throw_value_range(bus_node, XML_BUSPARAM_ATTR_SJW_BRS, *sjw_brs, SJW_DATA_MIN,
      SJW_DATA_MAX);
  }
}


// ---------------------------------------------------------------------------
static void eanStrToBcd(std::string eanStr, uint32_t &ean_hi, uint32_t &ean_lo)
{
  __int64 ean64;

  // Removed all '-' characters in EAN string.
  while (eanStr.find_first_of('-') != string::npos)
  {
    eanStr.erase(eanStr.find_first_of("-"), 1);
  }

  ean64 = 0;
  for (unsigned i = 0; i < eanStr.size(); i++)
  {
    int m = (int)eanStr[i] - (int)'0';
    ean64 <<= 4;
    ean64 |= m;
  }

  ean_hi = (uint32_t)(ean64 >> 32);
  ean_lo = (uint32_t)ean64;
}


// ---------------------------------------------------------------------------
static void checkBusParFd(xmlNode *bus_node, std::vector<std::string> *eanStrList,
                          uint32_t bitrate_brs, uint16_t tseg1_brs, uint16_t tseg2_brs,
                          uint8_t /* sjw_brs */)
{
  for (size_t k = 0; k < eanStrList->size(); k++) {
    std::string eanStr = eanStrList->at(k);

    uint32_t ean_lo;
    uint32_t ean_hi;

    eanStrToBcd(eanStr, ean_hi, ean_lo);

    int clockFreq = getClockFreqMhz(ean_hi, ean_lo) * 1000 * 1000;
    int brp = clockFreq / ((1 + tseg1_brs + tseg2_brs) * bitrate_brs);

    PRINTF(("checkBusParFd: eanStr='%s' ean_hi=0x%08X ean_lo=0x%08X\n", eanStr.c_str(),
            ean_hi, ean_lo));
    PRINTF(("checkBusParFd: clockFreq=%i brp=%i\n", clockFreq, brp));

    if (brp < BRP_MIN || brp > BRP_MAX)
      throw_value_range(bus_node, XML_BUSPARAM_ATTR_BRP_BRS, brp, BRP_MIN, BRP_MAX);

  }
}

// ---------------------------------------------------------------------------
uint32_t bitrateToBusParams (xmlNode *bus_node, ConfigFileBusParams* bus)
{
  uint32_t bitrate;
  uint8_t tseg1;
  uint8_t tseg2;
  uint32_t psc;
  uint8_t sjw;

  if (!bus_node) throw_nullpointer(__FUNCTION__);

  getBusPar(bus_node, &bitrate, &tseg1, &tseg2, &sjw);

  psc = CAN_SYS_FREQ / (PSC_FACTOR * bitrate * (1 + tseg1 + tseg2));
  if (psc < PSC_MIN || psc > PSC_MAX) {
    set_error_and_throw ( KvaXmlStatusERR_INTERNAL,
      "The calculated CAN pre-scaler value is invalid: %lu. "
      "Please check bitrate settings in '%s'.", psc, XML_BUSPARAM);
  }

  if (bus) {
    bus->used      = 1;
    bus->psc       = (uint8_t) psc;
    bus->prSeg     = 1;
    bus->phSeg1    = tseg1 - 1;
    bus->phSeg2    = tseg2;
    bus->sjw       = sjw;
    bus->samples   = 1;
    bus->silent    = yes_to_uint8_or_default(bus_node, XML_BUSPARAM_ATTR_SILENT, 1);
    PRINTF(("bitrateToBusParams: prop_seg=%u phase_seg1=%u phase_seg2=%u prescaler=%u\n",
      bus->prSeg, bus->phSeg1, bus->phSeg2,bus->psc));
  }
  return bitrate;
}


// ---------------------------------------------------------------------------
void busParamsToBitrate (ConfigFileBusParams* bus, uint32_t *bitrate,
                         uint8_t *tseg1, uint8_t *tseg2)
{
  if (!bus || !bitrate || !tseg1 || !tseg2 ) throw_nullpointer(__FUNCTION__);

  *tseg1 = bus->prSeg + bus->phSeg1;
  if (*tseg1 < TSEG1_MIN || *tseg1 > TSEG1_MAX) {
    set_error_and_throw ( KvaXmlStatusERR_VALUE_RANGE,
      "The value %d for '%s' must be within the range from %d to %d.\n",
      XML_BUSPARAM_ATTR_TSEG1, tseg1, TSEG1_MIN, TSEG1_MAX);
  }
  *tseg2 = bus->phSeg2;

  if (*tseg2 < TSEG2_MIN || *tseg2 > TSEG2_MAX) {
    set_error_and_throw ( KvaXmlStatusERR_VALUE_RANGE,
      "The value %d for '%s' must be within the range from %d to %d.\n",
      XML_BUSPARAM_ATTR_TSEG2, tseg2, TSEG2_MIN, TSEG2_MAX);
  }

  uint32_t tmp =  (PSC_FACTOR * bus->psc * (1 + *tseg1 + *tseg2));
  *bitrate = 0;
  if (tmp) *bitrate = CAN_SYS_FREQ / tmp;
  if (*bitrate == 0) {
    set_error_and_throw ( KvaXmlStatusERR_VALUE_RANGE,
      "The value %d for '%s' must be within the range from %d to %d.\n",
      XML_BUSPARAM_ATTR_BITRATE, *bitrate, KVASER_MIN_BITRATE, KVASER_MAX_BITRATE);
  }
      PRINTF(("busParamsToBitrate: bitrate=%u tseg1=%u tseg2=%u\n", *bitrate, *tseg1, *tseg2));
}

// ---------------------------------------------------------------------------
uint32_t bitrateToBusParamsFd (xmlNode *bus_node, std::vector<std::string> *eanList, ConfigFileBusParamsFd* busfd)
{
  if (!bus_node || !busfd ) throw_nullpointer(__FUNCTION__);

  busfd->used = 1;
  busfd->silent = yes_to_uint8_or_default(bus_node, XML_BUSPARAM_ATTR_SILENT, 1);

  if ( !isCanFdBusParameters(bus_node) ) {
    busfd->can_mode = OPEN_AS_CAN;
    getBusPar(bus_node, &busfd->bitrate, (uint8_t*)&busfd->tseg1,
              (uint8_t*)&busfd->tseg2, &busfd->sjw);
  }
  else {
    if (yes_to_uint8(bus_node, XML_BUSPARAM_ATTR_ISO_BRS)) {
      busfd->can_mode = OPEN_AS_CANFD_ISO;
    } else {
      busfd->can_mode = OPEN_AS_CANFD_NONISO;
    }
    getBusParFd(bus_node, &busfd->bitrate, &busfd->tseg1, &busfd->tseg2, &busfd->sjw,
                &busfd->bitrate_brs, &busfd->tseg1_brs, &busfd->tseg2_brs,
                &busfd->sjw_brs);

    checkBusParFd(bus_node, eanList, busfd->bitrate_brs, busfd->tseg1_brs, busfd->tseg2_brs, busfd->sjw_brs);
  }

  return std::max(busfd->bitrate, busfd->bitrate_brs);
}
