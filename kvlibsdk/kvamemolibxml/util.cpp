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
** Description: XML utility functions
** -----------------------------------------------------------------------------
*/

#include "util.h"
#include "convert.h"
#include "errorhandler.h"
#include "kvdebug.h"
#include "kvaMemoLibXMLConst.h"
#include "os_util.h"

#include <locale>
#include <sstream>
#include <algorithm>
#include <iostream>
#include <sstream>
#include <cstring>


// ---------------------------------------------------------------------------
xmlNode *findFirstElementNode(xmlNode *root_node, const char *element_name)
{
  xmlNode *cur_node = NULL;
  if (!root_node || !element_name) throw_nullpointer(__FUNCTION__);
  for (cur_node = root_node->children; cur_node; cur_node = cur_node->next) {
    if (cur_node->type == XML_ELEMENT_NODE && !xmlStrcmp(cur_node->name, (const xmlChar*) element_name)) {
      return cur_node;
    }
  }
  return NULL;
}

// ---------------------------------------------------------------------------
xmlNode *findNextElementNode(xmlNode *cur_node, const char *element_name)
{
  if (!cur_node || !element_name) throw_nullpointer(__FUNCTION__);
  for (; cur_node; cur_node = cur_node->next) {
    if (cur_node->type == XML_ELEMENT_NODE && !xmlStrcmp(cur_node->name, (const xmlChar*) element_name)) {
      return cur_node;
    }
  }
  return NULL;
}

// ---------------------------------------------------------------------------
bool hasAttribute(xmlNode *a_node, const char *attr_name)
{
  xmlChar *attr_value = NULL;

  if (!a_node || !attr_name) throw_nullpointer(__FUNCTION__);

  attr_value = xmlGetProp(a_node, (const xmlChar*) attr_name);
  if (!attr_value) return false;
  xmlFree(attr_value);
  return true;
}

// ---------------------------------------------------------------------------
bool getVersion (xmlNode *root_node, const char *name, unsigned int *major,
  unsigned int *minor)
{
  std::string tmp;
  xmlNode *version_node = NULL;

  *major = 0;
  *minor = 0;

  if (!root_node) return false;

  version_node = findFirstElementNode(root_node, name);

  if (!version_node) return false;

  getElementString(version_node, tmp);
  clearws(tmp);

  // Major and minor e.g. '5.0'
  if (sscanf( tmp.c_str(),"%u.%u", major, minor) == 2 ) {
    return true;
  }

  // Major only e.g. '5'
  if (sscanf( tmp.c_str(),"%u", major) == 1)
  {
    return true;
  }

  return false;
}

// ---------------------------------------------------------------------------
bool isElementType(xmlNode *a_node, char const * const string_list[], unsigned int len)
{
  unsigned int idx = 0;
  bool res = false;

  if (!a_node) throw_nullpointer(__FUNCTION__);

  if (XML_ELEMENT_NODE == a_node->type) {
    for (idx = 0; idx < len; idx++) {
      if (!xmlStrcmp(a_node->name, (const xmlChar *) string_list[idx])) {
        res = true;
        break;
      }
    }
  }
  return res;
}

// ---------------------------------------------------------------------------
bool isElementType(xmlNode *a_node, char const * name)
{
  if (!a_node) throw_nullpointer(__FUNCTION__);

  return a_node->type == XML_ELEMENT_NODE && !xmlStrcmp(a_node->name, (const xmlChar *) name);
}

// ---------------------------------------------------------------------------
bool isElementActive(xmlNode *a_node)
{
  xmlChar *attr_value = NULL;
  bool active = true;

  if (!a_node) throw_nullpointer(__FUNCTION__);

  attr_value = xmlGetProp(a_node, (const xmlChar*) "active");
  if (attr_value) {
    if (!xmlStrcmp(attr_value, (const xmlChar*)"NO")) {
    active = false;
    }
  }
  return active;
}

// ---------------------------------------------------------------------------
bool isTriggerElement(xmlNode *a_node)
{
  if (!a_node) throw_nullpointer(__FUNCTION__);
  return isElementType(a_node, TrigvarTypeStrings, TrigvarTypeLength);
}

// ---------------------------------------------------------------------------
bool isStatementElement(xmlNode *a_node)
{
  if (!a_node) throw_nullpointer(__FUNCTION__);

  return isElementType(a_node, XML_STATEMENT);
}

// ---------------------------------------------------------------------------
bool isFilterElement(xmlNode *a_node)
{
  if (!a_node) throw_nullpointer(__FUNCTION__);

  return isElementType(a_node, FilterTypeStrings, FilterTypeLength);
}

// ---------------------------------------------------------------------------
bool isModeElement(xmlNode *a_node)
{
  if (!a_node) throw_nullpointer(__FUNCTION__);

  return isElementType(a_node, XML_MODE);
}

// ---------------------------------------------------------------------------
bool isEanElement(xmlNode *a_node)
{
  if (!a_node) throw_nullpointer(__FUNCTION__);

  return isElementType(a_node, XML_TARGET_EAN);
}

// ---------------------------------------------------------------------------
bool isScriptElement(xmlNode *a_node)
{
  if (!a_node) throw_nullpointer(__FUNCTION__);

  return isElementType(a_node, XML_SCRIPT);
}

// ---------------------------------------------------------------------------
bool isActionBlock(xmlNode *a_node)
{
  if (!a_node) throw_nullpointer(__FUNCTION__);

  return isElementType(a_node, XML_ACTIONS);
}

// ---------------------------------------------------------------------------
bool isActionElement(xmlNode *a_node)
{
  if (!a_node) throw_nullpointer(__FUNCTION__);

  return isElementType(a_node, ActionTypeStrings, ActionTypeLength);
}

// ---------------------------------------------------------------------------
bool isExpressionElement(xmlNode *a_node)
{
  if (!a_node) throw_nullpointer(__FUNCTION__);

  return isElementType(a_node, XML_EXPRESSION);
}

// ---------------------------------------------------------------------------
bool isAfterburnerElement(xmlNode *a_node)
{
  if (!a_node) throw_nullpointer(__FUNCTION__);

  return isElementType(a_node, XML_AFTERBURNER);
}

// ---------------------------------------------------------------------------
bool isMessageElement(xmlNode *a_node)
{
  if (!a_node) throw_nullpointer(__FUNCTION__);

  return isElementType(a_node, XML_MSG);
}

// ---------------------------------------------------------------------------
bool isTransmitListElement(xmlNode *a_node)
{
  if (!a_node) throw_nullpointer(__FUNCTION__);

  return isElementType(a_node, XML_TRANSMIT_LIST);
}

// ---------------------------------------------------------------------------
bool isTransmitMessageElement(xmlNode *a_node)
{
  if (!a_node) throw_nullpointer(__FUNCTION__);

  return isElementType(a_node, XML_TRANSMIT_MSG);
}

// ---------------------------------------------------------------------------
bool isFilterChannelElement(xmlNode *a_node)
{
  if (!a_node) throw_nullpointer(__FUNCTION__);

  return isElementType(a_node, XML_FILTER_CHAN);
}

// ---------------------------------------------------------------------------
bool isBusParameterElement(xmlNode *a_node)
{
  if (!a_node) throw_nullpointer(__FUNCTION__);

  return isElementType(a_node, XML_BUSPARAM);
}

// ---------------------------------------------------------------------------
xmlNode * findElementTriggers (xmlNode *root_node)
{
  xmlNode *cur_node = NULL;

  if (!root_node) throw_nullpointer(__FUNCTION__);

  // Parse the TRIGGERBLOCK
  cur_node = findFirstElementNode(root_node, XML_TRIGGER_BLOCK);

  // TRIGGERBLOCK is mandatory
  if (!cur_node) {
      throw_element_not_found(root_node, XML_TRIGGER_BLOCK);
  }

  return findFirstElementNode(cur_node, XML_TRIGGERS);
}

// ---------------------------------------------------------------------------
xmlNode * findElementStatements (xmlNode *root_node)
{
  xmlNode *cur_node = NULL;

  if (!root_node) throw_nullpointer(__FUNCTION__);

  // Parse the TRIGGERBLOCK
  cur_node = findFirstElementNode(root_node, XML_TRIGGER_BLOCK);

  // TRIGGERBLOCK is mandatory
  if (!cur_node) {
      throw_element_not_found(root_node, XML_TRIGGER_BLOCK);
  }

  return findFirstElementNode(cur_node, XML_STATEMENT_BLOCK);
}

// ===========================================================================
// Debug/Helper functions
// ===========================================================================

// ---------------------------------------------------------------------------
// Prints a block header in param.lif
void print_block (BlockHead *ph)
{
  unsigned int i;

  if (!ph) {
    PRINTF(("print_block: BlockHead pointer is NULL.\n"));
    return;
  }

  if (ph->id == BLOCK_ID_END) {
    PRINTF(("%-16s     Checksum: 0x%x\n", "BLOCK_ID_END", ph->len));
    return;
  }

  for (i=0; i < BlockLength; i++) {
    if (ph->id == (unsigned int)BlockValues[i]) {
      PRINTF(("%-20s\t %5u bytes Id=%u.\n", BlockStrings[i], ph->len, ph->id));
      return;
    }
  }
  PRINTF(("Warning: print_block: Unknown block %d contains %d bytes.\n", ph->id, ph->len));

}

// ---------------------------------------------------------------------------
// Prints the names of the all the xml elements that are siblings or children
// of a given xml node.
void print_element_names(xmlNode * a_node)
{
    xmlNode *cur_node = NULL;

    for (cur_node = a_node; cur_node; cur_node = cur_node->next) {
        if (cur_node->type == XML_ELEMENT_NODE) {
            PRINTF(("node type: Element, name: %s\n", cur_node->name));
        }

        print_element_names(cur_node->children);
    }
}

// ---------------------------------------------------------------------------
// Prints the names and values of a given xml node.
void print_prop(xmlNode *a_node) {
  xmlAttrPtr propPtr;
  xmlChar *key, *value;

  if (!a_node) {
    PRINTF(("print_prop: Node pointer is NULL.\n"));
    return;
  }
  PRINTF(("'%s' contains ",a_node->name));
  propPtr = a_node->properties;
  while( propPtr ) {
    key = (xmlChar*) propPtr->name;
    value = xmlGetProp( a_node, key);
    PRINTF(("(%s=%s)", key, value));
    xmlFree(value);
    propPtr = propPtr->next;
  }
  PRINTF(("\n"));
}

// ---------------------------------------------------------------------------
// Prints the content of a given xml node.
void print_element_content(xmlNode *a_node) {
  xmlChar *attr_string = NULL;

  if (!a_node) {
    PRINTF(("print_element_content: Node pointer is NULL.\n"));
    return;
  }

  attr_string = xmlNodeGetContent(a_node);
  if (!attr_string || !strlen((const char*) attr_string)) {
    PRINTF(("'%s' contains <null>\n",a_node->name));
  } else {
    PRINTF(("'%s' contains '%s'\n", a_node->name, attr_string));
    xmlFree(attr_string);
  }
}


// ---------------------------------------------------------------------------
bool getFileSize(const char *filename, size_t *size)
{
  FILE* fp = NULL;
  bool exists = false;

  if (!filename || !size) throw_nullpointer(__FUNCTION__);

  fp = fopen(filename, "rb");
  if (fp) {
    os_fseek(fp, 0L, SEEK_END);
    *size = (int)os_ftell(fp);
    fclose(fp);
    exists = true;
  }
  return exists;
}

// ---------------------------------------------------------------------------
size_t copyFileToBuffer(const char *filename, void *buf, size_t len)
{
  FILE *fp = NULL;
  size_t bytesRead;
  memset(buf, 0, len);
  fp = fopen(filename, "rb");
  if(!fp) {
    return 0;
  }
  bytesRead = fread(buf, 1, len, fp);
  PRINTF(("Read %u bytes from '%s' to buffer with %u bytes.\n", (unsigned int)bytesRead, filename, (unsigned int)len));
  fclose(fp);
  return bytesRead;
}

// ---------------------------------------------------------------------------
std::string getScriptName(xmlNode *script_node)
{
  xmlNode *script_node_child = NULL;
  std::string fname, path, filename;
  size_t len = 0;

  // Script path is optional
  script_node_child = findFirstElementNode(script_node, XML_SCRIPT_PATH);
  if (script_node_child) {
    len = getElementString(script_node_child, path);
  }

  if (len > 0) {
    filename = path;
    if ( path.at(len-1) != '/' ) filename += "/";
  }

  // Script name is mandatory
  script_node_child = findFirstElementNode(script_node, XML_SCRIPT_FILE);
  if (!script_node_child) throw_element_not_found (script_node, XML_SCRIPT_FILE);
  getElementString(script_node_child, fname);
  filename += fname;

  return filename;
}

// ---------------------------------------------------------------------------
// Returns int converted to string.
std::string intToString(int a) {
  std::stringstream ss;
  ss << a;
  std::string str = ss.str();
  return str;
}

// ---------------------------------------------------------------------------
static int isspaceUTF8 ( int c )
{
  // locale set to 'en_US.UTF-8' at DLL_PROCESS_ATTACH
  std::locale loc;
  char d = (char) c;
  return std::isspace(d, loc);
}

// ---------------------------------------------------------------------------
// Removes whitespace from str.
void clearws(std::string& str) {
  str.erase(remove_if(str.begin(), str.end(), isspaceUTF8), str.end());
};

// ---------------------------------------------------------------------------
// Replaces all occurences of from in str with to.
void replaceAll(std::string& str, const std::string& from,
                const std::string& to) {
  size_t start_pos = 0;

  if (from.empty()) return;

  while((start_pos = str.find(from, start_pos)) != std::string::npos) {
    str.replace(start_pos, from.length(), to);
    start_pos += to.length();
  }
}

// ---------------------------------------------------------------------------
// Like replaceAll, but only replaces occurences of from in str with to if the
// found occurence is surrounded by neighbours.
void replaceAllWithNeighbourConditions(std::string& str,
                                       const std::string& from,
                                       const std::string& to,
                                       const std::string& neighbours) {
  size_t start_pos = 1;

  if (from.empty()) return;

  while((start_pos = str.find(from, start_pos)) != std::string::npos) {
    if ( (neighbours.find(str.at(start_pos - 1)) != std::string::npos) &&
         (neighbours.find(str.at(start_pos + from.length())) != std::string::npos) ) {
      str.replace(start_pos, from.length(), to);
    }
    start_pos += to.length();
  }
}

// ----------------------------------------------------------------------------
void tokenizeStr(const std::string &str,
                 std::vector<std::string> &tokens)
{
  std::istringstream iss(str);
  std::string token;
  while (iss >> token) {
    tokens.push_back(token);
  }
}

// ----------------------------------------------------------------------------
bool isSupportedXml(unsigned int major, unsigned int minor)
{
  if (KVASER_XML_VERSION_ELEMENT_MAJOR == major &&
      KVASER_XML_VERSION_ELEMENT_MINOR == minor) {
    return true;
  }
  return false;
}

// ----------------------------------------------------------------------------
bool isSupportedBinary(unsigned int major, unsigned int minor)
{
  if (KVASER_BINARY_VERSION_ELEMENT_MAJOR == major &&
      KVASER_BINARY_VERSION_ELEMENT_MINOR == minor) {
    return true;
  }

  if (KVASER_BINARY_VERSION_ELEMENT_MAJOR_OLD == major &&
      KVASER_BINARY_VERSION_ELEMENT_MINOR_OLD == minor) {
    return true;
  }
  return false;
}


// ---------------------------------------------------------------------------
bool isCanFdBusParameters (xmlNode * bus_node)
{
  if (!bus_node) throw_nullpointer(__FUNCTION__);
  if ( hasAttribute(bus_node, XML_BUSPARAM_ATTR_BITRATE_BRS) ) return true;
  if ( hasAttribute(bus_node, XML_BUSPARAM_ATTR_TSEG1_BRS) ) return true;
  if ( hasAttribute(bus_node, XML_BUSPARAM_ATTR_TSEG2_BRS) ) return true;
  if ( hasAttribute(bus_node, XML_BUSPARAM_ATTR_SJW_BRS) ) return true;
  if ( hasAttribute(bus_node, XML_BUSPARAM_ATTR_ISO_BRS) ) return true;
  return false;
}

// ---------------------------------------------------------------------------
int dlcToBytes(int dlc, bool isCanFd)
{
  int bytes;

  dlc &= 0xf;

  if (isCanFd) {
    switch (dlc) {
      case  9: bytes = 12;  break;
      case 10: bytes = 16;  break;
      case 11: bytes = 20;  break;
      case 12: bytes = 24;  break;
      case 13: bytes = 32;  break;
      case 14: bytes = 48;  break;
      case 15: bytes = 64;  break;
      default: bytes = dlc; break;
    }
  }
  else {
    dlc > 8 ? bytes = 8 : bytes = dlc;
  }

  return bytes;
}

static const std::string OP_AND = "AND";
static const std::string OP_OR = "OR";

bool isOp(std::string name) {
  std::transform(name.begin(), name.end(), name.begin(), ::toupper);
  return (OP_AND == name || OP_OR == name);
}
