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
** Description: Parse XML and search for ignored elements
** -----------------------------------------------------------------------------
*/

#include <string>
#include <vector>
#include <map>

#include <libxml/parser.h>
#include <libxml/encoding.h>
#include <libxml/xmlwriter.h>

#include "xmlvalidator.h"
#include "kvaMemoLibXMLConst.h"
#include "errorhandler.h"

#include "xmlreference.h"
#include "util.h"
#include "convert.h"
#include "kvdebug.h"

using namespace std;

/* TODO: I don't have the time to do this now, but the idea is to parse the
reference XML file and check if an element is present in the user XML. In that
case, decrease the allowed counter, XML_REF_MAX_ELEMENT_COUNT_ATTR, in the element
by one. If the result would be less than zero, the user has specified too many
elements of that type. Once we have checked all elements of that type, increase
the counter with the number of checked elements. */

/* TODO: Add a validation error code  KvaXmlValidationStatusERR_ELEMENT_COUNT
to notify the user of the duplicated elements. */

/* TODO: It would also be possible to check for ignored attributes by adding them
to to writeKvaserSpecification() and checking them in checkNode(). */

/* TODO: Add API to extract the elementd and a validation error code
KvaXmlValidationStatusWARN_IGNORED_ELEMENTS to notify the user of the ignored
elements */

// ---------------------------------------------------------------------------
// Each element in the reference XML contains an attribute with the maximum
// number of allowed elements or -1 if we don't care.
static void changeCounter(xmlNode *ref_node, int value, std::map<xmlNode*, int>& counter)
{
  string str;
  auto it = counter.find(ref_node);
  if (it != counter.end()) {
    it->second += value;
  }
  else {
    counter.insert( std::pair<xmlNode*, int>(ref_node, value) );
  }
}

// ---------------------------------------------------------------------------
static void increaseCounter(xmlNode *root_node, xmlNode *ref_node, std::map<xmlNode*, int>& counter)
{
  xmlNode *user_child_node;
  for (user_child_node = root_node->children; user_child_node != NULL; user_child_node = user_child_node->next) {
    if ( isElementType(ref_node, (const char*)user_child_node->name) ) {
      changeCounter(user_child_node, 1, counter);
      break;
    }
  }
}

// ---------------------------------------------------------------------------
static void checkNode(xmlNode *reference_node, xmlNode *user_node, std::vector<std::string>& ignored, std::map<xmlNode*, int>& counter)
{
  xmlNode *reference_child_node = NULL;
  xmlNode *user_child_node = NULL;

  if (NULL == reference_node || NULL == user_node) {
    throw_nullpointer(__FUNCTION__);
  }

  PRINTF(("Working on node '%s'", user_node->name));

  // Compare each child element in user XML with the children in reference XML.
  for (user_child_node = user_node->children; user_child_node != NULL; user_child_node = user_child_node->next) {
    bool found = false;
    if (user_child_node->type != XML_ELEMENT_NODE) continue;
    for (reference_child_node = reference_node->children; reference_child_node != NULL; reference_child_node = reference_child_node->next) {
      if ( isElementType(reference_child_node, (const char*)user_child_node->name) ) {
        found = true;
        break;
      }
    }
    if (found) {
      checkNode(reference_child_node, user_child_node, ignored, counter);
      increaseCounter(user_node, reference_child_node,  counter);

    } else {
      xmlChar * path = xmlGetNodePath(user_child_node);
      ignored.push_back((const char*)path);
      xmlFree(path);
    }
  }
}

static void reportBadNode(xmlNode *reference_node, xmlNode *user_node, std::map<xmlNode*, int>& counter, std::vector<std::string>& extra, std::vector<std::string>& missing) {
  xmlNode *reference_child_node = NULL;
  xmlNode *user_child_node = NULL;
  int val;
  string str;

  if (NULL == reference_node || NULL == user_node) {
    throw_nullpointer(__FUNCTION__);
  }

  auto it = counter.find(user_node);

  if (getAttributeString(reference_node, XML_REF_MAX_ELEMENT_COUNT_ATTR, str)) {
    val = std::stoi(str, NULL);
    if ( it != counter.end() && val < it->second) {
      xmlChar * path = xmlGetNodePath(reference_node);
      extra.push_back((const char*)path);
      xmlFree(path);
    }
  }
  str.clear();
  if (getAttributeString(reference_node, XML_REF_MIN_ELEMENT_COUNT_ATTR, str)) {
    val = std::stoi(str, NULL);
    if ( it == counter.end() || val > it->second ) { //note order of evaluation
      xmlChar * path = xmlGetNodePath(reference_node);
      missing.push_back((const char*)path);
      xmlFree(path);
    }
  }
  for (user_child_node = user_node->children; user_child_node != NULL; user_child_node = user_child_node->next) {
    bool found = false;
    if (user_child_node->type != XML_ELEMENT_NODE) continue;
    for (reference_child_node = reference_node->children; reference_child_node != NULL; reference_child_node = reference_child_node->next) {
      if ( isElementType(reference_child_node, (const char*)user_child_node->name) ) {
        found = true;
        break;
      }
    }
    if (found) {
      reportBadNode(user_child_node, reference_child_node, counter, extra, missing);
    }
  }
}

// ---------------------------------------------------------------------------
static void createXmlReferenceTree(xmlDocPtr *doc, xmlParserCtxtPtr *ctxt)
{
  xmlErrorPtr xml_err = NULL;
  xmlTextWriterPtr xml_writer = NULL;
  xmlBufferPtr xml_writer_buf = NULL;

  try {
    xml_writer_buf = xmlBufferCreate();
    if (xml_writer_buf == NULL)
      throw_xml_writer_failure ("Unable to allocate XML buffer.");

    xml_writer = xmlNewTextWriterMemory(xml_writer_buf, 0);
    if (xml_writer == NULL)
      throw_xml_writer_failure ("Unable to create XML writer from buffer.");

    xmlTextWriterSetIndent(xml_writer, 2);

    // Write a the Kvaser specific XML
    writeKvaserSpecification(xml_writer);

    // Create a parser context
    *ctxt = xmlNewParserCtxt();
    if (*ctxt == NULL) {
      throw_xml_writer_failure ("Failed to allocate XML parser context.");
    }

    // Count the lines to create nice error messages
    xmlLineNumbersDefault(1);
    initGenericErrorDefaultFunc(NULL);

    *doc = xmlReadMemory((const char*) xml_writer_buf->content, xml_writer_buf->use, "kvaser.xml", NULL, 0);
    xml_err = xmlGetLastError();
    if (!*doc || xml_err) {
      PRINTF(("xmlReadMemory: Error close to XML line: %d:\n  %s\n", xml_err->line, xml_err->message));
      throw_xml_writer_failure ("xmlReadMemory: Error in reference XML.");
    }

    xmlFreeTextWriter(xml_writer);
    xmlBufferFree(xml_writer_buf);

  }
   catch (KvaXmlStatus err)
  {
    set_error_status(err);
    if (xml_writer) xmlFreeTextWriter(xml_writer);
    if (xml_writer_buf) xmlBufferFree(xml_writer_buf);
    if (*doc) xmlFreeDoc(*doc);
    if (*ctxt) xmlFreeParserCtxt(*ctxt);
    xmlCleanupParser();
    throw_xml_writer_failure ("createXmlReferenceTree: Could not create XML referece tree.");
  }
}

void checkXmlContent(xmlNode *root_node, std::vector<std::string>& ignored, std::vector<std::string>& extra, std::vector<std::string>& missing)
{
  xmlDocPtr doc = NULL;
  xmlParserCtxtPtr ctxt = NULL;
  xmlNode * ref_root_node = NULL;
  std::map<xmlNode*, int> counter;

    // Create an XML tree with Kvaser specific XML
    createXmlReferenceTree(&doc, &ctxt);
    ref_root_node = xmlDocGetRootElement(doc);
    // Check each element in the user XML vs reference XML
    checkNode(ref_root_node, root_node, ignored, counter);
    reportBadNode(ref_root_node, root_node, counter, extra, missing);
    // Free the created contexts
    if (doc) xmlFreeDoc(doc);
    if (ctxt) xmlFreeParserCtxt(ctxt);
    xmlCleanupParser();
}
