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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <clocale>
#include <libxml/parser.h>
#include <libxml/tree.h>
#include <libxml/encoding.h>
#include <libxml/xmlwriter.h>

#include "kvaMemoLibXML.h"
#include "kvaMemoLibXMLConst.h"
#include "errorhandler.h"
#include "os_util.h"
#include "validation.h"
#include "canlib_version.h"
#include "kvdebug.h"

#define XML_BUFFER_SIZE (10*8192)
#define NEW_FILENAME_SUFFIX "_new"
#define SCRIPT_DIR "script"

static int ParserOptions = XML_PARSE_NOERROR;
static char xmlbuffer[XML_BUFFER_SIZE];

// ---------------------------------------------------------------------------
FILE* utf_fopen(const char* filename, const char* type)
{
	//return fopen(filename, type);
	PRINTF((filename));
	return fopen(filename, type);
}

// ---------------------------------------------------------------------------
static int fileToBuffer(const char *fname, void * buf, unsigned int len)
{
  FILE *fp = NULL;
  memset(buf, 0, sizeof(len));
  fp = fopen(fname, "rb");
  if (!fp) {
    return 1;
  }
  if (!fread(buf, 1, len, fp)) {
    return 1;
  }
  fclose(fp);
  return 0;
}

// ---------------------------------------------------------------------------
static int getScriptPath(const char * filename, char * scriptpath, unsigned int len)
{
  std::string tmp = std::string(filename);
  std::string drive;
  std::string dir;
  std::string base;
  std::string ext;

  if (!os_splitpath(tmp, drive, dir, base, ext)) {
    return 1;
  }

  tmp.clear();
  if (!os_makepath(tmp, drive, dir, SCRIPT_DIR,"")) {
    return 1;
  }

  if (tmp.size() > len) {
    return 1;
  }
  strncpy(scriptpath, tmp.c_str(), len);
  return 0;
}

// ---------------------------------------------------------------------------
KvaXmlStatus WINAPI kvaFileToXml(const char * parfile, const char * xmlfile)
{
  KvaXmlStatus status = KvaXmlStatusOK;
  char *buffer = NULL;
  char scriptpath[1024] = {0};
  unsigned int xmllen = sizeof(xmlbuffer);
  long version = 0;
  FILE *fp = NULL;

  buffer = (char*) malloc(PARAM_FILE_SIZE);
  if (!buffer) {
    set_error(KvaXmlStatusFail, "ERROR: Could not allocate memory in kvaFileToXML.\n");
    return get_error_status();
  }

  // Read param.lif into a buffer
  if ( fileToBuffer(parfile, buffer, PARAM_FILE_SIZE) ) {
    set_error(KvaXmlStatusFail ,"ERROR: Could not load file '%s'.\n", parfile);
    free(buffer);
    return get_error_status();
  }

  // Create a directory for the extracted script files
  getScriptPath(parfile, scriptpath, sizeof(scriptpath) - 1);
  os_mkdir(scriptpath);

  // Do the actual conversion
  status = kvaBufferToXml(buffer, PARAM_FILE_SIZE, xmlbuffer, &xmllen, &version, scriptpath);
  free(buffer);
  if (status) {
    return status;
  }

  // Write the resulting XML buffer to file
  //fp = fopen(xmlfile, "wb");
  fp = utf_fopen(xmlfile, "wb");
  if (!fp) {
    set_error(KvaXmlStatusFail, "ERROR: Could not write to file '%s'.\n", xmlfile);
    return get_error_status();
  }
  fwrite(xmlbuffer, 1, xmllen, fp);
  fclose(fp);
  return status;
}

// ----------------------------------------------------------------------------
static void free_and_cleanup(xmlDocPtr doc, xmlParserCtxtPtr ctxt)
{
  if (doc) xmlFreeDoc(doc);
  if (ctxt) xmlFreeParserCtxt(ctxt);
  xmlCleanupParser();
}

// ----------------------------------------------------------------------------
KvaXmlStatus WINAPI kvaXmlInitialize (void)
{
  PRINTF(("API->kvaXmlInitialize"));
  std::setlocale(LC_ALL, "en_US.UTF-8");

   /*
     * This initializes the library and check potential ABI mismatches
     * between the version it was compiled for and the actual shared
     * library used.
     */
    LIBXML_TEST_VERSION
    set_error_status(KvaXmlStatusOK);
    ParserOptions =  0; //XML_PARSE_NOERROR;
    return KvaXmlStatusOK;
}

// ----------------------------------------------------------------------------
KvaXmlStatus WINAPI kvaXmlGetLastError (char* buf, unsigned int len, KvaXmlStatus* err)
{
  PRINTF(("API->kvaXmlGetLastError"));
  if (!buf || !err) return KvaXmlStatusFail;

  *err = get_error_status();

  const char *msg = get_error_message();

  if (strlen(msg) < len) {
    strcpy(buf,msg);
    return KvaXmlStatusOK;
  }
  return KvaXmlStatusFail;
}

// ----------------------------------------------------------------------------
KvaXmlStatus WINAPI kvaXmlDebugOutput (int on)
{
  if (on) {
    ParserOptions &= ~XML_PARSE_NOERROR;
  } else {
    ParserOptions |= XML_PARSE_NOERROR;
  }
  return KvaXmlStatusOK;
}

// ----------------------------------------------------------------------------
static KvaXmlStatus parseXmlAndCreateBinaryBuffer(xmlDocPtr &doc,
                                                  xmlParserCtxtPtr &ctxt,
                                                  unsigned char *&buf,
                                                  size_t &buflen)
{
  xmlNode *root_element = NULL;
  xmlErrorPtr xml_err = NULL;
  XmlStruct xr;

  xml_err = xmlGetLastError();
  if (!doc || xml_err) {
    set_error(KvaXmlStatusFail, "xmlCtxtReadFile: Error close to XML line: %lu:\n  %s\n", xml_err->line, xml_err->message);
    if (xml_err->domain == XML_FROM_PARSER) set_error_status(KvaXmlStatusERR_XML_PARSER);
    if (xml_err->domain == XML_FROM_DTD) set_error_status(KvaXmlStatusERR_DTD_VALIDATION);
    free_and_cleanup(doc, ctxt);
    return get_error_status();
  }
  if (!ctxt->valid) {
    if (xml_err) {
      set_error(KvaXmlStatusERR_DTD_VALIDATION, "Ctxt->valid: Error close to XML line %lu:\n  %s\n", xml_err->line, xml_err->message);
    }
    set_error_status(KvaXmlStatusERR_DTD_VALIDATION);
    free_and_cleanup(doc, ctxt);
    return get_error_status();
  }

  root_element = (xmlDocGetRootElement(doc));

  //        Do we really need to bail out for every error?

  // Parse XML and create internal data structure
  try {
    xr.parseXml(root_element);
  }
  catch (KvaXmlStatus err) {
    set_error_status(err);
    free_and_cleanup(doc, ctxt);
    return get_error_status();
  }

  // Debug print internal data structure
#ifdef DEBUG
  xr.print();
#endif

  buf = (unsigned char *) malloc(PARAM_FILE_SIZE);
  if (!(buf)) {
    set_error(KvaXmlStatusFail, "Could not allocate memory in kvaXmlToFile().");
    free_and_cleanup(doc, ctxt);
    return get_error_status();
  }

  memset(buf, 0, PARAM_FILE_SIZE);

  try {
    buflen = xr.createBinary(buf);
  }
  catch (KvaXmlStatus err) {
    set_error_status(err);
    free_and_cleanup(doc, ctxt);
    free(buf);
    return get_error_status();
  }

  return get_error_status();
}

// ----------------------------------------------------------------------------
KvaXmlStatus WINAPI kvaXmlToBuffer (const char *xmlbuf, unsigned int xmllen, char *outbuf, unsigned int *outlen, long *version)
{
  PRINTF(("API->kvaXmlToBuffer"));
  xmlDocPtr doc; /* the resulting document tree */
  xmlParserCtxtPtr ctxt; /* the parser context */
  unsigned char *buf = NULL;
  size_t buflen = 0;

  set_error_status(KvaXmlStatusOK);

  if (!xmlbuf || !xmllen || !outbuf || !outlen|| !version) {
    set_error(KvaXmlStatusFail, "Error: kvaXmlToBuffer(0x%lx, %lu, 0x%lx, %lu, %d) called with invalid arguments.\n",
              xmlbuf,xmllen,outbuf,outlen,version);
    return get_error_status();
  }

  // Create a parser context
  ctxt = xmlNewParserCtxt();
  if (ctxt == NULL) {
    set_error(KvaXmlStatusFail,  "Failed to allocate XML parser context.\n");
    return get_error_status();
  }

  // Count the lines to create nice error messages
  xmlLineNumbersDefault(1);
  initGenericErrorDefaultFunc(NULL);

  // Parse the file
  doc = xmlCtxtReadMemory(ctxt, xmlbuf, xmllen, "kvaser.xml", NULL, ParserOptions);

  if (parseXmlAndCreateBinaryBuffer(doc, ctxt, buf, buflen) != KvaXmlStatusOK) {
    return get_error_status();
  }

  memset(outbuf, 0, *outlen);

  if (*outlen >= buflen) {
    memcpy(outbuf, buf, buflen);
    *outlen = (unsigned int) buflen;
  } else {
    set_error(KvaXmlStatusFail,
              "kvaXmlToBuffer: Output buffer is too small. Needed %lu, got %u.",
              buflen, outlen);
    free_and_cleanup(doc, ctxt);
    free(buf);
    return get_error_status();
  }

  free_and_cleanup(doc, ctxt);
  free(buf);

  return get_error_status();
}

// ----------------------------------------------------------------------------
KvaXmlStatus WINAPI kvaXmlToFile (const char *infile, const char *outfile)
{
  PRINTF(("API->kvaXmlToFile"));
  xmlDocPtr doc; /* the resulting document tree */
  xmlParserCtxtPtr ctxt; /* the parser context */
  unsigned char *buf = NULL;
  size_t buflen = 0;
  FILE *fp;

  set_error_status(KvaXmlStatusOK);

  if (!infile || !outfile) {
    set_error(KvaXmlStatusFail, "Error: kvaXmlToFile(0x%x, 0x%x) called with invalid arguments.\n",
    infile, outfile);
    return get_error_status();
  }

  // Create a parser context
  ctxt = xmlNewParserCtxt();
  if (ctxt == NULL) {
    set_error(KvaXmlStatusFail, "Failed to allocate XML parser context.\n");
    return get_error_status();
  }

  // Count the lines to create nice error messages
  xmlLineNumbersDefault(1);
  initGenericErrorDefaultFunc(NULL);

  // Parse the file
  doc = xmlCtxtReadFile(ctxt, infile, NULL, ParserOptions);

  if (parseXmlAndCreateBinaryBuffer(doc, ctxt, buf, buflen) != KvaXmlStatusOK) {
    return get_error_status();
  }

  //fp = fopen(outfile, "wb");
  fp = utf_fopen(outfile, "wb");
  if (!fp) {
    set_error(KvaXmlStatusFail, "Error: Could not create output file '%s'.\n", outfile);
    free_and_cleanup(doc, ctxt);
    free(buf);
    return get_error_status();
  }
  fwrite(buf, 1, buflen, fp);
  fclose(fp);

  free(buf);
  free_and_cleanup(doc, ctxt);

  return get_error_status();
}

// ----------------------------------------------------------------------------
KvaXmlStatus WINAPI kvaBufferToXml(const char *inbuf, unsigned int inlen,
                                   char *xmlbuf, unsigned int *xmllen,
                                   long * /*version*/, const char *scriptpath)
{
  PRINTF(("API->kvaBufferToXml"));
  xmlErrorPtr xml_err = NULL;
  xmlTextWriterPtr xml_writer = NULL;
  xmlBufferPtr xml_writer_buf = NULL;
  XmlStruct xr;

  try {

    xr.parseBinary(inbuf, inlen, scriptpath);

#ifdef DEBUG
    xr.print();
#endif

    xml_writer_buf = xmlBufferCreate();
    if (xml_writer_buf == NULL)
      throw_xml_writer_failure ("unable to create XML buffer");

    xml_writer = xmlNewTextWriterMemory(xml_writer_buf, 0);
    if (xml_writer == NULL)
      throw_xml_writer_failure ("unable to create writer memory");

    // Enable indentation in debug mode
    if ( !(ParserOptions & XML_PARSE_NOERROR) ) {
      xmlTextWriterSetIndent(xml_writer, 2);
    }

    if (xmlTextWriterStartDocument(xml_writer, "1.0", NULL, "yes") == -1)
      throw_xml_writer_failure ("unable to start document");

    xr.createXml(xml_writer);

    if (xmlTextWriterEndDocument(xml_writer) == -1) {
      throw_xml_writer_failure("Unable to end XML document");
    }

    if (*xmllen >= xml_writer_buf->use) {
      memcpy(xmlbuf, (const char *) xml_writer_buf->content, xml_writer_buf->use);
      *xmllen = xml_writer_buf->use;
      set_error_status(KvaXmlStatusOK);
    }
    else  {
      throw_xml_writer_failure("Failed to write XML to argument xmlbuf");
    }

    // Parse the file, activating the DTD validation option
    xmlReadMemory(xmlbuf, (int)*xmllen, "kvaser.xml", NULL, ParserOptions);
    xml_err = xmlGetLastError();
    if (xml_err) {
      set_error(KvaXmlStatusFail, "Error close to XML line: %lu:\n  %s\n",xml_err->line, xml_err->message);
      if (xml_err->domain == XML_FROM_PARSER) set_error_status(KvaXmlStatusERR_XML_PARSER);
      if (xml_err->domain == XML_FROM_DTD) set_error_status(KvaXmlStatusERR_DTD_VALIDATION);
      throw(get_error_status());
    }

    xmlFreeTextWriter(xml_writer);
    xmlBufferFree(xml_writer_buf);
    xmlCleanupParser();
  }

  catch (KvaXmlStatus err)
  {
    set_error_status(err);
    if(xml_writer) xmlFreeTextWriter(xml_writer);
    if(xml_writer_buf) xmlBufferFree(xml_writer_buf);
    xmlCleanupParser();
    return get_error_status();

  }
  return get_error_status();
}

// ---------------------------------------------------------------------------
// Validate a buffer with XML settings
KvaXmlStatus WINAPI kvaXmlValidate (const char *xmlbuf, unsigned int xmllen)
{
  PRINTF(("API->kvaXmlValidate"));
  xmlDocPtr doc;
  xmlParserCtxtPtr ctxt;
  xmlNode *root_element = NULL;
  xmlErrorPtr xml_err = NULL;

  set_error_status(KvaXmlStatusOK);

  if (!xmlbuf || !xmllen) {
    set_error(KvaXmlStatusFail, "%s: XML buffer pointer is NULL or buffer size is zero.", __FUNCTION__);
    return get_error_status();
  }

  // Create a parser context
  ctxt = xmlNewParserCtxt();
  if (ctxt == NULL) {
    set_error(KvaXmlStatusFail, "%s: Failed to allocate XML parser context.\n", __FUNCTION__);
    return get_error_status();
  }

  // Count the lines to create nice error messages
  xmlLineNumbersDefault(1);
  initGenericErrorDefaultFunc(NULL);

  // Parse the file
  doc = xmlCtxtReadMemory(ctxt, xmlbuf, xmllen, "kvaser.xml", NULL, ParserOptions);
  xml_err = xmlGetLastError();
  if (!doc || xml_err) {
    set_error(KvaXmlStatusFail, "%s: Error close to XML line: %lu:\n  %s\n", __FUNCTION__, xml_err->line, xml_err->message);
    if (xml_err->domain == XML_FROM_PARSER) set_error_status(KvaXmlStatusERR_XML_PARSER);
    free_and_cleanup(doc, ctxt);
    return get_error_status();
  }

  root_element = xmlDocGetRootElement(doc);

  // Parse XML and create internal data structure
  XmlStruct xr;
  try {
    clearValidationStatus();
    xr.parseXml(root_element);
  }
  catch (KvaXmlStatus err) {
    // Convert errors found during parsing to validation errors.
    set_error_status(err);
    setValidationError(XmlStatusToValidationStatusErr(get_error_status()), get_error_message());

    free_and_cleanup(doc, ctxt);
    return KvaXmlStatusOK;
  }

  try {
    validateDisabledTriggers(xr);
    validateTransmitAndSilentMode(xr);
    validateActiveLogging(xr);
    validateSpecialTriggerCount(xr);
    validateTriggerDiskFull(xr);
    validateNumericalValues(xr);
    validateScripts(xr);

    validateXmlElements(root_element);
  }
  catch (KvaXmlStatus err)
  {
    set_error_status(err);
  }

  free_and_cleanup(doc, ctxt);

  return get_error_status();
}

// ---------------------------------------------------------------------------
// Get the validation warnings. Call after  kvaXmlValidate() until KvaXmlValidationStatusOK
KvaXmlStatus WINAPI kvaXmlGetValidationWarning (KvaXmlValidationStatus *status,  char *buf, unsigned int len)
{
  PRINTF(("API->kvaXmlGetValidationWarning"));
  KvaXmlStatus res = KvaXmlStatusOK;
  if (!status)
    return KvaXmlStatusFail;

  if (buf && len > 0) {
    *status  = getValidationWarning(buf, len);
  }
  else {
    *status  = getValidationWarning(NULL, 0);
  }
  return res;
}

// ---------------------------------------------------------------------------
// Get the validation statuses. Call after  kvaXmlValidate() until KvaXmlValidationStatusOK
KvaXmlStatus WINAPI kvaXmlGetValidationError (KvaXmlValidationStatus *status, char *buf, unsigned int len)
{
  PRINTF(("API->kvaXmlGetValidationError"));
  KvaXmlStatus res = KvaXmlStatusOK;
  if (!status)
    return KvaXmlStatusFail;

  if (buf && len > 0) {
    *status  = getValidationError(buf, len);
  }
  else {
    *status  = getValidationError(NULL, 0);
  }
  return res;
}

// ---------------------------------------------------------------------------
KvaXmlStatus WINAPI kvaXmlGetValidationStatusCount (int *countErr, int *countWarn)
{
  PRINTF(("API->kvaXmlGetValidationStatusCount"));
  set_error_status(KvaXmlStatusOK);
  if (!countErr || !countWarn) {
    set_error(KvaXmlStatusFail, "%s: countErr or countWarn pointer is NULL.", __FUNCTION__);
  }
  else {
    getValidationCount(countErr, countWarn);
  }
  return get_error_status();
}

//------------------------------------------------------------------------------
unsigned short WINAPI kvaXmlGetVersion(void)
{
  PRINTF(("API->kvaXmlGetVersion"));
  return (CANLIB_MAJOR_VERSION << 8) + CANLIB_MINOR_VERSION;
}

//------------------------------------------------------------------------------
KvaXmlStatus WINAPI kvaXmlGetErrorText (KvaXmlStatus status,
                                        char *buf,
                                        unsigned int len)
{
  const char* msg;
  if ( get_error_text(status, msg) ) {
    strncpy(buf, msg, len);
   } else {
       strncpy(buf, "This is not an error code", len);
       buf[len-1] = '\0';
   }
   return KvaXmlStatusOK;
}

//------------------------------------------------------------------------------
KvaXmlStatus WINAPI kvaXmlGetValidationText (KvaXmlValidationStatus status,
                                             char *buf,
                                             unsigned int len)
{
  const char* msg;
  if ( get_error_text(KvaXmlValOffset + status, msg) ) {
    strncpy(buf, msg, len);
   } else {
    strncpy(buf, "This is not an error code", len);
       buf[len-1] = '\0';
   }
   return KvaXmlStatusOK;
}


//===========================================================================
// The DLL Entry Point.
//===========================================================================
