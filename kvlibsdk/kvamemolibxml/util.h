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

#include <string>
#include <vector>
#include <libxml/tree.h>
#include "logger_config_file.h"

xmlNode *findFirstElementNode (xmlNode *root_node, const char *element_name);

xmlNode *findNextElementNode (xmlNode *cur_node, const char *element_name);

bool isElementType (xmlNode *a_node, char const * const string_list[], unsigned int len);

bool isElementType(xmlNode *a_node, char const * name);

bool isElementActive (xmlNode *a_node);

bool isTriggerElement(xmlNode *a_node);

bool isStatementElement(xmlNode *a_node);

bool isFilterElement(xmlNode *a_node);

bool isModeElement(xmlNode *a_node);

bool isEanElement(xmlNode *a_node);

bool isScriptElement(xmlNode *a_node);

bool isActionBlock(xmlNode *a_node);

bool isActionElement(xmlNode *a_node);

bool isExpressionElement(xmlNode *a_node);

bool isAfterburnerElement(xmlNode *a_node);

bool isMessageElement(xmlNode *a_node);

bool isTransmitMessageElement(xmlNode *a_node);

bool isTransmitListElement(xmlNode *a_node);

bool isFilterChannelElement(xmlNode *a_node);

bool isBusParameterElement(xmlNode *a_node);

bool isCanFdBusParameters (xmlNode * bus_node);

bool hasAttribute(xmlNode *a_node, const char *attr_name);

xmlNode * findElementTriggers (xmlNode *root_node);

xmlNode * findElementStatements (xmlNode *root_node);

bool getFileSize (const char *filename, size_t *size);

size_t copyFileToBuffer (const char *filename, void *buf, size_t len);

std::string getScriptName(xmlNode *script_node);

std::string intToString(int a);

void clearws(std::string& str);

void replaceAll(std::string& str, const std::string& from, const std::string& to);

void replaceAllWithNeighbourConditions(std::string& str, const std::string& from, const std::string& to, const std::string& neighbours);

void tokenizeStr(const std::string &str, std::vector<std::string> &tokens);

bool isSupportedXml(unsigned int major, unsigned int minor);

bool isSupportedBinary(unsigned int major, unsigned int minor);

int dlcToBytes(int dlc, bool isCanFd);

bool isOp(std::string name);

bool getVersion (xmlNode *root_node, const char *name, unsigned int *major, unsigned int *minor);

// ===========================================================================
// Debug/Helper functions
// ===========================================================================

void print_block (BlockHead *ph);

void print_element_names (xmlNode * a_node);

void print_prop (xmlNode *a_node);

void print_element_content (xmlNode *a_node);
