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

#include "kvaMemoLibXML.h"
#include <stdarg.h>
#include <libxml/tree.h>
#include <string>

void set_error_status (const KvaXmlStatus status);

KvaXmlStatus get_error_status (void);

const char *  get_error_message (void);

void set_error_message ( const char * format, ... );

void set_error ( const KvaXmlStatus status, const char * format, ... );

void set_error_and_throw ( const KvaXmlStatus status, const char * format, ... );

void throw_nullpointer (const char* fcn_name);

void throw_general_failure (const char* err_msg);

void throw_attribute_not_found (xmlNode *a_node, const char* attr_name);

void throw_attribute_value (xmlNode *a_node, const char* attr_name, xmlChar *attr_value);

void throw_element_value (xmlNode *a_node, xmlChar *attr_value);

void throw_element_not_found (xmlNode *a_node, const char* element_name);

void throw_wrong_element_count (xmlNode *a_node, const char* element_name,
                                int count, int min, int max);

void throw_wrong_element_order (xmlNode *a_node, const char* first,
                                const char* second, const char* parent);

void throw_value_range (xmlNode *a_node, const char* name, const int value,
                        const int lower_limit, const int upper_limit);

void throw_value_unique (xmlNode *a_node, const char* name, const int value);

void throw_string_unique (xmlNode *a_node, const char* name, std::string& str);

void throw_value_consecutive (xmlNode *a_node, const char* name);

void throw_invalid_expression (xmlNode *a_node);

void warning_element_unknown (xmlNode *a_node);

void throw_xml_writer_failure (const char* err_msg);

void throw_xml_writer_element_failure (const char* element);

void throw_xml_writer_attribute_failure (const char* element, const char* attribute);

void throw_xml_writer_comment_failure (const char* comment);

#define KvaXmlValOffset -1000
bool get_error_text (int code, const char* &msg);
