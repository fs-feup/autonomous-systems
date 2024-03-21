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
#ifndef KV_PARSER_H__
#define KV_PARSER_H__

#include <vector>
#include <string>

//----------------------------------------------------
//
// ERRORS
//
#define ERR_OK                    0
#define ERR_INVALID_HANDLE       -99
#define ERR_POSTFIX_UNAVAILABLE  -98
#define ERR_UNIDENTIFIED_SYMBOL  -1
#define ERR_EXPRESSION           -2
#define ERR_UNMATCHED_LEFT_PAR   -3
#define ERR_UNMATCHED_RIGHT_PAR  -4
#define ERR_MUST_BE_FOLLOWD_BY   -5
#define ERR_EXTERNAL             -6
#define ERR_UNEXPECTED_STR       -7
#define ERR_CANNOT_START_WITH    -8

//---------------------------------------------------
// DATATYPES

// handle type
typedef void* kv_parser_handle;

// type to parse for
enum struct_type { Memorator, Memorator2};

// Symbol type
enum symbol_type { Variable,Operator,Bracket,Error };

// symbol struct
struct symbol
{
  std::string  m_name;
  symbol_type  m_type;
  bool         valid;
};

// parse struct
typedef struct t_parser_data
{
  struct_type          type;
  bool                 error;
  std::string          *error_description;
  bool                 result;

  // hold the expressions and postfix expression
  std::vector<symbol>  *expression_vec;
  std::vector<symbol>  *postfix_vec;

  // to check if there is a postfix expression
  bool                 postfix_available;

  // iterator for getfirstvar and getnextvar
  std::vector<symbol>::iterator variable_iterator;

} t_parser_data;


//----------------------------------------------------------------------------------
//
// Kvaser Memorator Trigger Expression Parsing functions
//
//----------------------------------------------------------------------------------

#define T_ID      100
#define T_OP_AND  110
#define T_OP_OR   111
#define T_LPAREN  120
#define T_RPAREN  121
#define T_EOL     130
#define T_ERROR   140

#define ERR_UNBALANCED_PAREN      100
#define ERR_EMPTY_EXPR            101
#define ERR_ILLEGAL_CHARACTER     102
#define ERR_EXTRA_OPERATOR        103
#define ERR_MISSING_OPERATOR      104
#define ERR_EXTRA_RPAREN          105
#define ERR_UNKNOWN_TOKEN         106
#define ERR_TRAILING_GARBAGE      107

#endif

