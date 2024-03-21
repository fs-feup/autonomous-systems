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

#  ifndef TRUE
#    define TRUE true
#    define FALSE false
#  endif

#include <cstdio>
#include <vector>
#include <stack>
#include <string>
#include <cstring>
#include "kvdebug.h"
#include "errorhandler.h"
#include "kv_parser.h"


//----------------------------------------------
// GLOBAL VARIABLES
std::vector<t_parser_data> parser_list;


//=========================================================================
// Return true if c is a letter or digit
bool IsLetterOrDigit(char c)
{
  if (((c>='0') && (c<='9')) ||
      ((c>='A') && (c<='Z')) ||
      ((c>='a') && (c<='z')) ||
      (c=='_') ||
      (c=='-'))
  {
    return TRUE;
  }
  return FALSE;
}


//=========================================================================
KvParseHandle* WINAPI kvaToolsParseCreate(void)
{
  return (KvParseHandle*) calloc(1, sizeof(KvParseHandle));
}


//=========================================================================
void WINAPI kvaToolsParseDestroy(KvParseHandle *h)
{
  Token *t, *n;

  if (!h) return;
  t = h->next;

  while (t) {
    n = t->next;
    if (t->name) free(t->name);
    free(t);
    t = n;
  }
  free(h);
}


//=========================================================================
void WINAPI kvaToolsExprGetErrorString(int errCode, char *s, size_t bufsiz)
{
  const char *err;

  if (!s || bufsiz == 0) return;

  switch (errCode) {
    case ERR_UNBALANCED_PAREN:  err = "Unbalanced paranthesis"; break;
    case ERR_EMPTY_EXPR:        err = "Empty expression"; break;
    case ERR_ILLEGAL_CHARACTER: err = "Illegal character"; break;
    case ERR_EXTRA_OPERATOR:    err = "Superfluous operator"; break;
    case ERR_MISSING_OPERATOR:  err = "Missing operator"; break;
    case ERR_EXTRA_RPAREN:      err = "Extra right parenthesis"; break;
    case ERR_UNKNOWN_TOKEN:     err = "Unknown token"; break;
    case ERR_TRAILING_GARBAGE:  err = "Trailing garbage"; break;
    default:
      err = "";
      break;
  }
  strncpy(s, err, bufsiz);
  s[bufsiz-1] = '\0';
}


//=========================================================================
static Token* allocate_token(KvParseHandle *h, int type)
{
  Token *t = (Token*) calloc(1, sizeof(Token));
  t->type = type;
  t->next = h->next;
  h->next = t;
  return t;
}


//=========================================================================
static int next_token(KvParseHandle *h, char *s, Token **t, int *linepos)
{
  int state = 0;
  int p = *linepos;
  int id_start = 0;
  int id_len = 0;
  int ready = FALSE;
  Token *tok;

  PRINTF(("next_token: '%s' %d (%p(\n", s, *linepos, linepos));

  while (!ready) {

    int advance_pos;
    advance_pos = TRUE;

    switch (state) {
      case 0:
        if (IsLetterOrDigit(s[p])) {
          // Start of identifier
          id_start = p;
          id_len = 1;
          state = 1;
        }
        else if (s[p] == '&') {
          tok = allocate_token(h, T_OP_AND);
          tok->start_pos = tok->end_pos = p;
          ready = TRUE;
        }
        else if (s[p] == '|') {
          tok = allocate_token(h, T_OP_OR);
          tok->start_pos = tok->end_pos = p;
          ready = TRUE;
        }
        else if (s[p] == '(') {
          tok = allocate_token(h, T_LPAREN);
          tok->start_pos = tok->end_pos = p;
          ready = TRUE;
        }
        else if (s[p] == ')') {
          tok = allocate_token(h, T_RPAREN);
          tok->start_pos = tok->end_pos = p;
          ready = TRUE;
        }
        else if (s[p] == '\0') {
          tok = allocate_token(h, T_EOL);
          tok->start_pos = tok->end_pos = p;
          ready = TRUE;
          advance_pos = FALSE;
        }
        else if (isspace(s[p])) {
          // Do nothing
        }
        else {
          tok = allocate_token(h, T_ERROR);
          tok->start_pos = tok->end_pos = p;
          tok->errCode = ERR_ILLEGAL_CHARACTER;
          ready = TRUE;
        }
        break;

      case 1:
        // Munch identifier
        if (IsLetterOrDigit(s[p])) {
          id_len++;
        } else {
          tok = allocate_token(h, T_ID);
          tok->name = (char*) calloc(id_len+1, 1);
          strncpy(tok->name, &s[id_start], id_len);
          tok->start_pos = id_start;
          tok->end_pos = id_start + id_len;
          ready = TRUE;
          advance_pos = FALSE;
        }
        break;
    }
    if (advance_pos) p++;
  }

  *linepos = p;
  *t = tok;
  return tok->type;
}


//=========================================================================
int peek_token(KvParseHandle *h, char *s, Token **t, int *linepos)
{
  int p = *linepos;
  return next_token(h, s, t, &p);
}


//=========================================================================
Token* allocate_error_token(KvParseHandle *h, int errCode, int pos)
{
  Token *tok;

  tok = allocate_token(h, T_ERROR);
  tok->errCode = errCode;
  tok->start_pos = pos;
  // field.
  return tok;
}


//=========================================================================
/*
 * Simplistic parser that handles a grammar like this:
 * expr : <empty>
 *      | <ident>
 *      | ( expr )
 *      | expr <op> expr
 *      ;
 *
 * All operators are assumed to have the same priority. If we need
 * different priorities in the future we should probably go for a Bison
 * parser instead..
 */

Token* parse(KvParseHandle *h, char *s, int *linepos)
{
  Token *t, *tmp;
  Token *left = NULL;
  Token *right = NULL;
  Token *op = NULL;
  Token *result;
  int ready = FALSE;
  int state = 0;
  int toktyp;

  result = NULL;
  PRINTF(("parse '%s' %d\n", s, *linepos));

  while (!ready) {
    switch (state) {
      case 0:
        // Expect left-side expression
        toktyp = next_token(h, s, &t, linepos);
        switch (toktyp) {
          case T_LPAREN:
            //free_token(t);
            t = parse(h, s, linepos);
            peek_token(h, s, &tmp, linepos);
            if (tmp->type != T_RPAREN) {
              // Error: Unbalanced parenthesis
              //free_token(t);
              result = allocate_error_token(h, ERR_UNBALANCED_PAREN, *linepos);
              ready = TRUE;
            } else {
              //free_token(tmp);
              next_token(h, s, &tmp, linepos);
            }
            //free_token(tmp);
            left = t;
            state = 1;
            break;

          case T_ID:
            left = t;
            state = 1;
            break;

          case T_EOL:
            // Empty expression
            //free_token(t);
            result = allocate_error_token(h, ERR_EMPTY_EXPR, *linepos);
            ready = TRUE;
            break;

          case T_OP_AND:
          case T_OP_OR:
            //free_token(t);
            result = allocate_error_token(h, ERR_EXTRA_OPERATOR, *linepos);
            ready = TRUE;
            break;

          case T_ERROR:
            result = left = t;
            ready = TRUE;
            break;

          case T_RPAREN:
            //free_token(t);
            result = allocate_error_token(h, ERR_EXTRA_RPAREN, *linepos);
            ready = TRUE;
            break;

          default:
            // Error: unknown token (really an internal error)
            //free_token(t);
            ready = TRUE;
            break;
        }
        break;


      case 1:
        // Expect infix operator
        toktyp = peek_token(h, s, &t, linepos);
        switch (toktyp) {
          case T_OP_AND:
          case T_OP_OR:
            //free_token(t);
            next_token(h, s, &t, linepos);
            op = t;
            state = 2;
            break;

          case T_RPAREN:
            //free_token(t);
            result = left;
            ready = TRUE;
            break;

          case T_ERROR:
            //free_token(t);
            next_token(h, s, &t, linepos);
            result = t;
            ready = TRUE;
            break;

          case T_EOL:
            result = left;
            //free_token(t);
            next_token(h, s, &t, linepos);
            //free_token(t);
            ready = TRUE;
            break;

          case T_ID:
          case T_LPAREN:
            //free_token(t);
            next_token(h, s, &t, linepos);
            //free_token(t);
            result = allocate_error_token(h, ERR_MISSING_OPERATOR, *linepos);
            ready = TRUE;
            break;

          default:
            // Error: unknown token (really an internal error)
            //free_token(t);
            next_token(h, s, &t, linepos);
            //free_token(t);
            result = allocate_error_token(h, ERR_UNKNOWN_TOKEN, *linepos);
            ready = TRUE;
            break;
        }
        break;


      case 2:
        // Expect right-side expression
        toktyp = next_token(h, s, &t, linepos);
        switch (toktyp) {
          case T_LPAREN:
            //free_token(t);
            t = parse(h, s, linepos);
            peek_token(h, s, &tmp, linepos);
            if (tmp->type != T_RPAREN) {
              // Error: Unbalanced parenthesis
              //free_token(t);
              result = allocate_error_token(h, ERR_UNBALANCED_PAREN, *linepos);
              ready = TRUE;
            } else {
              //free_token(tmp);
              next_token(h, s, &tmp, linepos);
            }
            //free_token(tmp);
            right = t;
            state = 3;
            break;

          case T_ID:
            right = t;
            state = 3;
            break;

          case T_EOL:
            //free_token(t);
            result = allocate_error_token(h, ERR_EMPTY_EXPR, *linepos);
            ready = TRUE;
            break;

          case T_OP_AND:
          case T_OP_OR:
            //free_token(t);
            result = allocate_error_token(h, ERR_EXTRA_OPERATOR, *linepos);
            ready = TRUE;
            break;

          case T_ERROR:
            result = t;
            ready = TRUE;
            break;

          default:
            // Error: unknown token (really an internal error)
            //free_token(t);
            ready = TRUE;
            break;
        }
        break;

      case 3:
        // Reduce this expression...
        op->left = left;
        op->right = right;
        left = result = op;

        // ... and look for another one, which doesn't need to be
        // there. But if it _is_ there we'll see an operator now..
        toktyp = peek_token(h, s, &t, linepos);
        switch (toktyp) {
          case T_OP_AND:
          case T_OP_OR:
            op = t;
            state = 1;
            //free_token(t);
            break;

          case T_ERROR:
            result = t;
            ready = TRUE;
            break;

          case T_EOL:
            // just so that we get linepos right
            next_token(h, s, &t, linepos);
            ready = TRUE;
            //free_token(t);
            break;

          case T_RPAREN:
            //free_token(t);
            ready = TRUE;
            break;

          case T_LPAREN:
            //free_token(t);
            result = allocate_error_token(h, ERR_MISSING_OPERATOR, *linepos);
            ready = TRUE;
            break;

          default:
            // error
            //free_token(t);
            ready = TRUE;
            break;
        }
        break;
    }
  }

  return result;
}


//=========================================================================
void dump_token(Token *t, int level)
{
  if (!t) return;
  PRINTF(("Level=%d Type: %d Name='%s' Err=%d Left=%p Right=%p\n",
          level, t->type, t->name?t->name:"", t->errCode, t->left, t->right));
  dump_token(t->left, level+1);
  dump_token(t->right, level+1);
}


//=========================================================================
int WINAPI kvaToolsParseExpr(KvParseHandle *h, char* expr, Token **t)
{
  int pos = 0;
  Token *tok;

  if (!h) return 0;

  tok = parse(h, expr, &pos);
  if (pos != (int)strlen(expr)) {
    //free_token(*t);
    tok = allocate_error_token(h, ERR_TRAILING_GARBAGE, pos);
  }
  if (!tok) tok = allocate_error_token(h, ERR_EMPTY_EXPR, 0);
  *t = tok;
  return 0;
}


//=========================================================================
int WINAPI kvaToolsFreeExpr(KvParseHandle * /*h*/, Token * /*t*/)
{
  // free_token(t);
  return 0;
}


//=========================================================================
int WINAPI kvaToolsDumpExpr(KvParseHandle * /*h*/, Token *t)
{
  dump_token(t, 0);
  return 0;
}


//=========================================================================
int WINAPI kvaToolsExprHasErrors(KvParseHandle * /*h*/, Token *t)
{
  if (!t) return FALSE;
  if (t->type == T_ERROR) return TRUE;
  return (kvaToolsExprHasErrors(0, t->left) || kvaToolsExprHasErrors(0, t->right));
}


//=========================================================================
int WINAPI kvaToolsExprGetError(KvParseHandle *h, Token *t, int *errCode, int *pos)
{
  if (!t) {
    *errCode = ERR_EMPTY_EXPR;
    *pos = 0;
    return TRUE;
  }
  if (t->type == T_ERROR) {
    *errCode = t->errCode;
    *pos = t->start_pos;
    return FALSE;
  }
  if (kvaToolsExprGetError(h, t->left, errCode, pos)) return TRUE;
  if (kvaToolsExprGetError(h, t->right, errCode, pos)) return TRUE;
  return FALSE;
}


//=============================================================================
kv_parser_handle AllocateParserData(struct_type type)
{
  t_parser_data *parse_data;

  PRINTF(("AllocateParserData(%u)\n", (unsigned int)sizeof(t_parser_data)));
  parse_data = (t_parser_data*)malloc(sizeof(t_parser_data));
  if (parse_data == NULL) {
    PRINTF(("Failed allocating mem. \n"));
    return NULL;
  }

  parse_data->error_description = new std::string();
  parse_data->expression_vec    = new std::vector<symbol>();
  parse_data->postfix_vec       = new std::vector<symbol>();

  parse_data->postfix_available  = false;
  parse_data->type               = type;
  parse_data->error              = false;
  *parse_data->error_description = "None";
  parse_data->result             = false;
  parse_data->expression_vec->clear();
  parse_data->postfix_vec->clear();

  //parser_list.push_back(*parse_data);

  return parse_data;
}

