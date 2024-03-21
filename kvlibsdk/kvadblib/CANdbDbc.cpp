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
 **   CANdbDbc.h: Read & write Vector CANdb (.dbc) files into/from CANdb classes
 **
 ** ---------------------------------------------------------------------------
 */


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <stdarg.h>
#include <math.h>
#include <clocale>
#include <float.h>
#include <climits>
#include <cerrno>

#include "CANdb.h"
#include "CANdbDbc.h"

 // ****************************************************************************

#ifndef MAX_PATH
#  define MAX_PATH      1024
#endif

#define DUMMY_RECEIVER_NODE                     "Vector__XXX"

// Environment variable types are partially defined by these strings,
// and partially by the type token.
#define DUMMY_EV_TOKEN_STRING                   "DUMMY_NODE_VECTOR8000"
#define DUMMY_EV_TOKEN_STRING_READ              "DUMMY_NODE_VECTOR8001"
#define DUMMY_EV_TOKEN_STRING_WRITE             "DUMMY_NODE_VECTOR8002"
#define DUMMY_EV_TOKEN_STRING_READWRITE         "DUMMY_NODE_VECTOR8003"

#define DUMMY_EV_TOKEN_NOT_STRING               "DUMMY_NODE_VECTOR0"
#define DUMMY_EV_TOKEN_NOT_STRING_READ          "DUMMY_NODE_VECTOR1"
#define DUMMY_EV_TOKEN_NOT_STRING_WRITE         "DUMMY_NODE_VECTOR2"
#define DUMMY_EV_TOKEN_NOT_STRING_READWRITE     "DUMMY_NODE_VECTOR3"

// Gleaned from NMEA2000.dbc by Vector Informatik...
// These are used to collect unused signals into one message where they all
// start at bit 0.
#define DUMMY_MSG_FOR_INDEPENDENT_SIGNALS       "VECTOR__INDEPENDENT_SIG_MSG"
#define DUMMY_MSGID_FOR_INDEPENDENT_SIGNALS     0xc0000000

// Tokens:
#define T_EOF              0
#define T_UNKNOWN        998
#define T_NEWLINE        999
#define T_BU            1000
#define T_BO            1001
#define T_BG            1002
#define T_SG            1003
#define T_CM            1004
#define T_NS            1005
#define T_BA            1006
#define T_EV            1007
#define T_BA_DEF        1008
#define T_BA_DEF_DEF    1009
#define T_SIG_VALTYPE   1010
#define T_CAT           1011
#define T_ENVVAR_DATA   1012
#define T_VAL           1020
#define T_IDENT         1021
#define T_UINT_CONST    1022
#define T_INT_CONST     1023
#define T_DOUBLE_CONST  1024
#define T_STRING_CONST  1025


// ****************************************************************************

static CANdbFileFormat candb_reader ("DBC",
                                     "Vector compatible DBC files",
                                     "Description of CAN network traffic in DBC format",
                                     "dbc",
                                     CANDB_FILE_FORMAT_FLAG_READ | CANDB_FILE_FORMAT_FLAG_WRITE,
                                     CANdbDBC::build);

// ****************************************************************************

struct s_keyword {
         const char *name;
         int  token;
       };

struct s_hash_entry {
         struct s_keyword *keyword;
         unsigned int     hash_val;
       };

// ****************************************************************************

static struct s_hash_entry *keyword_hash_table = NULL;
static unsigned int keyword_hash_table_size = 0;

static struct s_keyword keywords[] = {
         { "BA_",       T_BA      },      // User defined attribute
         { "BA_DEF_",   T_BA_DEF  },      // Definition of user defined attribute
         { "BA_DEF_DEF_",                 // Default for user defined attribute
                        T_BA_DEF_DEF  },
         { "BU_",       T_BU      },      // Network node
         { "BO_",       T_BO      },      // Message
         { "SG_",       T_SG      },      // Signal
         { "EV_",       T_EV      },      // Environment variable
         { "CM_",       T_CM      },      // Comment
         { "VAL_",      T_VAL     },      // Signal values
         { "CAT_",      T_CAT     },      // Category
         { "ENVVAR_DATA_",                // Environment variable of type "DATA"
                        T_ENVVAR_DATA },
         { "SIG_VALTYPE_",                // Valuetype of signal
                        T_SIG_VALTYPE },
         { "NS_",       T_NS      },      // New symbol
         { NULL,        0 },
       };

// Additional record types / tokens (those marked with '*' are manually checked for in the code):
//  BS_              Bit timing register
//  VERSION          Version information
//  INT            * Integer
//  HEX            * Hex
//  FLOAT          * Float
//  STRING         * String
//  ENUM           * Enumeration
//  NS_DESC_         Description of new symbol
//  CAT_DEF_         Category definition
//  FILTER           View filter
//  EV_DATA_         Environment variable of type "DATA"
//  SGTYPE_          Signaltype
//  SGTYPE_VAL_      Signaltype values
//  BA_DEF_SGTYPE_   Definition of user defined attributes for signaltypes
//  BA_SGTYPE_       User defined attributes for signaltypes
//  SIG_TYPE_REF_    Reference to signaltype
//  VAL_TABLE_       Value description table
//  SIG_GROUP_       Signal group
//  SIGTYPE_VALTYPE_ Valuetype of signaltype

// ****************************************************************************

static unsigned long hash_value (const char *s)
{
  unsigned long h = 0;
  if (!s) return h;
  while (*s) {
    h = h * 2 + *((unsigned char*) s);
    h = (h & 0x00ffffff) + ((h >> 24) & 0x000000ff);
    ++s;
  }
  return h;
} // hash_value


static void build_keyword_hash_table (void)
{
  if (!keyword_hash_table) {
    unsigned int keyword_count = 0;
    while (keywords [keyword_count].name) ++keyword_count;

    keyword_hash_table_size = keyword_count * 2 + 1;
    keyword_hash_table = new struct s_hash_entry [keyword_hash_table_size];
  }

  if (keyword_hash_table) {
    unsigned int i;

    for (i = 0; i < keyword_hash_table_size; ++i) {
      keyword_hash_table [i].keyword = NULL;
      keyword_hash_table [i].hash_val = 0;
    }

    i = 0;
    while (keywords[i].name) {
      unsigned long h = hash_value (keywords [i].name);
      unsigned int idx = h % keyword_hash_table_size;
      while (keyword_hash_table [idx].keyword) idx = (idx + 1) % keyword_hash_table_size;
      keyword_hash_table [idx].keyword  = &keywords [i];
      keyword_hash_table [idx].hash_val = h;
      ++i;
    }

    /*
    for (i = 0; i < keyword_hash_table_size; ++i) {
      printf ("%3d: ", i);
      if (keyword_hash_table [i].keyword) {
        printf ("%12lu -> %s",
                keyword_hash_table [i].hash_val,
                keyword_hash_table [i].keyword->name);
      }
      else printf ("    ****");
      printf ("\n");
    }
    */
  }
} // build_keyword_hash_table


static int find_token (char *str)
{
  if (keyword_hash_table) {
    unsigned long h = hash_value (str);
    unsigned int idx = h % keyword_hash_table_size;
    while (keyword_hash_table [idx].keyword &&
           ((keyword_hash_table [idx].hash_val != h) ||
            (strcmp (keyword_hash_table [idx].keyword->name, str) != 0))) {
      idx = (idx + 1) % keyword_hash_table_size;
    }
    if (!keyword_hash_table [idx].keyword) {
      /* printf ("KKK not found: %s\n", str); */
      return 0;
    }
      /* printf ("KKK found: %s / %s \n", str, hash_table [idx].keyword->string); */
    return keyword_hash_table [idx].keyword->token;
  }
  return -1;
} /* find_token */

/** Tries to convert the parsed token into a double constant.
 *
 * \return Whether the token was coerced to a double.
 */
static bool token_to_double (int t, Token token, double &out) {
  switch (t) {
  case T_UINT_CONST: out = token.uint_const; return true;
  case T_INT_CONST: out = token.int_const; return true;
  case T_DOUBLE_CONST: out = token.double_const; return true;
  default: return false;
  }
}

/** Tries to convert the parsed token into an int constant.
 *
 * \return Whether the token was coerced to an int.
 */
static bool token_to_int (int t, Token token, int &out) {
  switch (t) {
  case T_UINT_CONST:
    if (token.uint_const > INT_MAX) return false;
    out = token.uint_const;
    return true;
  case T_INT_CONST: out = token.int_const; return true;
  default: return false;
  }
}

// ****************************************************************************

/** Converts between sawtooth and sequential bit positions. */
static unsigned reflect (unsigned i)
{
  return (i & ~(8 - 1)) | (7 - i % 8);
}

/** Converts from Motorola forward MSB to -LSB. */
static unsigned motorola_forward_msb_to_lsb (unsigned i, unsigned len)
{
  return reflect (reflect (i) + len - 1);
}

/** Converts from Motorola forward LSB to -MSB. */
static unsigned motorola_forward_lsb_to_msb (unsigned i, unsigned len)
{
  return reflect (reflect (i) + 1 - len);
}

// ****************************************************************************


CANdbDBC::CANdbDBC (const char *filename)
  : CANdbFileIo (filename)
{
  file = NULL;
  current_db = NULL;
  file_eof = false;
  strcpy (string_buf, "");
} // CANdbDBC::CANdbDBC


CANdbDBC::~CANdbDBC ()
{
} // CANdbDBC::~CANdbDBC


// ****************************************************************************


char CANdbDBC::skip_space (void)
{
  signed char c;
  do {
    c = fgetc (file);
    if (c == '\n') lineno++;
    if (c == EOF) break;
  } while (c <= ' ');

  return c;
} // CANdbDBC::skip_space


int CANdbDBC::read_string (char c, Token& token)
{
  int i = 0;
  int abort = 0;

  if (c != '"') return -1;
  do {
    c = fgetc (file);
    if (i < CANDB_DBC_MAX_STRING) string_buf[i++] = c;

    if (c == '\n') lineno ++;

    if (c == '"') {
      c = fgetc (file);
      if (c != '"') {
        ungetc (c, file);
        abort = 1;
      }
    }
    else if (c == '\\') {
      c = fgetc (file);
      if (c == 'n') string_buf [i-1] = '\n';
      else if (c == 't') string_buf [i-1] = '\t';
      else if (c == 'r') string_buf [i-1] = '\r';
      else if (c == '"') string_buf [i-1] = c;
      else ungetc (c, file);
    }
    else if (feof(file)) {
      return -1;
    }
  }
  while (abort == 0);
  string_buf [i-1] = '\0';

  token.string_const = string_buf;

  return T_STRING_CONST;
} // CANdbDBC::read_string


int CANdbDBC::read_value (char c, Token& token)
{
  int i = 0, sign = 1;

  switch (c) {
  case '-': sign = -1; // fallthrough
  case '+':
    string_buf[i++] = c;
    c = fgetc (file);
    break;
  }

  if (! isdigit (c)) return -1;

  string_buf[i++] = c;
  while (isdigit (c = fgetc (file))) string_buf[i++] = c;

  /* Hex-Zahl */
  if ((c == 'x') || (c == 'X')) {
    if (string_buf [0] == '0') {
      if (i != 1) return -1;
    }
    else if ((string_buf [0] == '+') || (string_buf [0] == '-')) {
      if ((string_buf [1] != '0') || (i != 2)) return -1;
    }
    else return -1;
    --i;
    for (; isxdigit (c = fgetc (file)); i++) string_buf [i] = c;
    ungetc (c, file);
    string_buf [i] = '\0';
    sscanf (string_buf, "%x", &token.int_const);
    return T_INT_CONST;
  }

  /* Long- bzw. Integer-Zahl in Dezimalform */
  else if ((c != '.') && (c != 'E') && (c != 'e')) {
    string_buf[i] = '\0';

    errno = 0;
    if (sign > 0) {
      unsigned long u = strtoul (string_buf, NULL, 10);
      if (!(u == ULONG_MAX && errno == ERANGE) && u <= UINT_MAX) {
        ungetc (c, file);
        token.uint_const = u;
        return T_UINT_CONST;
      }
    } else {
      long l = strtol (string_buf, NULL, 10);
      if (!((l == LONG_MIN || l == LONG_MAX) && errno == ERANGE)
        && INT_MIN <= l && l <= INT_MAX) {
        ungetc (c, file);
        token.int_const = l;
        return T_INT_CONST;
      }
    }
  }

  /* Double- bzw. Float-Zahl */
  if (c == '.') {
    string_buf[i++] = c;
    while (isdigit (c = fgetc (file))) string_buf[i++] = c;
  }
  if ((c == 'E') || (c == 'e')) {
    string_buf[i++] = c;
    c = fgetc (file);
    if ((c == '-') || (c == '+')) {
      string_buf [i++] = c;
      c = fgetc (file);
    }
    if (isdigit (c)) {
      string_buf [i++] = c;
      while (isdigit (c = fgetc (file))) string_buf[i++] = c;
    }
    else return -1;
  }
  ungetc (c, file);
  string_buf [i] = '\0';

  sscanf (string_buf, "%lf", &token.double_const);
  return T_DOUBLE_CONST;
} // CANdbDBC::read_value

/* Reads a token from the infile. Blank space is skipped first. If during this, there is a new line,
* the token T_NEWLINE is returned.
* At EOF, a T_NEWLINE is returned; the next call results in T_EOF.
* Comments are just skipped.
*
*/
int CANdbDBC::lex (Token& token)
{
  signed char c;
  char c1;
  int linenoSave;

start:
  /* Leerzeichen ueberlesen */
  linenoSave = lineno;
  c = skip_space ();
  if (c == EOF) {
    if (file_eof) return T_EOF;
    file_eof = true;
    return T_NEWLINE; // Next call will result in T_EOF
  }

  if (lineno != linenoSave) {
    ungetc (c, file);
    return T_NEWLINE;
  }

  /* Kommentar ueberlesen */
  if (c == '/') {
    //int startline = lineno;
    c1 = fgetc (file);
    if (c1 == '*') {
      for (;;) {
        c = fgetc (file);
      next_char:
        if (c == '\n') lineno ++;
        if (c == EOF) {
          error ("Error: Unexpected eof in comment");
          // yyerror ("Error: Unexpected eof in comment (line %d)", startline);
          return -1;
        }
        if (c == '*') {
          c = fgetc (file);
          if (c == '/') goto start;
          goto next_char;
        }
      }
    }
    else ungetc (c1, file);
  }

  /* symbol or keyword */
  if (isalpha (c) || (c == '_')) {
    int tok = 0,
        i = 0;

    //static int tib = 0;
    // if ((tib++ % 2) == 0) ib = ident_buf1; else ib = ident_buf2;
    char *ib = string_buf;

    do {
      ib [i++] = c;
      c = fgetc (file);
    } while (isalpha (c) ||
             isdigit (c) ||
             (c == '_'));

    ungetc (c, file);
    ib[i] = '\0';

    tok = find_token (ib);
    if (tok > 0) return tok;

    token.ident = ib;
    return T_IDENT;
  }

  /* Zahlen (hex, int, double) */
  if ((c == '.') || (c == '-') || (c == '+')) {
    c1 = fgetc (file);
    /*
    if (c1 == '.') {
      c2 = fgetc (file);
      if (c1 == '.') return T_VARARG;
      ungetc (c2, file);
    }
    ungetc (c1, file);
    */
    ungetc (c1, file);
    if (!isdigit (c1)) return c;
  }

  if (isdigit (c) || (c == '.') || (c == '-') || (c == '+')) {
    return read_value (c, token);
  }

  /* string */
  else if (c == '"') {
    return read_string (c, token);
  }
  else {
    /*
    c1 = fgetc (file);
    if ((c == '&') && (c1 == '&')) return T_LOGICAL_AND;
    if ((c == '|') && (c1 == '|')) return T_LOGICAL_OR;
    if ((c == '+') && (c1 == '+')) return T_INC;
    if ((c == '-') && (c1 == '-')) return T_DEC;
    if (c1 == '=') {
      if (c == '>') return T_GE;
      if (c == '<') return T_LE;
      if (c == '!') return T_NE;
      if (c == '=') return T_EQ;
    }
    ungetc (c1, file);
    */
    return c;
  }

  return T_UNKNOWN;
} // CANdbDBC::lex


// Print a value, not including any unnecessary trailing decimal points.
void CANdbDBC::print_value (double val)
{
  char buf [100];

  // ME: Changed to %lg because the result is more like the original DBC files
  //     The old format led to a buffer overrun anyway
  // db: note that .11 is needed because default precision is 6..
  sprintf (buf, "%.11lg", val);

  // I don't think we need that / ME
  // Remove trailing zeroes after the decimal point.
  //if (strchr (buf, '.')) {
  //  char *p = buf + strlen (buf) - 1;
  //  while (*p == '0') *p-- = '\0';
  //  if (*p == '.') *p = '\0';
  //}

  fputs (buf, file);
} // CANdbDBC::print_value


/* Prints a '"'-delimited string. Certain control codes are escaped. The string can span
 * several lines.
 * A NULL-string is printed as an empty string.
 */
void CANdbDBC::print_string (const char *str) {
  unsigned char *s = (unsigned char*)str;
  fputc ('"', file);
  if (s) {
    while(*s) {
      if (*s == '\n') fputc (*s, file);
      else if (*s == '\t') fputs ("\\t", file);
      else if (*s == '\r') fputs ("\\r", file);
      else if (*s == '"') fputs ("\\\"", file);
      else if (*s >= 32) fputc (*s, file);
      else fprintf (file, "\\x%02x", *s);
      s++;
    }
  }
  fputc ('"', file);
} // CANdbDBC::print_string


// ****************************************************************************


int CANdbDBC::read_dbc_signal (CANdbSignal *signal, Token& token)
{
  int t;

  t = lex (token);
  if (t != T_IDENT) {
    error ("Signal name missing or invalid");
    return -1;
  }
  signal->set_name (token.ident);

  t = lex (token);
  if (t == T_IDENT) {
    char *id = token.ident;
    if (id [0] == 'm') {
      signal->set_mode ((int)_atoi64 (id + 1));
      //printf ("mode dependend %s\n", id);
    }
    else if (id [0] == 'M') {
      signal->set_mode_signal (true);
      // printf ("mode signal %s\n", id);
    }
    t = lex (token);
  }

  if (t != ':') {
    error ("':' expected after signal name (invalid character in name?)");
    return -1;
  }

  t = lex (token);
  if (t != T_UINT_CONST) {
    error ("Signal bit position expected");
    return -1;
  }
  signal->set_start_bit (token.uint_const);

  t = lex (token);
  if (t != '|') {
    error ("'|' expected after signal name");
    return -1;
  }

  t = lex (token);
  if (t != T_UINT_CONST) {
    error ("Signal length expected");
    return -1;
  }
  signal->set_length (token.uint_const);

  t = lex (token);
  if (t != '@') {
    error ("'@' expected after signal name");
    return -1;
  }

  t = lex (token); // 1: Intel, 0: Motorola
  if (t != T_UINT_CONST) {
    error ("Signal number expected");
    return -1;
  }
  if (token.uint_const == 0) {
    signal->set_motorola (true);
    // If we have a motorola message, we have to convert the start bit.
    // See the relating comment in the header file.
    unsigned sb = signal->get_start_bit (), len = signal->get_length ();
    signal->set_start_bit (motorola_forward_msb_to_lsb (sb, len));
  }

  t = lex (token);
  if ((t != '+') && (t != '-')) {
    error ("'+' or '-' expected");
    return -1;
  }
  if (t == '+') signal->set_type (CANDB_UNSIGNED);
           else signal->set_type (CANDB_SIGNED);

  t = lex (token);
  if (t != '(') {
    error ("'(' expected after signal name");
    return -1;
  }

  t = lex (token);
  double d;
  if (token_to_double (t, token, d)) signal->set_factor (d);
  else {
    error ("Factor expected");
    return -1;
  }

  t = lex (token);
  if (t != ',') {
    error ("',' expected after signal name");
    return -1;
  }

  t = lex (token);
  if (token_to_double (t, token, d)) signal->set_offset (d);
  else {
    error ("Offset expected");
    return -1;
  }

  t = lex (token);
  if (t != ')') {
    error ("')' expected after signal name");
    return -1;
  }

  t = lex (token);
  if (t != '[') {
    error ("'[' expected after signal name");
    return -1;
  }

  t = lex (token);
  if (token_to_double (t, token, d)) signal->set_min_val (d);
  else {
    error ("Min value expected");
    return -1;
  }

  t = lex (token);
  if (t != '|') {
    error ("'|' expected after signal name");
    return -1;
  }

  t = lex (token);
  if (token_to_double (t, token, d)) signal->set_max_val (d);
  else {
    error ("Max value expected");
    return -1;
  }

  t = lex (token);
  if (t != ']') {
    error ("']' expected after signal name");
    return -1;
  }

  t = lex (token);
  if (t != T_STRING_CONST) {
    error ("Signal unit expected");
    return -1;
  }

  signal->set_unit (token.string_const);

  t = lex (token);
  while (t == T_IDENT) { // receive nodes
    // signal->add_receive_node (token.ident);
    signal->add_receive_node (current_db->find_node_by_name (token.ident));
    t = lex (token);
    if (t == ',') t = lex (token);
  }

  return t;
} // CANdbDBC::read_dbc_signal


int CANdbDBC::read_dbc_message (CANdbMessage *msg, Token& token)
{
  int t;

  t = lex (token);
  if (t != T_UINT_CONST) {
    error ("CAN id expected");
    return -1;
  }
  msg->set_id (token.uint_const & ~CANDB_EXT_FLAG);

  msg->set_extended ((token.uint_const & CANDB_EXT_FLAG) != 0);

  t = lex (token);
  if (t != T_IDENT) {
    error ("Message name missing or invalid");
    return -1;
  }
  msg->set_name (token.ident);

  t = lex (token);
  if (t != ':') {
    error ("':' expected after message name (invalid character in name?)");
    return -1;
  }

  t = lex (token);
  if (t != T_UINT_CONST) {
    error ("DLC expected");
    return -1;
  }
  msg->set_dlc (token.uint_const);

  t = lex (token);
  if (t == T_IDENT) {
    if (strcmp (token.ident, DUMMY_RECEIVER_NODE) != 0) {
      msg->set_send_node (current_db->find_node_by_name (token.ident));
    }
    else {
      msg->set_send_node (NULL);
    }
    //msg->set_send_node (current_db->find_node_by_name (token.ident));
    t = lex (token);
  }
  else if (t != T_NEWLINE) {
    error ("Sender name expected");
    return -1;
  }

  /*
  while (t == T_IDENT) {
    // msg->add_receive_node (token.ident);
    t = lex (token);
    if (t == ',') {
      t = lex (token);
      if (t != T_IDENT) {
        error ("Name of receive node expected after ','");
        return -1;
      }
    }
  }
  */

  for (;;) {
    if (t == T_NEWLINE) {
      t = lex (token);
      continue;
    }
    if (t != T_SG)
      break;
    CANdbSignal *signal = new CANdbSignal;
    t = read_dbc_signal (signal, token);
    if (t >= 0)
      msg->insert_signal (signal);
    else
      return t;
  }

  return t;
} // CANdbDBC::read_dbc_message


int CANdbDBC::read_dbc_env_variable (CANdbEnvVariable *var, Token& token)
{
  int t;

  t = lex (token);
  if (t != T_IDENT) {
    error ("Variable name expected");
    return -1;
  }
  var->set_name (token.ident);

  t = lex (token);
  if (t != ':') {
    error ("':' expected after variable name");
    return -1;
  }

  // Type is 0 (int, string, data) or 1 (float)
  // "String" and "float" is further characterized by the "dummy" placeholder
  // described below
  t = lex (token);
  if (t != T_UINT_CONST) {
    error ("variable type expected");
    return -1;
  }
  //var->set_type ((CANdbSignalType)token.uint_const);

  // Assign a tentative type..
  switch (token.uint_const) {
  case 0: var->set_type (CANDB_EV_INTEGER); break;
  case 1: var->set_type (CANDB_EV_FLOAT); break;
  default:
    error ("Unknown environment variable type");
    return -1;
  }

  t = lex (token);
  if (t != '[') {
    error ("'[' expected after variable type");
    return -1;
  }

  t = lex (token);
  double d;
  if (token_to_double (t, token, d)) var->set_min_val (d);
  else {
    error ("Min value expected");
    return -1;
  }

  t = lex (token);
  if (t != '|') {
    error ("'|' expected in variable min/max");
    return -1;
  }

  t = lex (token);
  if (token_to_double (t, token, d)) var->set_max_val (d);
  else {
    error ("Max value expected");
    return -1;
  }

  t = lex (token);
  if (t != ']') {
    error ("']' expected after variable min/max");
    return -1;
  }

  t = lex (token);
  if (t != T_STRING_CONST) {
    error ("Variable unit expected");
    return -1;
  }

  var->set_unit (token.string_const);

  t = lex (token);
  if (token_to_double (t, token, d)) var->set_start_value (d);
  else {
    error ("Start value expected");
    return -1;
  }

  t = lex (token);
  if (t != T_UINT_CONST) {
    error ("Num id expected");
    return -1;
  }
  var->set_num_id (token.uint_const);

  // Oh blimey. "string" has type 0 (see above) and this token is
  // "DUMMY_NODE_VECTOR8000". For all other types it's
  // "DUMMY_NODE_VECTOR0". Type "data" has an "ENVVAR_DATA_" record as well.
  t = lex (token);
  if (t != T_IDENT) {
    error ("dummy place holder expected");
    return -1;
  }

  // Save the string verbatim.
  var->set_dummy_node (token.ident);

  // Identify the string type.
  if (strcmp (DUMMY_EV_TOKEN_STRING, token.ident) == 0) {
    if (var->get_type () == CANDB_EV_INTEGER) {
      var->set_type (CANDB_EV_STRING);
      var->set_access (CANDB_EV_UNRESTRICTED);
    }
    else {
      error ("Inconsistent environment variable type.");
      return -1;
    }
  }
  // Indentify the string type with read access
  else if (strcmp (DUMMY_EV_TOKEN_STRING_READ, token.ident) == 0) {
    if (var->get_type () == CANDB_EV_INTEGER) {
      var->set_type (CANDB_EV_STRING);
      var->set_access (CANDB_EV_READ);
    }
    else {
      error ("Inconsistent environment variable type.");
      return -1;
    }
  }

  // Indentify the string type with write access
  else if (strcmp (DUMMY_EV_TOKEN_STRING_WRITE, token.ident) == 0) {
    if (var->get_type () == CANDB_EV_INTEGER) {
      var->set_type (CANDB_EV_STRING);
      var->set_access (CANDB_EV_WRITE);
    }
    else {
      error ("Inconsistent environment variable type.");
      return -1;
    }
  }

  // Indentify the string type with read/write access
  else if (strcmp (DUMMY_EV_TOKEN_STRING_READWRITE, token.ident) == 0) {
    if (var->get_type () == CANDB_EV_INTEGER) {
      var->set_type (CANDB_EV_STRING);
      var->set_access (CANDB_EV_READWRITE);
    }
    else {
      error ("Inconsistent environment variable type.");
      return -1;
    }
  }

  // Indentify the non string type with unrestricted access
  else if (strcmp (DUMMY_EV_TOKEN_NOT_STRING, token.ident) == 0) {
    var->set_access (CANDB_EV_UNRESTRICTED);
  }

  // Indentify the non string type with read access
  else if (strcmp(DUMMY_EV_TOKEN_NOT_STRING_READ, token.ident) == 0) {
    var->set_access (CANDB_EV_READ);
  }

  // Indentify the non string type with write access
  else if (strcmp (DUMMY_EV_TOKEN_NOT_STRING_WRITE, token.ident) == 0) {
    var->set_access (CANDB_EV_WRITE);
  }

  // Indentify the non string type with read/write access
  else if (strcmp (DUMMY_EV_TOKEN_NOT_STRING_READWRITE, token.ident) == 0) {
    var->set_access (CANDB_EV_READWRITE);
  }

  t = lex (token);
  while (t == T_IDENT) { // receive nodes
    if (strcmp (token.ident, DUMMY_RECEIVER_NODE) != 0) {
       // Skip dummy receiver nodes.
       var->add_receive_node (current_db->find_node_by_name (token.ident));
    }
    t = lex (token);
    if (t == ',') t = lex (token);
  }

  return t;
} // CANdbDBC::read_dbc_signal


int CANdbDBC::read_dbc_attribute_default_value (Token& token)
{
  int t;

  t = lex (token);
  if (t != T_STRING_CONST) {
    error ("Attribute name expected");
    return -1;
  }
  CANdbAttributeDefinition *ad = current_db->find_attribute_definition_by_name (token.string_const);
  if (!ad) {
    error ("Attribute '%s' not found\n", token.string_const);
    return -1;
  }

  t = lex (token);

  if (ad->get_type () == CANDB_ATTR_TYPE_STRING) {
    if (t != T_STRING_CONST) {
      error ("String expected");
      return -1;
    }
    ad->set_string_default (token.string_const);
  }

  if (ad->get_type () == CANDB_ATTR_TYPE_ENUMERATION) {
    if (t != T_STRING_CONST) {
      error ("String expected");
      return -1;
    }
    int v = ad->get_enumeration_value_by_name (token.string_const);
    ad->set_enumeration_default (v);
  }

  if ((ad->get_type () == CANDB_ATTR_TYPE_INTEGER) ||
      (ad->get_type () == CANDB_ATTR_TYPE_HEX)) {
    if (t == T_UINT_CONST) {
      if (ad->get_type () == CANDB_ATTR_TYPE_INTEGER) ad->set_integer_default (token.uint_const);
      if (ad->get_type () == CANDB_ATTR_TYPE_HEX) ad->set_hex_default (token.uint_const);
    } else {
      if (t != T_INT_CONST) {
        error ("Integer value expected");
        return -1;
      }
      if (ad->get_type () == CANDB_ATTR_TYPE_INTEGER) ad->set_integer_default (token.int_const);
      if (ad->get_type () == CANDB_ATTR_TYPE_HEX) ad->set_hex_default (token.int_const);

    }
  }

  else if (ad->get_type () == CANDB_ATTR_TYPE_FLOAT) {
    double d;
    if (token_to_double (t, token, d)) { ad->set_float_default (d); }
    else {
      error ("Float value expected");
      return -1;
    }

  }

  t = lex (token);
  if (t != ';') {
    error ("';' expected (attribute default value)");
    return -1;
  }
  t = lex (token);

  return t;
} // CANdbDBC::read_dbc_attribute_default_value


int CANdbDBC::read_dbc_attribute (Token& token)
{
  int t;
  CANdbAttribute *a = NULL;

  t = lex (token);
  if (t != T_STRING_CONST) {
    error ("Attribute name expected");
    return -1;
  }
  CANdbAttributeDefinition *ad = current_db->find_attribute_definition_by_name (token.string_const);
  if (!ad) {
    printf ("Attribute '%s' not found\n", token.string_const);
    return -1;
  }
  else a = new CANdbAttribute (ad);

  CANdbAttributeList *al = NULL;

  int tsave = t = lex (token);
  if ((t == T_BO) || (t == T_SG)) {
    t = lex (token);
    if (t != T_UINT_CONST) {
      error ("Message Id expected");
      delete a;
      return -1;
    }
    CANdbMessage *m = current_db->find_message_by_id (token.uint_const);
    if (!m) warning ("Message %d not found", token.uint_const);
    else al = m->get_attributes ();

    if (tsave == T_SG) {
      t = lex (token);
      if (t != T_IDENT) {
        error ("Message name expected");
        delete a;
        return -1;
      }
      if (m) {
        CANdbSignal *s = m->find_signal_by_name (token.ident);
        if (s) al = s->get_attributes ();
      }
    }

    t = lex (token);
  }
  else if (t == T_EV) {
    t = lex (token);
    if (t != T_IDENT) {
      error ("EnvVar name expected");
      delete a;
      return -1;
    }
    CANdbEnvVariable *envvar = current_db->find_env_variable_by_name (token.ident);
    if (envvar) al = envvar->get_attributes ();
    t = lex (token);
  }
  else if (t == T_BU) {
    t = lex (token);
    if (t != T_IDENT) {
      error ("Node name expected");
      delete a;
      return -1;
    }
    CANdbNode *n = current_db->find_node_by_name (token.ident);
    if (n) al = n->get_attributes ();
    t = lex (token);
  }
  else {
    al = current_db->get_attributes ();
  }

  if (ad->get_type () == CANDB_ATTR_TYPE_STRING) {
    if (t != T_STRING_CONST) {
      error ("String expected");
      delete a;
      return -1;
    }
    if (a) a->set_string_value (token.string_const);
  }

  else if ((ad->get_type () == CANDB_ATTR_TYPE_INTEGER) ||
      (ad->get_type () == CANDB_ATTR_TYPE_HEX) ||
      (ad->get_type () == CANDB_ATTR_TYPE_ENUMERATION)) {
    if (t == T_UINT_CONST) { if (a) a->set_integer_value (token.uint_const); }
    else if (t == T_INT_CONST) { if (a) a->set_integer_value (token.int_const); }
    else {
      error ("Integer value expected");
      delete a;
      return -1;
    }
  }

  else if (ad->get_type () == CANDB_ATTR_TYPE_FLOAT) {
    double d;
    if (token_to_double (t, token, d)) { if (a) a->set_float_value (d); }
    else {
      error ("Float value expected");
      delete a;
      return -1;
    }
  }

  t = lex (token);
  if (t != ';') {
    error ("';' expected (attribute value)");
    delete a;
    return -1;
  }
  t = lex (token);

  if (al && a) al->insert (a);

  return t;
} // CANdbDBC::read_dbc_attribute


int CANdbDBC::read_dbc_attribute_definition (Token& token)
{
  int t;
  CANdbAttributeOwner ao = CANDB_ATTR_OWNER_INVALID;
  CANdbAttributeDefinition *a = NULL;

  t = lex (token);
  if (t == T_SG) {
    ao = CANDB_ATTR_OWNER_SIGNAL;
    t = lex (token);
  }
  else if (t == T_BU) {
    ao = CANDB_ATTR_OWNER_NODE;
    t = lex (token);
  }
  else if (t == T_BO) {
    ao = CANDB_ATTR_OWNER_MESSAGE;
    t = lex (token);
  }
  else if (t == T_EV) {
    ao = CANDB_ATTR_OWNER_ENVIRONMENT;
    t = lex (token);
  }
  else if (t == T_STRING_CONST) ao = CANDB_ATTR_OWNER_DB;

  if (ao != CANDB_ATTR_OWNER_INVALID) {
    a = new CANdbAttributeDefinition;
    a->set_owner (ao);
  }

  if (t != T_STRING_CONST) {
    error ("Attribute name expected");
    delete a;
    return -1;
  }
  if (a) a->set_name (token.string_const);

  t = lex (token);
  if (t != T_IDENT) {
    error ("Attribute type expected");
    delete a;
    return -1;
  }

  CANdbAttributeType at = CANDB_ATTR_TYPE_INVALID;
  if (strcmp (token.ident, "ENUM") == 0) at = CANDB_ATTR_TYPE_ENUMERATION;
  else if (strcmp (token.ident, "INT") == 0) at = CANDB_ATTR_TYPE_INTEGER;
  else if (strcmp (token.ident, "HEX") == 0) at = CANDB_ATTR_TYPE_HEX;
  else if (strcmp (token.ident, "STRING") == 0) at = CANDB_ATTR_TYPE_STRING;
  else if (strcmp (token.ident, "FLOAT") == 0) at = CANDB_ATTR_TYPE_FLOAT;
  if (a) {
    a->set_type (at);
    t = lex (token);

    if (at == CANDB_ATTR_TYPE_ENUMERATION) {
      int value = 0;

      while ((t > 0) && (t != ';')) {
        if (t == T_STRING_CONST) {
          a->add_enumeration (token.string_const, value);
          ++value;
        }
        t = lex (token);
      }
    }

    else if (at == CANDB_ATTR_TYPE_INTEGER) {
      int i;
      if (!token_to_int (t, token, i)) {
        warning ("Integer attribute min value expected");
        i = INT_MIN; // Set default value
      }
      a->set_integer_min (i);

      t = lex (token);
      if (!token_to_int (t, token, i)) {
        warning ("Integer attribute max value expected");
        i = INT_MAX; // Set default value
      }
      a->set_integer_max (i);

      t = lex (token);
    }
    else if (at == CANDB_ATTR_TYPE_HEX) {
      if (t != T_UINT_CONST) {
        warning ("Hex attribute min value expected");
        token.uint_const = 0; // Set default value
      }
      a->set_hex_min (token.uint_const);

      t = lex (token);
      if (t != T_UINT_CONST) {
        warning ("Hex attribute max value expected");
        token.uint_const = UINT_MAX; // Set default value
      }
      a->set_hex_max (token.uint_const);

      t = lex (token);
    }

    else if (at == CANDB_ATTR_TYPE_STRING) {
      if (t != ';') {
        error ("';' expected (String Attribute)");
        delete a;
        return -1;
      }
    }

    else if (at == CANDB_ATTR_TYPE_FLOAT) {
      double d;
      if (!token_to_double (t, token, d)) {
        warning ("Attribute min value expected");
        d = -FLT_MAX;
      }
      a->set_float_min (d);

      t = lex (token);
      if (!token_to_double (t, token, d)) {
        warning ("Attribute max value expected");
        d = FLT_MAX;
      }
      a->set_float_max (d);

      t = lex (token);
    }
  }
  else { // skip until ';'
    while ((t > 0) && (t != ';')) {
      t = lex (token);
    }
  }

  if (t == ';') t = lex (token);

  if (a) current_db->insert_attribute_definition (a);

  return t;
} // CANdbDBC::read_dbc_attribute_definition


int CANdbDBC::read_file (CANdb *db)
{
  current_db = db;

  build_keyword_hash_table ();

  if (!get_filename ()) {
    set_read_ok (false);
    return -1;
  }

  file = fopen(get_filename(), "r");
  
  if (!file) {
	set_read_ok (false);
    file_eof = true;
    return -1;
  }
  file_eof = false;

  int t;
  Token token;

  t = lex (token);

  char *init_locale = std::setlocale(LC_NUMERIC,"C");
  
  set_read_ok (true);
  int n_err = 0;

  do {
    if (t == T_NS) { // skip the stupid header
      do {
        t = lex (token);
      } while ((t > 0) && (t != T_BU));
    }

    if (t == T_BU) { // node list
      t = lex (token);
      if (t != ':') {
        error ("':' expected");
        //return -1;
        t = -1;
        continue;
      }
      do {
        t = lex (token);
        if (t == T_IDENT) {
          if (strcmp (token.ident, DUMMY_RECEIVER_NODE) != 0) {
            CANdbNode *node = new CANdbNode;
            node->set_name (token.ident);
            current_db->insert_node (node);
          }
        }
      } while (t == T_IDENT);
    }

    else if (t == T_BA_DEF) { // attribute definition
      t = read_dbc_attribute_definition (token);
    }

    else if (t == T_BA_DEF_DEF) { // attribute default value
      t = read_dbc_attribute_default_value (token);
    }

    else if (t == T_BA) { // attribute  value
      t = read_dbc_attribute (token);
    }

    else if (t == T_CAT) { // skip the category sections
      while ((t > 0) && (t != ';')) {
        t = lex (token);
      }
      if (t == ';') t = lex (token);
    }

    else if (t == T_BO) { // read message
      //printf ("Read Message\n");
      CANdbMessage *msg = new CANdbMessage;
      t = read_dbc_message (msg, token);
      if (t >= 0) current_db->insert_message (msg);
        else delete msg;
    }

    else if (t == T_EV) { // read environment variable
      //printf ("Read Variable\n");
      CANdbEnvVariable *var = new CANdbEnvVariable;
      t = read_dbc_env_variable (var, token);
      if (t >= 0) current_db->insert_env_variable (var);
             else delete var;
    }
    else if (t == T_ENVVAR_DATA) { // read variable data
      t = lex (token);
      if (t != T_IDENT) error ("variable name expected for variable data");
      else {
        CANdbEnvVariable *var = current_db->find_env_variable_by_name (token.string_const);
        t = lex(token);
        if (t != ':')
          error ("':' expected");
        else
          t = lex(token);
        if (!var)
          error ("No variable found for data line");
        else {
          double d;
          if (token_to_double (t, token, d)) { var->set_data (d); }
          else {
            error ("Value expected");
          }
        }
        t = lex (token);
        if (t != ';')
          error ("';' expected after variable data");
      }
    }
    else if (t == T_CM) { // read comment
      //printf ("Read Comment %d\n", lineno);

      t = lex (token);

      if (t == T_STRING_CONST) { // comment for data base
        current_db->set_comment (token.string_const);
      }
      else if (t == T_BU) { // comment for node
        t = lex (token);
        if (t != T_IDENT) {
          error ("Name expected for module comment");
        }
        else {
          CANdbNode *n = current_db->find_node_by_name (token.string_const);
          if (!n)
            error ("No node found for comment line");
          else {
            t = lex (token);
            if (t != T_STRING_CONST)
              error ("String expected for node comment");
            else
              n->set_comment (token.string_const);
          }
        }
      }
      else if (t == T_BO) { // comment for message
        t = lex (token);
        if (t != T_UINT_CONST) error ("Id expected for message comment");
        else {
          CANdbMessage *msg = current_db->find_message_by_id (token.uint_const);
          if (!msg) error ("No message found for comment line");
          else {
            t = lex (token);
            if (t != T_STRING_CONST) error ("String expected for message comment");
            else msg->set_comment (token.string_const);
          }
        }
      }
      else if (t == T_SG) { // comment for signal
        t = lex (token);
        if (t != T_UINT_CONST) error ("Id expected for signal comment");
        else {
          CANdbMessage *msg = current_db->find_message_by_id (token.uint_const);
          if (!msg) error ("Message (id=%d) found for comment line", token.int_const);
          else {
            t = lex (token);
            if (t != T_IDENT) error ("Signal name expected for signal comment");
            else {
              CANdbSignal *signal = msg->find_signal_by_name (token.string_const);
              if (!signal) error ("Signal not found");
              else {
                t = lex (token);
                if (t != T_STRING_CONST) error ("String expected for signal comment");
                else signal->set_comment (token.string_const);
              }
            }
          }
        }
      }
      else if (t == T_EV) { // comment for environment variable
        t = lex (token);
        if (t != T_IDENT) {
          error ("Name expected for variable comment");
        }
        else {
          CANdbEnvVariable *var = current_db->find_env_variable_by_name (token.string_const);
          if (!var) {
            error ("No variable found for comment line");
          }
          else {
            t = lex (token);
            if (t != T_STRING_CONST) {
              error ("String expected for variable comment");
            }
            else var->set_comment (token.string_const);
          }
        }
      }

      // skip the rest
      while ((t > 0) && (t != ';')) {
        t = lex (token);
      }
      if (t == ';') t = lex (token);
    }

    else if (t == T_SIG_VALTYPE) { // type i.e. float for signal
      t = lex (token);
      if (t != T_UINT_CONST) error ("Id expected for signal type");
      else {
        CANdbMessage *msg = current_db->find_message_by_id (token.uint_const);
        if (!msg) error ("Message (id=%d) found for type info", token.int_const);
        else {
          t = lex (token);
          if (t != T_IDENT) error ("Signal name expected for signal type");
          else {
            CANdbSignal *signal = msg->find_signal_by_name (token.string_const);
            if (!signal) error ("Signal not found");
            else {
              t = lex (token);
              if (t != ':') error ("Expected ':'");
              else {
                t = lex (token);
                if (t != T_UINT_CONST) error ("Number expected for signal type");
                else switch (token.uint_const) {
                    case 1:
                        signal->set_type (CANDB_FLOAT);
                        break;
                    case 2:
                        signal->set_type (CANDB_DOUBLE);
                        break;
                    default:
                        error ("Unknown signal type %d", token.uint_const);
                }
                t = lex (token);
                if (t == ';') t = lex (token);
              }
            }
          }
        }
      }
    }

    else if (t == T_VAL) { // string values for signal
      t = lex (token);
      if (t == T_IDENT) {
        // Values for an environment variable
        CANdbEnvVariable *var = current_db->find_env_variable_by_name (token.string_const);
        if (!var) error ("Variable not found");
        else {
          t = lex (token);
          while (1) {
            int num;
            if (!token_to_int (t, token, num)) {
              error ("Number expected for variable value");
              break;
            }

            t = lex (token);
            if (t != T_STRING_CONST) {
              error ("String expected for variable value name");
              break;
            }

            var->add_value(num, token.string_const);

            t = lex (token);
            if (t == ';') {
              t = lex (token);
              break;
            }
          }
        }
      }
      else if (t == T_UINT_CONST) {
        CANdbMessage *msg = current_db->find_message_by_id (token.uint_const);
        if (!msg) error ("Message (id=%d) found for type info", token.uint_const);
        else {
          t = lex (token);
          if (t != T_IDENT) error ("Signal name expected for signal value");
          else {
            CANdbSignal *signal = msg->find_signal_by_name (token.string_const);
            if (!signal) error ("Signal not found");
            else {
              t = lex (token);
              while (1) {
                int num;
                if (!token_to_int (t, token, num)) {
                  error ("Number expected for signal value");
                  break;
                }

                t = lex (token);
                if (t != T_STRING_CONST) {
                  error ("String expected for signal value");
                  break;
                }

                signal->add_value (num, token.string_const);

                t = lex (token);
                if (t == ';') {
                  t = lex (token);
                  break;
                }
              }
            }
          }
        }
      }
      else {
        error ("Unsupported VAL record");
        // Skip it
        while ((t > 0) && (t != ';')) {
          t = lex (token);
        }
        if (t == ';') t = lex (token);
      }
    }

    else t = lex (token);

    if (t<0) {
      n_err += 1;
      set_read_ok (false);
    }

  } while (t != T_EOF && n_err == 0);

  if (init_locale != NULL) {
    std::setlocale(LC_NUMERIC, init_locale);
  }
    
  fclose (file);
  file = NULL;

  if (n_err == 0) 
  {
    current_db->update_j1939_flag ();

    // Setup internal data structures
    CANdbMessage *message = current_db->get_first_message ();
    while (message) {
      message->setup ();

      message = message->get_next ();
    }
  }

  if (keyword_hash_table) delete [] keyword_hash_table;
  keyword_hash_table = NULL;
  keyword_hash_table_size = 0;
  
  return (is_read_ok ()) ? 0: -1;
} // CANdbDBC::read_file


// ****************************************************************************


// Save the database to file. Returns 0 if success.
int CANdbDBC::save_file (CANdb *db)
{
  current_db = db;
  
  file = fopen(get_filename(), "w");
  if (!file) {
    return -1;
  }
  
  char *init_locale = std::setlocale(LC_NUMERIC,"C");
  int ret_code = 0;

  // The version.
  fputs ("VERSION \"HIPBNYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYY/4/%%%/4/'%**4YYY///\"\n", file);

  fprintf (file, "\n\nNS_ : \n");
  fprintf (file, "\tNS_DESC_\n"
           "\tCM_\n"
           "\tBA_DEF_\n"
           "\tBA_\n"
           "\tVAL_\n"
           "\tCAT_DEF_\n"
           "\tCAT_\n"
           "\tFILTER\n"
           "\tBA_DEF_DEF_\n"
           "\tEV_DATA_\n"
           "\tENVVAR_DATA_\n"
           "\tSGTYPE_\n"
           "\tSGTYPE_VAL_\n"
           "\tBA_DEF_SGTYPE_\n"
           "\tBA_SGTYPE_\n"
           "\tSIG_TYPE_REF_\n"
           "\tVAL_TABLE_\n"
           "\tSIG_GROUP_\n"
           "\tSIG_VALTYPE_\n"
           "\tSIGTYPE_VALTYPE_\n"
           "\tBO_TX_BU_\n"
           "\tBA_DEF_REL_\n"
           "\tBA_REL_\n"
           "\tBA_DEF_DEF_REL_\n"
           "\tBU_SG_REL_\n"
           "\tBU_EV_REL_\n"
           "\tBU_BO_REL_\n"
           "\tSG_MUL_VAL_\n\n");

  fprintf (file, "BS_:\n\n");

  // The node list.
  fprintf (file, "BU_:");
  CANdbNode *node = db->get_first_node ();
  while (node) {
    fprintf (file, " %s", node->get_name ());
    node = node->get_next ();
  }
  fprintf (file, "\n\n\n");

  // The messages
  CANdbMessage *m = db->get_first_message ();
  while (m) {
    fprintf (file, "BO_ %u %s: %d", m->get_id_and_ext(), m->get_name (), m->get_dlc ());

    fprintf (file,
             " %s\n",
             (m->get_send_node () && m->get_send_node () ->get_name ()) ?
                m->get_send_node ()->get_name () :
                DUMMY_RECEIVER_NODE);

    CANdbSignal *s = m->get_first_signal ();
    while (s) {
      fprintf (file, " SG_ %s", s->get_name ());

      if (s->is_mode_signal ()) {
        fprintf (file, " M");
      }
      else if (s->is_mode_dependent ()) {
        fprintf (file, " m%d", s->get_mode());
      }

      if (s->is_motorola ()) {
        unsigned sb = s->get_start_bit (), len = s->get_length ();
        fprintf (file, " : %u|%u@0", motorola_forward_lsb_to_msb (sb, len), len);
      }
      else {
        fprintf (file, " : %d|%d@1", s->get_start_bit(), s->get_length());
      }

      fputc (s->get_type () == CANDB_UNSIGNED ? '+' : '-', file);

      // Print (factor,offset) [min,max]
      fprintf (file, " (");
      print_value (s->get_factor ());
      fprintf (file, ",");
      print_value (s->get_offset ());
      fprintf (file, ") [");
      print_value (s->get_min_val ());
      fprintf (file, "|");
      print_value (s->get_max_val ());
      fprintf (file, "] ");

      // Print the engineering unit
      print_string (s->get_unit ());

      // Print receiver node(s)
      // If there are no receivers defined, write "Vector__XXX" which is *magic* and required
      // for CANdb compability.
      CANdbNodeEntry *ne = s->get_first_receive_node_entry ();
      bool first = true;
      while (ne) {
        if (first) {
          fprintf (file, "  ");
          first = false;
        }
        else fprintf (file, ",");

        fprintf (file, "%s", ne->get_node () ->get_name ());
        ne = ne->get_next ();
      }
      // No node was defined.
      if (first) fprintf (file, " %s", DUMMY_RECEIVER_NODE);

      fprintf (file, "\n");
      s = m->get_next_signal ();
    }
    fprintf (file, "\n");
    m = db->get_next_message ();
  }

  // EV_
  // The enviroment variables
  CANdbEnvVariable *ev = db->get_first_env_variable ();
  while (ev) {
    //fprintf (file, "\nEV_ %s: %d [", ev->get_name (), ev->get_type ());
    fprintf (file, "\nEV_ %s: ", ev->get_name ());

    switch (ev->get_type ()) {
      case CANDB_EV_INVALID:
      case CANDB_EV_INTEGER:
      case CANDB_EV_STRING:
      case CANDB_EV_DATA:
           fprintf (file, "0");
           break;

      case CANDB_EV_FLOAT:
           fprintf (file, "1");
           break;
    }

    fprintf(file, " [");
    print_value (ev->get_min_val ());
    fprintf (file, "|");
    print_value (ev->get_max_val ());
    fprintf (file, "] ");
    print_string (ev->get_unit ());
    fprintf (file, " ");
    print_value (ev->get_start_value ());
    fprintf (file, " %u %s", ev->get_num_id (), ev->get_dummy_node ());
    CANdbNodeEntry *ne = ev->get_first_receive_node_entry ();
    bool first = true;
    while (ne) {
      if (first) {
        fprintf (file, "  ");
        first = false;
      }
      else fprintf (file, ",");
      fprintf (file, "%s", ne->get_node () ->get_name ());
      ne = ne->get_next ();
    }
    if (first) {
      fprintf (file, "  ");
      fprintf (file, DUMMY_RECEIVER_NODE);
    }
    fprintf (file, ";\n");
    ev = ev->get_next ();
  }
  // ENVVAR_DATA_
  // The enviroment variable ata
  ev = db->get_first_env_variable ();
  while (ev) {
    if (ev->get_data_flag ()) {
      fprintf (file, "ENVVAR_DATA_ %s: ", ev->get_name ());
      print_value (ev->get_data ());
      fprintf (file, ";\n");
    }
    ev = ev->get_next ();
  }
  fprintf (file, "\n");

  // The comments:
  // Database comments
  if (db->get_comment ()) {
    fprintf (file, "CM_ ");
    print_string (db->get_comment ());
    fprintf (file, ";\n");
  }

  // BU_ (node) comments
  CANdbNode *n = db->get_first_node ();
  while(n) {
    if (n->get_comment ()) {
      fprintf (file, "CM_ BU_ %s ", n->get_name ());
      print_string (n->get_comment ());
      fprintf (file, ";\n");
    }
    n = n->get_next ();
  }

  // BO_ (message) comments and SG_ (signal) comments
  CANdbMessage *msg = db->get_first_message ();
  while (msg) {
    if (msg->get_comment ()) {
      fprintf (file, "CM_ BO_ %u ", msg->get_id_and_ext ());
      print_string (msg->get_comment ());
      fprintf (file, ";\n");
    }
    CANdbSignal *s = msg->get_first_signal ();
    while (s) {
      if (s->get_comment ()) {
        fprintf (file, "CM_ SG_ %u %s ", msg->get_id_and_ext (), s->get_name ());
        print_string (s->get_comment ());
        fprintf (file, ";\n");
      }
      s = s->get_next ();
    }
    msg = msg->get_next ();
  }

  // EV_-comments
  CANdbEnvVariable *v = db->get_first_env_variable ();
  while (v) {
    if (v->get_comment ()) {
      fprintf (file, "CM_ EV_ %s ", v->get_name ());
      print_string (v->get_comment ());
      fprintf (file, ";\n");
    }
    v = v->get_next ();
  }

  // BA_DEF
  // The attribute definitions
  CANdbAttributeDefinition *ad = db->get_first_attribute_definition ();
  while (ad) {
    const char *owner_name;
    switch (ad->get_owner ()) {
      case CANDB_ATTR_OWNER_DB:
           owner_name = "";
           break;

      case CANDB_ATTR_OWNER_MESSAGE:
           owner_name = "BO_";  // T_BO
           break;

      case CANDB_ATTR_OWNER_NODE:
           owner_name = "BU_";  // T_BU
           break;

      case CANDB_ATTR_OWNER_SIGNAL:
           owner_name = "SG_";  // T_SG
           break;

      case CANDB_ATTR_OWNER_ENVIRONMENT:
           owner_name = "EV_";  // T_EV
           break;

      case CANDB_ATTR_OWNER_INVALID:
      default:
           owner_name = NULL;
           break;
    }

    if (!owner_name) {
      ret_code = 1;
      goto finally;
    }

    fprintf (file, "BA_DEF_ ");
    if (*owner_name) fprintf (file, "%s ", owner_name);
    fputc (' ', file); // There is extra space in files written by the vector tools
    print_string (ad->get_name ());
    fputc (' ', file);
    switch (ad->get_type ()) {
      case CANDB_ATTR_TYPE_ENUMERATION: {
             int vv = 0;
             const char *enumName;
             fprintf (file, "ENUM  ");
             while ((enumName = ad->get_enumeration_name_by_value (vv)) != NULL) {
               if (vv) fputc (',', file);
               print_string (enumName);
               vv++;
             }
           }
           break;

      case CANDB_ATTR_TYPE_INTEGER:
           fprintf (file, "INT %d %d", ad->get_integer_min (), ad->get_integer_max ());
           break;

      case CANDB_ATTR_TYPE_HEX:
           fprintf (file, "HEX %u %u", ad->get_hex_min (), ad->get_hex_max ());
           break;

      case CANDB_ATTR_TYPE_FLOAT:
           fprintf (file, "FLOAT ");
           print_value (ad->get_float_min ());
           fputc (' ', file);
           print_value (ad->get_float_max ());
           break;

      case CANDB_ATTR_TYPE_STRING:
           fprintf (file, "STRING ");
           break;

      case CANDB_ATTR_TYPE_INVALID:
           break;

    }
    fputs (";\n", file);

    ad = ad->get_next ();
  }

  // BA_DEF_DEF
  // Default value for the different attributes
  ad = db->get_first_attribute_definition ();
  while (ad) {
    fprintf (file, "BA_DEF_DEF_  ");
    print_string (ad->get_name ());
    fputc (' ', file);
    switch (ad->get_type ()) {
      case CANDB_ATTR_TYPE_ENUMERATION: {
             int vDef = ad->get_enumeration_default();
             const char *enumDef;
             enumDef = ad->get_enumeration_name_by_value (vDef);
             print_string (enumDef);
           }
           break;

      case CANDB_ATTR_TYPE_INTEGER:
           fprintf (file, "%d", ad->get_integer_default ());
           break;

      case CANDB_ATTR_TYPE_HEX:
           fprintf (file, "%u", ad->get_hex_default ());
           break;

      case CANDB_ATTR_TYPE_FLOAT:
           print_value (ad->get_float_default ());
           break;

      case CANDB_ATTR_TYPE_STRING:
           print_string (ad->get_string_default ());
           break;

      case CANDB_ATTR_TYPE_INVALID:
           break;

    }
    fputs (";\n", file);

    ad = ad->get_next ();
  }

  // BA_
  // The attribute values for the different messages and signals.
  msg = db->get_first_message ();
  while (msg) {
    const CANdbAttributeList *al = msg->get_attributes ();
    CANdbAttribute *a = al->get_first_attribute ();
    while (a) {
      fprintf (file, "BA_ ");
      print_string (a->get_definition ()->get_name ());
      fprintf (file, " BO_ %u ", msg->get_id_and_ext ());

      switch (a->get_type ()) {
        case CANDB_ATTR_TYPE_ENUMERATION:
             fprintf (file, "%d", a->get_enumeration_value ());
             break;

        case CANDB_ATTR_TYPE_INTEGER:
             fprintf (file, "%d", a->get_integer_value ());
             break;

        case CANDB_ATTR_TYPE_HEX:
             fprintf (file, "%u", a->get_hex_value ());
             break;

        case CANDB_ATTR_TYPE_FLOAT:
             print_value (a->get_float_value ());
             break;

        case CANDB_ATTR_TYPE_STRING:
             print_string (a->get_string_value ());
             break;

        case CANDB_ATTR_TYPE_INVALID:
             break;

      }
      fputs (";\n", file);
      a = a->get_next ();
    }

    CANdbSignal *s = msg->get_first_signal ();
    while (s) {
      al = s->get_attributes ();
      a = al->get_first_attribute ();
      while (a) {
        fprintf (file, "BA_ ");
        print_string (a->get_definition ()->get_name ());
        fprintf (file, " SG_ %u %s ", msg->get_id_and_ext (), s->get_name ());

        switch (a->get_type ()) {
          case CANDB_ATTR_TYPE_ENUMERATION:
               fprintf (file, "%d", a->get_enumeration_value ());
               break;

          case CANDB_ATTR_TYPE_INTEGER:
               fprintf (file, "%d", a->get_integer_value ());
               break;

          case CANDB_ATTR_TYPE_HEX:
               fprintf (file, "%u", a->get_hex_value ());
               break;

          case CANDB_ATTR_TYPE_FLOAT:
               print_value (a->get_float_value ());
               break;

          case CANDB_ATTR_TYPE_STRING:
               print_string (a->get_string_value ());
               break;

          case CANDB_ATTR_TYPE_INVALID:
               break;

        }
        fputs (";\n", file);
        a = a->get_next ();
      }
      s = s->get_next ();
    }
    msg = msg->get_next ();
  }

  // BA
  // The attribute values for the different nodes
  node = db->get_first_node();
  while (node) {
    const CANdbAttributeList *al = node->get_attributes ();
    CANdbAttribute *a = al->get_first_attribute ();
    while (a) {
      fprintf (file, "BA_ ");
      print_string (a->get_definition ()->get_name ());
      fprintf (file, " BU_ %s ", node->get_name ());
      switch (a->get_type ()) {
        case CANDB_ATTR_TYPE_ENUMERATION:
             fprintf (file, "%d", a->get_enumeration_value ());
             break;

        case CANDB_ATTR_TYPE_INTEGER:
             fprintf (file, "%d", a->get_integer_value ());
             break;

        case CANDB_ATTR_TYPE_HEX:
             fprintf (file, "%u", a->get_hex_value ());
             break;

        case CANDB_ATTR_TYPE_FLOAT:
             print_value (a->get_float_value ());
             break;

        case CANDB_ATTR_TYPE_STRING:
             print_string (a->get_string_value ());
             break;

        case CANDB_ATTR_TYPE_INVALID:
             break;

      }
      fputs (";\n", file);
      a = a->get_next ();
    }
    node = node->get_next ();
  }

  // BA
  // The attribute values for the different environmentvariables
  ev = db->get_first_env_variable();
  while (ev) {
    const CANdbAttributeList *al = ev->get_attributes ();
    CANdbAttribute *a = al->get_first_attribute ();
    while (a) {
      fprintf (file, "BA_ ");
      print_string (a->get_definition ()->get_name ());
      fprintf (file, " EV_ %s ", ev->get_name ());
      switch (a->get_type ()) {
        case CANDB_ATTR_TYPE_ENUMERATION:
             fprintf (file, "%d", a->get_enumeration_value ());
             break;

        case CANDB_ATTR_TYPE_INTEGER:
             fprintf (file, "%d", a->get_integer_value ());
             break;

        case CANDB_ATTR_TYPE_HEX:
             fprintf (file, "%u", a->get_hex_value ());
             break;

        case CANDB_ATTR_TYPE_FLOAT:
             print_value (a->get_float_value ());
             break;

        case CANDB_ATTR_TYPE_STRING:
             print_string (a->get_string_value ());
             break;

        case CANDB_ATTR_TYPE_INVALID:
             break;

      }
      fputs (";\n", file);
      a = a->get_next ();
    }
    ev = ev->get_next ();
  }

  // BA
  // The attribute values for a network
  {
    const CANdbAttributeList *al = db->get_attributes();
    
    CANdbAttribute *a = al->get_first_attribute ();
    while (a) {
      fprintf (file, "BA_ ");
      print_string (a->get_definition ()->get_name ());
      fprintf (file, " ");
      switch (a->get_type ()) {
        case CANDB_ATTR_TYPE_ENUMERATION:
             fprintf (file, "%d", a->get_enumeration_value ());
             break;

        case CANDB_ATTR_TYPE_INTEGER:
             fprintf (file, "%d", a->get_integer_value ());
             break;

        case CANDB_ATTR_TYPE_HEX:
             fprintf (file, "%u", a->get_hex_value ());
             break;

        case CANDB_ATTR_TYPE_FLOAT:
             print_value (a->get_float_value ());
             break;

        case CANDB_ATTR_TYPE_STRING:
             print_string (a->get_string_value ());
             break;

        case CANDB_ATTR_TYPE_INVALID:
             break;

        }
        fputs (";\n", file);
        a = a->get_next ();
    }

  }
  // VAL_
  msg = db->get_first_message ();
  while (msg) {
    CANdbSignal *s = msg->get_first_signal ();
    while (s) {
      CANdbEnumValue *val = s->get_first_value ();
      if (val) {
        fprintf (file, "VAL_ %u %s", msg->get_id_and_ext (), s->get_name ());
        while (val) {
          fprintf (file, " %d ", val->get_value ());
          print_string (val->get_name ());
          val = val->get_next ();
        }
        fputs (" ;\n", file);
      }
      s = s->get_next ();
    }
    msg = msg->get_next ();
  }

  {
    CANdbEnvVariable *var = db->get_first_env_variable ();
    while (var) {
      CANdbEnumValue *val = var->get_first_value ();
      if (val) {
        fprintf (file, "VAL_ %s", var->get_name ());
        while (val) {
          fprintf (file, " %d ", val->get_value ());
          print_string (val->get_name ());
          val = val->get_next ();
        }
        fputs (" ;\n", file);
      }
      var = var->get_next ();
    }
  }
  // CAT_DEF_

  // CAT_

  // SIG_VALTYPE_
  // Types of certain signals are defined here. Iterate over all messages and signals.
  //
  msg = db->get_first_message ();
  while (msg) {
    CANdbSignal *s = msg->get_first_signal ();
    while (s) {
      switch (s->get_type ()) {
        case CANDB_FLOAT:
             fprintf (file,
                      "SIG_VALTYPE_ %u %s : %d;\n",
                      msg->get_id_and_ext (),
                      s->get_name (),
                      1);
             break;

        case CANDB_DOUBLE:
             fprintf (file,
                      "SIG_VALTYPE_ %u %s : %d;\n",
                      msg->get_id_and_ext (),
                      s->get_name (),
                      2);
             break;

        default:
            /* do nothing*/;
      }
      s = s->get_next ();
    }
    msg = msg->get_next ();
  }

finally:
  if (init_locale != NULL) {
    std::setlocale(LC_NUMERIC, init_locale);
  }

  fclose (file);
  return ret_code;
} // CANdbDBC::save_file


CANdbFileIo* CANdbDBC::build ()
{
  CANdbDBC *fio = new CANdbDBC ();
  return fio;
} // CANdbDBC::build


// ****************************************************************************


