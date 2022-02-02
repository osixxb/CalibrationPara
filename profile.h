/*********************************************************************
*
* Copyright (C) 1997-2002 Steve Karg
*
* Permission is hereby granted, free of charge, to any person obtaining
* a copy of this software and associated documentation files (the
* "Software"), to deal in the Software without restriction, including
* without limitation the rights to use, copy, modify, merge, publish,
* distribute, sublicense, and/or sell copies of the Software, and to
* permit persons to whom the Software is furnished to do so, subject to
* the following conditions:
*
* The above copyright notice and this permission notice shall be included
* in all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
* IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
* CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
* TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
* SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*
*********************************************************************/
#ifndef PROFILE_H
#define PROFILE_H

#if !defined(_INC_WINDOWS)
  #include <stdio.h> // for size_t

  typedef unsigned char BOOL;

  #ifndef FALSE
    #define FALSE 0
  #endif
  #ifndef TRUE
    #define TRUE 1
  #endif

  #ifndef MAX_LINE_LEN
  #define MAX_LINE_LEN 255
  #endif

  #ifdef __cplusplus
  extern "C" {
  #endif /* __cplusplus */

  BOOL WritePrivateProfileString(
    const char *pAppName,	// pointer to section name
    const char *pKeyName,	// pointer to key name
    const char *pString,	// pointer to string to add
    const char *pFileName); 	// pointer to initialization filename

  size_t GetPrivateProfileString(
    const char *pAppName,	// points to section name
    const char *pKeyName,	// points to key name
    const char *pDefault,	// points to default string
    char *pReturnedString,	// points to destination buffer
    size_t nSize,	// size of destination buffer
    const char *pFileName); 	// points to initialization filename

  extern void rmlead(char *str);
  extern void rmtrail(char *str);
  extern int rmquotes(char *str);
  extern int rmbrackets(char *str);

  char *stptok(const char *s, char *tok, size_t toklen, const char *brk);

  #ifdef __cplusplus  
  }
  #endif /* __cplusplus */

  
#endif // !Windows

#endif
