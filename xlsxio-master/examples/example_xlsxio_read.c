/*****************************************************************************
Copyright (C)  2016  Brecht Sanders  All Rights Reserved

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*****************************************************************************/

/**
 * @file example_xlsxio_read.c
 * @brief XLSX I/O example illustrating how to read from an .xlsx file.
 * @author Brecht Sanders
 *
 * This example reads data from an .xslx file using the simple method.
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#if _WIN32
#include <windows.h>
#endif
#include "xlsxio_read.h"

const char* filename = "example.xlsx";

int main (int argc, char* argv[])
{
#ifdef _WIN32
  //switch Windows console to UTF-8
  SetConsoleOutputCP(CP_UTF8);
#endif

  xlsxioreader xlsxioread;
  //open .xlsx file for reading
  if ((xlsxioread = xlsxioread_open(filename)) == NULL) {
    fprintf(stderr, "Error opening .xlsx file\n");
    return 1;
  }

  //list available sheets
  xlsxioreadersheetlist sheetlist;
  const char* sheetname;
  printf("Available sheets:\n");
  if ((sheetlist = xlsxioread_sheetlist_open(xlsxioread)) != NULL) {
    while ((sheetname = xlsxioread_sheetlist_next(sheetlist)) != NULL) {
      printf(" - %s\n", sheetname);
    }
    xlsxioread_sheetlist_close(sheetlist);
  }

  //read values from first sheet
  char* value;
  xlsxioreadersheet sheet = xlsxioread_sheet_open(xlsxioread, NULL, XLSXIOREAD_SKIP_EMPTY_ROWS);
  while (xlsxioread_sheet_next_row(sheet)) {
    while ((value = xlsxioread_sheet_next_cell(sheet)) != NULL) {
      printf("%s\t", value);
      free(value);
    }
    printf("\n");
  }
  xlsxioread_sheet_close(sheet);

  //clean up
  xlsxioread_close(xlsxioread);
  return 0;
}
