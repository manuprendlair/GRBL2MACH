/*
  mingw_strset_fix.h - MinGW build workaround (GRBL2MACH)

  MinGW's <string.h> declares the legacy Microsoft function strset(char *, int),
  which clashes with strset(string_t *, size_t, const char *) from the grblHAL
  core (grbl/strutils.h). This header is force-included (-include) in every
  MinGW compilation unit: it pulls in <string.h> first (guarded against
  re-inclusion), then renames the core symbol. No core source is modified.

  Part of GRBL2MACH. GPL-3.0-or-later.
*/
#include <string.h>
#define strset grbl_strset
