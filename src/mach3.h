/*
  mach3.h - the machine's Mach3 profile (XML) as GRBL2MACH's configuration

  The profile describes the machine: axes, homing, inputs, outputs, spindle. At start-up its
  values are applied to grblHAL's settings and to the bridge's options. Settings changed later
  (from gSender or any other sender) are written back to the profile, which stays the single
  reference. See the mapping table in README.md.

  Part of GRBL2MACH. GPL-3.0-or-later.
*/

#ifndef mach3_h
#define mach3_h

#include <stdbool.h>
#include <stddef.h>

#include "rnr/sink_rnr.h"

#define MACH3_PROFILE_NAME "MACH3.xml"

// Path of MACH3.xml in the directory of the executable
void mach3_default_path (char *path, size_t size, const char *argv0);

// Loads the profile. Returns false (and a message in err) if it is missing or cannot be used.
bool mach3_load (const char *path, char *err, size_t err_size);
bool mach3_loaded (void);

// Bridge options described by the profile: step/dir inversion, inputs, outputs, pulse rate
void mach3_bridge_options (rnr_options_t *opt);

// Hooks the core: applies the profile's settings once the core runs, writes changed settings back
void mach3_attach_core (void);

// To call from the driver's settings_changed handler (core thread)
void mach3_settings_changed (void);

#endif
