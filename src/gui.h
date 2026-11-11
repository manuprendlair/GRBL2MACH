/*
  gui.h - GRBL2MACH status window (Windows)

  Part of GRBL2MACH. GPL-3.0-or-later.

  Launched from a shortcut or by a double-click, the bridge shows a small window instead of its
  console: a status light, the controller and sender state, the problems reported, and a few
  actions (reconnect the controller, load another Mach3 profile, open the log). Everything the
  program prints is captured and also written to a log file next to the program (logs\).
  Launched from a terminal, or with --no-gui, it runs in its console as before.
*/

#ifndef gui_h
#define gui_h

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    Gui_Starting,       // reading the profile, opening the controller
    Gui_Waiting,        // no controller yet: retrying
    Gui_Running,        // the core and the simulation are running
    Gui_Stopped         // the bridge has ended (normally or on an error)
} gui_phase_t;

typedef struct {
    bool (*client_connected)(void);     // a sender is connected
    void (*stop_machine)(void);         // brings the machine to a controlled stop and disarms it
} gui_hooks_t;

#ifdef WIN32

// Called first thing in main. Decides whether the window is wanted (--gui, --no-gui, otherwise
// a console of its own and either no arguments or -u) and, without arguments, substitutes the
// default command line (-u -p 23 -n). Removes the window's own options from argv.
bool gui_wanted (int *argc, char ***argv);
// Opens the window and captures the program's output. Returns false if another bridge window is
// already open (it is brought to the front): the program should end.
bool gui_start (const gui_hooks_t *hooks);
bool gui_active (void);
void gui_set_phase (gui_phase_t phase);
void gui_set_profile (const char *path, bool given);   // given: --mach3 on the command line
void gui_set_listen (const char *addr, unsigned port);
void gui_set_controller (const char *info);
// The bridge has ended with this code: the window stays open until the user closes it.
int gui_finish (int rc);

#else

static inline bool gui_wanted (int *argc, char ***argv) { (void)argc; (void)argv; return false; }
static inline bool gui_start (const gui_hooks_t *hooks) { (void)hooks; return false; }
static inline bool gui_active (void) { return false; }
static inline void gui_set_phase (gui_phase_t phase) { (void)phase; }
static inline void gui_set_profile (const char *path, bool given) { (void)path; (void)given; }
static inline void gui_set_listen (const char *addr, unsigned port) { (void)addr; (void)port; }
static inline void gui_set_controller (const char *info) { (void)info; }
static inline int gui_finish (int rc) { return rc; }

#endif

#endif
