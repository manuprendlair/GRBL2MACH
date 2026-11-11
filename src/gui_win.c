/*
  gui_win.c - GRBL2MACH status window (Windows)

  Part of GRBL2MACH. GPL-3.0-or-later.

  Threads: the window has its own thread (lowest priority: it must never slow the bridge
  down), and a reader thread drains the pipe that replaces stdout and stderr. The reader never
  waits for the window: a program line blocked on a full pipe would freeze the core.
*/

#ifndef _WIN32_WINNT
#define _WIN32_WINNT 0x0A00
#endif
#include <windows.h>
#include <commctrl.h>
#include <uxtheme.h>
#include <commdlg.h>
#include <shellapi.h>
#include <io.h>
#include <fcntl.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <wchar.h>

#include "gui.h"
#include "slicer.h"
#include "build_info.h"

#define WINDOW_CLASS    "GRBL2MACH.Window"
#define INSTANCE_MUTEX  "Local\\GRBL2MACH.Bridge"
#define LOG_FILES_KEPT  10
#define MAX_ENTRIES     500

// Core states (same bits as grbl/system.h)
#define ST_ALARM        (1u << 0)
#define ST_CHECK_MODE   (1u << 1)
#define ST_HOMING       (1u << 2)
#define ST_CYCLE        (1u << 3)
#define ST_HOLD         (1u << 4)
#define ST_JOG          (1u << 5)
#define ST_SAFETY_DOOR  (1u << 6)
#define ST_SLEEP        (1u << 7)
#define ST_ESTOP        (1u << 8)
#define ST_TOOL_CHANGE  (1u << 9)
#define ST_MOVING       (ST_HOMING | ST_CYCLE | ST_HOLD | ST_JOG | ST_SAFETY_DOOR | ST_TOOL_CHANGE)

#define WM_APP_LOG      (WM_APP + 1)
#define WM_APP_TRAY     (WM_APP + 2)

enum { ID_LIST = 100, ID_RECONNECT, ID_PROFILE, ID_EDIT, ID_LOG, ID_CLEAR, ID_ALL, ID_ABOUT };
enum { Led_Grey, Led_Green, Led_Orange, Led_Red, Led_Count };
typedef enum { Lvl_Info, Lvl_Warning, Lvl_Problem } level_t;

static const COLORREF led_colors[Led_Count] = { RGB(156, 163, 175), RGB(34, 165, 91), RGB(245, 158, 11), RGB(220, 38, 38) };

typedef struct {
    COLORREF bg, surface, surface_down, text, muted, line, warn, red, accent;
    bool dark;
} palette_t;

static const palette_t light_palette = {
    RGB(255, 255, 255), RGB(243, 244, 246), RGB(229, 231, 235), RGB(17, 24, 39), RGB(107, 114, 128),
    RGB(229, 231, 235), RGB(180, 83, 9), RGB(220, 38, 38), RGB(37, 99, 235), false
};
static const palette_t dark_palette = {
    RGB(32, 33, 36), RGB(48, 50, 54), RGB(63, 66, 71), RGB(232, 234, 237), RGB(154, 160, 166),
    RGB(60, 64, 67), RGB(251, 191, 36), RGB(248, 113, 113), RGB(96, 165, 250), true
};
static const palette_t *pal = &light_palette;

#define COL_TEXT    (pal->text)
#define COL_MUTED   (pal->muted)
#define COL_LINE    (pal->line)
#define COL_BG      (pal->bg)
#define COL_WARN    (pal->warn)
#define COL_RED     (pal->red)

// Theme: follows Windows' app mode unless chosen from the window menu (kept in the registry)
enum { Theme_Auto, Theme_Light, Theme_Dark };
#define SETTINGS_KEY    "Software\\GRBL2MACH"
#define REPO_URL        L"https://github.com/manuprendlair/GRBL2MACH"
// Window menu (system menu) commands: below 0xF000, low 4 bits clear
#define SYS_THEME_AUTO  0x0110
#define SYS_THEME_LIGHT 0x0120
#define SYS_THEME_DARK  0x0130
#define SYS_ABOUT       0x0140

typedef struct {
    char time[16];
    char text[300];
    level_t level;
} entry_t;

// What the window shows, recomputed on every timer tick
typedef struct {
    int led;
    char title[80], detail[300];
    char controller[220], sender[120], machine[120], run[160];
    bool controller_bad, run_bad;
    bool running, moving, can_reconnect;
} view_t;

static bool active;
static gui_hooks_t hooks;
static volatile LONG phase = Gui_Starting;
static int exit_code;
static char **cmd_argv;             // command line in use (restart)
static int cmd_argc;
static DWORD restart_of;            // --restart-of: process to wait for before starting
static char exe_path[MAX_PATH], exe_dir[MAX_PATH];

static CRITICAL_SECTION lock;       // everything below, between the reader and the window
static char profile_path[MAX_PATH], listen_str[80], controller_str[200];
static bool profile_given;
static entry_t entries[MAX_ENTRIES];
static int entry_first, entry_count;
static unsigned entry_serial;       // changes whenever entries change
static unsigned problems_unseen;    // problems since the last Clear: the light stays red
static char last_problem[300];
static char last_run[160];
static bool last_run_bad;
static volatile DWORD reconnect_asked_at;

static char log_path[MAX_PATH];
static FILE *log_file;

static HANDLE window_thread, window_ready;
static HWND hwnd, list, btn_reconnect, btn_profile, btn_edit, btn_log, btn_about, btn_clear, chk_all;
static HFONT font_title, font_text, font_bold;
static int dpi = 96;
static view_t shown;
static unsigned list_serial = ~0u;
static bool list_all, closing;
static NOTIFYICONDATAA tray;
static HICON tray_icons[Led_Count];
static int tray_led = -1;
static unsigned problems_notified;

static int S (int v)
{
    return MulDiv(v, dpi, 96);
}

static void copy_str (char *dst, size_t size, const char *src)
{
    snprintf(dst, size, "%s", src ? src : "");
}

/* --- Command line ----------------------------------------------------------------------------- */

bool gui_wanted (int *argc, char ***argv)
{
    int n = *argc, m = 0, force = -1;
    char **v = *argv, **out = calloc((size_t)n + 6, sizeof(char *));
    bool usb = false;

    if(!out)
        return false;
    out[m++] = v[0];
    for(int i = 1; i < n; i++) {
        if(!strcmp(v[i], "--gui"))
            force = 1;
        else if(!strcmp(v[i], "--no-gui"))
            force = 0;
        else if(!strcmp(v[i], "--restart-of") && i + 1 < n)
            restart_of = strtoul(v[++i], NULL, 10);
        else {
            usb |= !strcmp(v[i], "-u");
            out[m++] = v[i];
        }
    }

    // A console of its own: started from a shortcut or Explorer, not from a terminal
    DWORD pids[2];
    bool own_console = GetConsoleProcessList(pids, 2) <= 1;
    bool want = force >= 0 ? force == 1 : own_console && (m == 1 || usb);

    if(want && m == 1) {
        out[m++] = "-u";
        out[m++] = "-p";
        out[m++] = "23";
        out[m++] = "-n";
    }
    out[m] = NULL;
    if(want && own_console && GetConsoleWindow())
        ShowWindow(GetConsoleWindow(), SW_HIDE);

    cmd_argc = *argc = m;
    cmd_argv = *argv = out;

    return want;
}

/* --- Program output: log file, problem list ---------------------------------------------------- */

static const char *const problem_marks[] = {
    "FAULT", "POSITION MISMATCH", "STARVED", "NO RAMP", "[irq-lock]", "stalled", "' failed",
    "link to the machine lost", "Fatal", "does not answer", "No RNR", "Mach3 profile:", "Error opening",
    "Unable", "cannot write", "position unknown", "Unrecognized option", "mutually exclusive",
    "is not a value", NULL
};
static const char *const warning_marks[] = {
    "WARNING", "e-stop", "client gone", "not at rest in time", "reconnect", "found again", "reachable again",
    "home the machine", "stale", "still engaged", "NOT written", "changed on disk", "Waiting for the RNR", NULL
};

static bool has_mark (const char *line, const char *const marks[])
{
    for(int i = 0; marks[i]; i++)
        if(strstr(line, marks[i]))
            return true;

    return false;
}

// "[rnr] run: ..." or "[rnr] homing X: ..." at the end of each run: kept as the last run's numbers
static bool take_run (const char *line, level_t *level)
{
    const char *q;

    if(strncmp(line, "[rnr] ", 6) || !strstr(line, " slices, ") || !(q = strstr(line, "underruns ")))
        return false;

    unsigned underruns = strtoul(q + 10, NULL, 10);
    double gap = (q = strstr(line, "worst gap ")) ? atof(q + 10) : 0.0;
    char lead[40] = "lead not measured";
    if((q = strstr(line, "lead min ")) && (q = strstr(q, "slices (")))
        snprintf(lead, sizeof(lead), "lead min %.0f ms", atof(q + 8));

    SYSTEMTIME t;
    GetLocalTime(&t);
    snprintf(last_run, sizeof(last_run), "%s, %u underrun%s, USB worst gap %.1f ms (%02u:%02u)",
             lead, underruns, underruns == 1 ? "" : "s", gap, t.wHour, t.wMinute);
    last_run_bad = underruns > 0;
    if(underruns > 0)
        *level = Lvl_Problem;

    return true;
}

static void take_line (const char *line)
{
    SYSTEMTIME t;
    level_t level = Lvl_Info;

    if(!*line)
        return;
    GetLocalTime(&t);
    if(log_file) {
        fprintf(log_file, "%04u-%02u-%02u %02u:%02u:%02u.%03u %s\n", t.wYear, t.wMonth, t.wDay,
                t.wHour, t.wMinute, t.wSecond, t.wMilliseconds, line);
        fflush(log_file);
    }

    if(has_mark(line, problem_marks))
        level = Lvl_Problem;
    else if(has_mark(line, warning_marks))
        level = Lvl_Warning;
    // The link loss that follows a reconnection asked from the window is expected
    if(level == Lvl_Problem && strstr(line, "link to the machine lost") && GetTickCount() - reconnect_asked_at < 5000)
        level = Lvl_Warning;

    EnterCriticalSection(&lock);
    take_run(line, &level);
    entry_t *e = &entries[(entry_first + entry_count) % MAX_ENTRIES];
    if(entry_count < MAX_ENTRIES)
        entry_count++;
    else
        entry_first = (entry_first + 1) % MAX_ENTRIES;
    snprintf(e->time, sizeof(e->time), "%02u:%02u:%02u", t.wHour, t.wMinute, t.wSecond);
    copy_str(e->text, sizeof(e->text), line);
    e->level = level;
    entry_serial++;
    if(level == Lvl_Problem) {
        problems_unseen++;
        copy_str(last_problem, sizeof(last_problem), line);
    }
    LeaveCriticalSection(&lock);

    if(hwnd)
        PostMessageA(hwnd, WM_APP_LOG, 0, 0);
}

static DWORD WINAPI reader_main (LPVOID arg)
{
    HANDLE rd = arg;
    char buf[1024], line[600];
    size_t len = 0;
    DWORD got;

    while(ReadFile(rd, buf, sizeof(buf), &got, NULL) && got) {
        for(DWORD i = 0; i < got; i++) {
            char c = buf[i];
            if(c == '\r')
                continue;
            if(c == '\n' || len == sizeof(line) - 1) {
                line[len] = 0;
                take_line(line);
                len = 0;
                if(c == '\n')
                    continue;
            }
            line[len++] = c;
        }
    }

    return 0;
}

static int compare_names (const void *a, const void *b)
{
    return strcmp((const char *)a, (const char *)b);
}

// logs\grbl2mach-<date>-<time>.log next to the program, the oldest ones removed
static void open_log (void)
{
    char dir[MAX_PATH], pattern[MAX_PATH], (*names)[MAX_PATH] = NULL;
    int count = 0, size = 0;
    WIN32_FIND_DATAA fd;
    SYSTEMTIME t;

    snprintf(dir, sizeof(dir), "%slogs", exe_dir);
    CreateDirectoryA(dir, NULL);
    snprintf(pattern, sizeof(pattern), "%s\\grbl2mach-*.log", dir);
    HANDLE h = FindFirstFileA(pattern, &fd);
    if(h != INVALID_HANDLE_VALUE) {
        do {
            if(count == size) {
                void *more = realloc(names, (size_t)(size = size ? size * 2 : 16) * MAX_PATH);
                if(!more)
                    break;
                names = more;
            }
            copy_str(names[count++], MAX_PATH, fd.cFileName);
        } while(FindNextFileA(h, &fd));
        FindClose(h);
    }
    if(names) {
        qsort(names, (size_t)count, MAX_PATH, compare_names);
        for(int i = 0; i < count - (LOG_FILES_KEPT - 1); i++) {
            char old[MAX_PATH * 2];
            snprintf(old, sizeof(old), "%s\\%s", dir, names[i]);
            DeleteFileA(old);
        }
        free(names);
    }

    GetLocalTime(&t);
    snprintf(log_path, sizeof(log_path), "%s\\grbl2mach-%04u%02u%02u-%02u%02u%02u.log", dir,
             t.wYear, t.wMonth, t.wDay, t.wHour, t.wMinute, t.wSecond);
    log_file = fopen(log_path, "w");
}

// stdout and stderr go to a pipe drained by the reader thread
static void capture_output (void)
{
    SECURITY_ATTRIBUTES sa = { sizeof(sa), NULL, FALSE };
    HANDLE rd, wr;

    if(!CreatePipe(&rd, &wr, &sa, 1 << 16))
        return;
    fflush(stdout);
    fflush(stderr);
    FreeConsole();
    int fd = _open_osfhandle((intptr_t)wr, _O_TEXT | _O_WRONLY);
    _dup2(fd, 1);
    _dup2(fd, 2);
    SetStdHandle(STD_OUTPUT_HANDLE, (HANDLE)_get_osfhandle(1));
    SetStdHandle(STD_ERROR_HANDLE, (HANDLE)_get_osfhandle(2));
    setvbuf(stdout, NULL, _IONBF, 0);
    setvbuf(stderr, NULL, _IONBF, 0);
    CloseHandle(CreateThread(NULL, 0, reader_main, rd, 0, NULL));
}

/* --- Status ------------------------------------------------------------------------------------ */

static const char *alarm_text (uint32_t code)
{
    switch(code) {
        case 1: return "hard limit";
        case 2: return "soft limit";
        case 3: return "reset during motion, position lost";
        case 4: case 5: case 13: return "probing failed";
        case 6: case 7: case 8: case 9: case 18: return "homing failed";
        case 10: return "emergency stop";
        case 11: return "homing required";
        case 12: return "limit switch engaged";
        case 17: return "controller link lost or controller fault";
        default: return "";
    }
}

static void state_text (uint32_t st, char *buf, size_t size)
{
    if(st & ST_ESTOP)
        copy_str(buf, size, "Emergency stop");
    else if(st & ST_ALARM) {
        uint32_t code = slicer_core_alarm();
        const char *why = alarm_text(code);
        snprintf(buf, size, "Alarm %u%s%s", code, *why ? ": " : "", why);
    } else if(st & ST_SAFETY_DOOR)
        copy_str(buf, size, "Safety door open");
    else if(st & ST_TOOL_CHANGE)
        copy_str(buf, size, "Tool change");
    else if(st & ST_HOLD)
        copy_str(buf, size, "Feed hold");
    else if(st & ST_HOMING)
        copy_str(buf, size, "Homing");
    else if(st & ST_JOG)
        copy_str(buf, size, "Jogging");
    else if(st & ST_CYCLE)
        copy_str(buf, size, "Running a job");
    else if(st & ST_CHECK_MODE)
        copy_str(buf, size, "Check mode");
    else if(st & ST_SLEEP)
        copy_str(buf, size, "Sleep");
    else
        copy_str(buf, size, "Idle");
}

static void compute_view (view_t *v)
{
    char ctrl[200], listen[80], problem[300];
    unsigned unseen;

    memset(v, 0, sizeof(*v));
    EnterCriticalSection(&lock);
    copy_str(ctrl, sizeof(ctrl), controller_str[0] ? controller_str : "RNR Eco Motion");
    copy_str(listen, sizeof(listen), listen_str);
    copy_str(problem, sizeof(problem), last_problem);
    copy_str(v->run, sizeof(v->run), last_run[0] ? last_run : "none yet");
    v->run_bad = last_run_bad;
    unseen = problems_unseen;
    LeaveCriticalSection(&lock);

    gui_phase_t ph = (gui_phase_t)phase;
    copy_str(v->machine, sizeof(v->machine), "-");
    copy_str(v->sender, sizeof(v->sender), "-");

    if(closing) {
        v->led = Led_Grey;
        copy_str(v->title, sizeof(v->title), "Stopping");
        copy_str(v->detail, sizeof(v->detail), "Bringing the machine to a stop and disarming the controller.");
        copy_str(v->controller, sizeof(v->controller), ctrl);
        return;
    }

    switch(ph) {

        case Gui_Starting:
            v->led = Led_Grey;
            copy_str(v->title, sizeof(v->title), "Starting");
            copy_str(v->detail, sizeof(v->detail), "Reading the Mach3 profile and opening the controller.");
            copy_str(v->controller, sizeof(v->controller), "opening");
            return;

        case Gui_Waiting:
            v->led = Led_Orange;
            copy_str(v->title, sizeof(v->title), "Waiting for the controller");
            copy_str(v->detail, sizeof(v->detail), "Plug in the RNR Eco Motion board and power it on. GRBL2MACH tries again every second.");
            copy_str(v->controller, sizeof(v->controller), "not found yet");
            v->controller_bad = true;
            return;

        case Gui_Stopped:
            v->led = exit_code ? Led_Red : Led_Grey;
            copy_str(v->title, sizeof(v->title), exit_code ? "Stopped on an error" : "Stopped");
            if(exit_code && problem[0])
                copy_str(v->detail, sizeof(v->detail), problem);
            else
                snprintf(v->detail, sizeof(v->detail), "GRBL2MACH has stopped (exit code %d). Close this window or load a profile to start again.", exit_code);
            copy_str(v->controller, sizeof(v->controller), "closed");
            return;

        case Gui_Running:
            break;
    }

    uint32_t st = slicer_core_state();
    bool link = slicer_link_ok(), fault = slicer_motor_fault(), estop = slicer_estop() || (st & ST_ESTOP);
    bool client = hooks.client_connected && hooks.client_connected();
    char state[120];

    state_text(st, state, sizeof(state));
    v->running = true;
    v->moving = (st & ST_MOVING) != 0;
    v->can_reconnect = slicer_can_reconnect() && !v->moving;
    copy_str(v->machine, sizeof(v->machine), state);
    if(client)
        copy_str(v->sender, sizeof(v->sender), "connected");
    else
        snprintf(v->sender, sizeof(v->sender), "waiting on %s", listen);
    v->controller_bad = !link || fault;
    snprintf(v->controller, sizeof(v->controller), "%s - %s", ctrl,
             !link ? "not reachable, retrying every second" : fault ? "disarmed after a fault" : "connected");

    if(!link) {
        v->led = Led_Red;
        copy_str(v->title, sizeof(v->title), "Controller not reachable");
        copy_str(v->detail, sizeof(v->detail), "Check the USB cable and the board's power. GRBL2MACH reconnects by itself.");
    } else if(fault) {
        v->led = Led_Red;
        copy_str(v->title, sizeof(v->title), "Controller fault");
        copy_str(v->detail, sizeof(v->detail), "The controller was disarmed. Press Reconnect controller, then home the machine.");
    } else if(estop) {
        v->led = Led_Red;
        copy_str(v->title, sizeof(v->title), "Emergency stop");
        copy_str(v->detail, sizeof(v->detail), "Release the e-stop, then reset and unlock from the sender.");
    } else if(unseen) {
        v->led = Led_Red;
        copy_str(v->title, sizeof(v->title), unseen > 1 ? "Problems reported" : "Problem reported");
        copy_str(v->detail, sizeof(v->detail), problem);
    } else if(st & ST_ALARM) {
        v->led = Led_Orange;
        copy_str(v->title, sizeof(v->title), "Alarm");
        snprintf(v->detail, sizeof(v->detail), "%s. Unlock or home the machine from the sender.", state);
    } else if(st & (ST_HOLD | ST_SAFETY_DOOR | ST_TOOL_CHANGE)) {
        v->led = Led_Orange;
        copy_str(v->title, sizeof(v->title), state);
        copy_str(v->detail, sizeof(v->detail), "Resume or stop from the sender.");
    } else {
        v->led = Led_Green;
        copy_str(v->title, sizeof(v->title), st == 0 ? "Ready" : state);
        if(client)
            copy_str(v->detail, sizeof(v->detail), "Sender connected.");
        else
            snprintf(v->detail, sizeof(v->detail), "Connect your sender (gSender: Ethernet) to %s.", listen);
    }
}

/* --- Drawing ----------------------------------------------------------------------------------- */

// Anti-aliased status light: a solid disc in a pale halo. alpha: icon pixels (transparent
// background) instead of pixels blended on bg.
static void render_led (uint32_t *px, int size, COLORREF col, bool halo, bool alpha, COLORREF bg)
{
    double c = size / 2.0, outer = size / 2.0 - 0.5, inner = halo ? outer * 0.62 : outer;
    int r = GetRValue(col), g = GetGValue(col), b = GetBValue(col);

    for(int y = 0; y < size; y++) {
        for(int x = 0; x < size; x++) {
            double d = hypot(x + 0.5 - c, y + 0.5 - c);
            double ci = inner - d + 0.5, co = outer - d + 0.5;
            ci = ci < 0 ? 0 : ci > 1 ? 1 : ci;
            co = co < 0 ? 0 : co > 1 ? 1 : co;
            double a = ci + (halo ? co * 0.22 * (1 - ci) : 0);
            uint32_t *p = &px[(size - 1 - y) * size + x];   // bottom-up DIB
            if(alpha)
                *p = ((uint32_t)(a * 255 + 0.5) << 24) | ((uint32_t)r << 16) | ((uint32_t)g << 8) | (uint32_t)b;
            else {
                int rr = (int)(r * a + GetRValue(bg) * (1 - a) + 0.5), gg = (int)(g * a + GetGValue(bg) * (1 - a) + 0.5),
                    bb = (int)(b * a + GetBValue(bg) * (1 - a) + 0.5);
                *p = ((uint32_t)rr << 16) | ((uint32_t)gg << 8) | (uint32_t)bb;
            }
        }
    }
}

static HBITMAP led_bitmap (HDC dc, int size, COLORREF col, bool halo, bool alpha, COLORREF bg)
{
    BITMAPINFO bi = { .bmiHeader = { sizeof(BITMAPINFOHEADER), size, size, 1, 32, BI_RGB } };
    void *bits;
    HBITMAP bmp = CreateDIBSection(dc, &bi, DIB_RGB_COLORS, &bits, NULL, 0);

    if(bmp)
        render_led(bits, size, col, halo, alpha, bg);

    return bmp;
}

static HICON make_tray_icon (int led)
{
    int size = GetSystemMetrics(SM_CXSMICON);
    HDC dc = GetDC(NULL);
    ICONINFO ii = { TRUE, 0, 0, CreateBitmap(size, size, 1, 1, NULL), led_bitmap(dc, size, led_colors[led], false, true, 0) };
    HICON icon = CreateIconIndirect(&ii);

    ReleaseDC(NULL, dc);
    DeleteObject(ii.hbmMask);
    DeleteObject(ii.hbmColor);

    return icon;
}

static void draw_text (HDC dc, HFONT font, COLORREF col, const char *s, int x, int y, int w, int h, UINT flags)
{
    RECT r = { x, y, x + w, y + h };

    SelectObject(dc, font);
    SetTextColor(dc, col);
    DrawTextA(dc, s, -1, &r, flags | DT_NOPREFIX | DT_SINGLELINE | DT_VCENTER);
}

static void fill (HDC dc, int x, int y, int w, int h, COLORREF col)
{
    RECT r = { x, y, x + w, y + h };
    HBRUSH br = CreateSolidBrush(col);

    FillRect(dc, &r, br);
    DeleteObject(br);
}

// Layout, in pixels at 96 dpi
#define HEADER_H    84
#define ROWS_Y      96
#define ROW_H       24
#define ROWS        5
#define SECTION_Y   (ROWS_Y + ROWS * ROW_H + 12)
#define LIST_Y      (SECTION_Y + 32)
#define FOOTER_H    56
#define MARGIN      20
#define LABEL_W     96

static void paint (HDC dc, int w, int h)
{
    const view_t *v = &shown;
    int led = S(30), tx = S(MARGIN) + led + S(14);

    fill(dc, 0, 0, w, h, COL_BG);
    SetBkMode(dc, TRANSPARENT);

    // Header: light, state, detail
    HDC mem = CreateCompatibleDC(dc);
    HBITMAP bmp = led_bitmap(dc, led, led_colors[v->led], true, false, COL_BG);
    HGDIOBJ old = SelectObject(mem, bmp);
    BitBlt(dc, S(MARGIN), (S(HEADER_H) - led) / 2, led, led, mem, 0, 0, SRCCOPY);
    SelectObject(mem, old);
    DeleteObject(bmp);
    DeleteDC(mem);
    draw_text(dc, font_title, COL_TEXT, v->title, tx, S(14), w - tx - S(MARGIN), S(30), DT_END_ELLIPSIS);
    draw_text(dc, font_text, v->led == Led_Red ? COL_RED : COL_MUTED, v->detail, tx, S(44), w - tx - S(MARGIN), S(22), DT_END_ELLIPSIS);
    fill(dc, 0, S(HEADER_H), w, 1, COL_LINE);

    // Rows
    struct { const char *label, *value; COLORREF col; UINT flags; } rows[ROWS] = {
        { "Controller", v->controller, v->controller_bad ? COL_RED : COL_TEXT, DT_END_ELLIPSIS },
        { "Sender", v->sender, COL_TEXT, DT_END_ELLIPSIS },
        { "Machine", v->machine, COL_TEXT, DT_END_ELLIPSIS },
        { "Profile", profile_path[0] ? profile_path : "-", COL_TEXT, DT_PATH_ELLIPSIS },
        { "Last run", v->run, v->run_bad ? COL_RED : COL_TEXT, DT_END_ELLIPSIS },
    };
    for(int i = 0; i < ROWS; i++) {
        int y = S(ROWS_Y + i * ROW_H);
        draw_text(dc, font_text, COL_MUTED, rows[i].label, S(MARGIN), y, S(LABEL_W), S(ROW_H), 0);
        draw_text(dc, font_text, rows[i].col, rows[i].value, S(MARGIN + LABEL_W), y, w - S(2 * MARGIN + LABEL_W), S(ROW_H), rows[i].flags);
    }

    draw_text(dc, font_bold, COL_TEXT, "Messages", S(MARGIN), S(SECTION_Y), S(200), S(26), 0);

    // Frame around the list, line above the buttons
    RECT lr;
    GetWindowRect(list, &lr);
    MapWindowPoints(NULL, hwnd, (POINT *)&lr, 2);
    fill(dc, lr.left - 1, lr.top - 1, lr.right - lr.left + 2, 1, COL_LINE);
    fill(dc, lr.left - 1, lr.bottom, lr.right - lr.left + 2, 1, COL_LINE);
    fill(dc, lr.left - 1, lr.top - 1, 1, lr.bottom - lr.top + 2, COL_LINE);
    fill(dc, lr.right, lr.top - 1, 1, lr.bottom - lr.top + 2, COL_LINE);
}

static void layout (int w, int h)
{
    int bh = S(30), by = h - S(FOOTER_H) + (S(FOOTER_H) - bh) / 2, x = S(MARGIN);
    HWND buttons[] = { btn_reconnect, btn_profile, btn_edit, btn_log };
    int widths[] = { 160, 120, 100, 90 };

    for(int i = 0; i < 4; i++) {
        MoveWindow(buttons[i], x, by, S(widths[i]), bh, TRUE);
        x += S(widths[i]) + S(8);
    }
    MoveWindow(btn_about, w - S(MARGIN) - S(80), by, S(80), bh, TRUE);
    MoveWindow(btn_clear, w - S(MARGIN) - S(80), S(SECTION_Y), S(80), S(26), TRUE);
    MoveWindow(chk_all, w - S(MARGIN) - S(80) - S(8) - S(120), S(SECTION_Y), S(120), S(26), TRUE);
    int lh = h - S(FOOTER_H) - S(LIST_Y);
    MoveWindow(list, S(MARGIN) + 1, S(LIST_Y) + 1, w - S(2 * MARGIN) - 2, lh > S(40) ? lh : S(40), TRUE);

    ListView_SetColumnWidth(list, 0, LVSCW_AUTOSIZE_USEHEADER);    // the client width, scroll bar or not
}

static void fill_list (void)
{
    LVITEMA it = { 0 };
    char text[340];
    int row = 0;

    SendMessageA(list, WM_SETREDRAW, FALSE, 0);
    ListView_DeleteAllItems(list);
    EnterCriticalSection(&lock);
    list_serial = entry_serial;
    for(int i = entry_count - 1; i >= 0; i--) {     // newest first
        entry_t *e = &entries[(entry_first + i) % MAX_ENTRIES];
        if(!list_all && e->level == Lvl_Info)
            continue;
        snprintf(text, sizeof(text), "%s    %s", e->time, e->text);
        it.mask = LVIF_TEXT | LVIF_PARAM;
        it.iItem = row++;
        it.pszText = text;
        it.lParam = e->level;
        ListView_InsertItem(list, &it);
    }
    LeaveCriticalSection(&lock);
    if(!row) {
        it.mask = LVIF_TEXT | LVIF_PARAM;
        it.iItem = 0;
        it.pszText = (char *)(list_all ? "No message yet." : "No problem or warning reported.");
        it.lParam = -1;
        ListView_InsertItem(list, &it);
    }
    ListView_SetColumnWidth(list, 0, LVSCW_AUTOSIZE_USEHEADER);
    SendMessageA(list, WM_SETREDRAW, TRUE, 0);
    InvalidateRect(list, NULL, TRUE);
}

/* --- Actions ----------------------------------------------------------------------------------- */

static void append_arg (char *cmd, size_t size, const char *arg)
{
    size_t len = strlen(cmd);
    bool quote = !*arg || strpbrk(arg, " \t");

    snprintf(cmd + len, size - len, quote ? " \"%s\"" : " %s", arg);
}

static bool spawn (const char *cmd, HANDLE out, PROCESS_INFORMATION *pi)
{
    SECURITY_ATTRIBUTES sa = { sizeof(sa), NULL, TRUE };
    HANDLE nul = CreateFileA("NUL", GENERIC_READ | GENERIC_WRITE, FILE_SHARE_READ | FILE_SHARE_WRITE, &sa, OPEN_EXISTING, 0, NULL);
    STARTUPINFOEXA si = { .StartupInfo = { .cb = sizeof(si), .dwFlags = STARTF_USESTDHANDLES } };
    HANDLE inherit[2] = { nul, out };
    SIZE_T size = 0;
    char *line = _strdup(cmd);
    bool ok = false;

    // Only these handles go to the child: not the listening socket, which would keep the port
    // taken after this process ends
    si.StartupInfo.hStdInput = nul;
    si.StartupInfo.hStdOutput = si.StartupInfo.hStdError = out ? out : nul;
    InitializeProcThreadAttributeList(NULL, 1, 0, &size);
    si.lpAttributeList = malloc(size);
    if(line && si.lpAttributeList && InitializeProcThreadAttributeList(si.lpAttributeList, 1, 0, &size)) {
        if(UpdateProcThreadAttribute(si.lpAttributeList, 0, PROC_THREAD_ATTRIBUTE_HANDLE_LIST, inherit,
                                     (out ? 2 : 1) * sizeof(HANDLE), NULL, NULL))
            ok = CreateProcessA(exe_path, line, NULL, NULL, TRUE, CREATE_NO_WINDOW | EXTENDED_STARTUPINFO_PRESENT,
                                NULL, exe_dir, &si.StartupInfo, pi);
        DeleteProcThreadAttributeList(si.lpAttributeList);
    }
    free(si.lpAttributeList);
    free(line);
    CloseHandle(nul);

    return ok;
}

// Runs "grbl2mach --check-mach3 <file>": the profile is read by a fresh process, this one keeps
// its own. Returns true if it can be used, else the reason in msg.
static bool check_profile (const char *file, char *msg, size_t size)
{
    SECURITY_ATTRIBUTES sa = { sizeof(sa), NULL, TRUE };
    PROCESS_INFORMATION pi;
    HANDLE rd, wr;
    char cmd[MAX_PATH * 3] = "", out[4096] = "";
    DWORD got, total = 0, code = 1;

    snprintf(msg, size, "The profile could not be checked.");
    append_arg(cmd, sizeof(cmd), exe_path);
    append_arg(cmd, sizeof(cmd), "--no-gui");
    append_arg(cmd, sizeof(cmd), "--check-mach3");
    append_arg(cmd, sizeof(cmd), file);
    if(!CreatePipe(&rd, &wr, &sa, 0))
        return false;
    SetHandleInformation(rd, HANDLE_FLAG_INHERIT, 0);
    bool started = spawn(cmd, wr, &pi);
    CloseHandle(wr);
    if(started) {
        while(total < sizeof(out) - 1 && ReadFile(rd, out + total, sizeof(out) - 1 - total, &got, NULL) && got)
            total += got;
        out[total] = 0;
        WaitForSingleObject(pi.hProcess, 10000);
        GetExitCodeProcess(pi.hProcess, &code);
        CloseHandle(pi.hProcess);
        CloseHandle(pi.hThread);
        // Last line printed: "Profile OK" or the reason
        char *end = out + strlen(out);
        while(end > out && (end[-1] == '\n' || end[-1] == '\r'))
            *--end = 0;
        char *last = strrchr(out, '\n');
        copy_str(msg, size, last ? last + 1 : out);
    }
    CloseHandle(rd);

    return started && code == 0;
}

// Starts a new bridge (it waits for this one to end) and ends this one, the machine stopped
static void restart (const char *new_profile)
{
    char cmd[4096] = "";
    PROCESS_INFORMATION pi;
    char pid[16];

    append_arg(cmd, sizeof(cmd), exe_path);
    append_arg(cmd, sizeof(cmd), "--gui");
    append_arg(cmd, sizeof(cmd), "--restart-of");
    snprintf(pid, sizeof(pid), "%lu", (unsigned long)GetCurrentProcessId());
    append_arg(cmd, sizeof(cmd), pid);
    for(int i = 1; i < cmd_argc; i++) {
        if(new_profile && profile_given && !strcmp(cmd_argv[i], "--mach3") && i + 1 < cmd_argc) {
            i++;
            continue;
        }
        append_arg(cmd, sizeof(cmd), cmd_argv[i]);
    }

    if(new_profile && profile_given) {
        append_arg(cmd, sizeof(cmd), "--mach3");
        append_arg(cmd, sizeof(cmd), new_profile);
    } else if(new_profile && _stricmp(new_profile, profile_path)) {
        // The profile next to the program is replaced, the previous one kept as .bak
        char bak[MAX_PATH + 8];
        snprintf(bak, sizeof(bak), "%s.bak", profile_path);
        CopyFileA(profile_path, bak, FALSE);
        if(!CopyFileA(new_profile, profile_path, FALSE)) {
            MessageBoxA(hwnd, "The profile could not be copied next to the program.", "GRBL2MACH", MB_ICONERROR);
            return;
        }
    }

    if(!spawn(cmd, NULL, &pi)) {
        MessageBoxA(hwnd, "GRBL2MACH could not be started again.", "GRBL2MACH", MB_ICONERROR);
        return;
    }
    CloseHandle(pi.hProcess);
    CloseHandle(pi.hThread);

    closing = true;
    compute_view(&shown);
    RedrawWindow(hwnd, NULL, NULL, RDW_INVALIDATE | RDW_UPDATENOW);
    Shell_NotifyIconA(NIM_DELETE, &tray);
    if(phase == Gui_Running && hooks.stop_machine)
        hooks.stop_machine();
    exit(EXIT_SUCCESS);
}

static void on_load_profile (void)
{
    char file[MAX_PATH] = "", msg[400], text[700];
    OPENFILENAMEA ofn = { .lStructSize = sizeof(ofn), .hwndOwner = hwnd, .lpstrFilter = "Mach3 profile (*.xml)\0*.xml\0All files\0*.*\0",
                          .lpstrFile = file, .nMaxFile = sizeof(file), .lpstrTitle = "Load a Mach3 profile",
                          .Flags = OFN_FILEMUSTEXIST | OFN_PATHMUSTEXIST | OFN_HIDEREADONLY | OFN_NOCHANGEDIR };

    compute_view(&shown);
    if(shown.moving) {
        MessageBoxA(hwnd, "The machine is moving. Stop the job from the sender first.", "GRBL2MACH", MB_ICONINFORMATION);
        return;
    }
    if(!GetOpenFileNameA(&ofn))
        return;
    if(!check_profile(file, msg, sizeof(msg))) {
        snprintf(text, sizeof(text), "This profile cannot be used:\n\n%s", msg);
        MessageBoxA(hwnd, text, "GRBL2MACH", MB_ICONERROR);
        return;
    }
    snprintf(text, sizeof(text), "GRBL2MACH will restart with this profile%s.%s\n\n%s",
             profile_given ? "" : " (the current one is kept as MACH3.xml.bak)",
             phase == Gui_Running ? " Your sender will have to connect again." : "", file);
    if(MessageBoxA(hwnd, text, "GRBL2MACH", MB_OKCANCEL | MB_ICONQUESTION) == IDOK)
        restart(file);
}

static void on_reconnect (void)
{
    if(MessageBoxA(hwnd, "Close the link to the controller and open it again?\n\n"
                   "The machine position will be lost: home the machine afterwards.",
                   "GRBL2MACH", MB_OKCANCEL | MB_ICONQUESTION) != IDOK)
        return;
    reconnect_asked_at = GetTickCount();
    if(!slicer_reconnect())
        MessageBoxA(hwnd, "Not now: the machine is moving.", "GRBL2MACH", MB_ICONINFORMATION);
}

static void editor_path (char *path, size_t size)
{
    snprintf(path, size, "%smach3_editor.html", exe_dir);
}

static void on_close (void)
{
    if(phase == Gui_Stopped) {
        DestroyWindow(hwnd);
        return;
    }
    compute_view(&shown);
    if(shown.moving && MessageBoxA(hwnd, "A job or a move is in progress.\n\nStop the machine and close GRBL2MACH?",
                                   "GRBL2MACH", MB_YESNO | MB_ICONWARNING | MB_DEFBUTTON2) != IDYES)
        return;
    closing = true;
    compute_view(&shown);
    RedrawWindow(hwnd, NULL, NULL, RDW_INVALIDATE | RDW_UPDATENOW);
    Shell_NotifyIconA(NIM_DELETE, &tray);
    if(phase == Gui_Running && hooks.stop_machine)
        hooks.stop_machine();
    exit(EXIT_SUCCESS);
}

/* --- Theme, buttons, About --------------------------------------------------------------------- */

static int theme_choice = Theme_Auto;
static HBRUSH bg_brush;

static int read_theme_choice (void)
{
    DWORD v = Theme_Auto, size = sizeof(v);

    if(RegGetValueA(HKEY_CURRENT_USER, SETTINGS_KEY, "Theme", RRF_RT_REG_DWORD, NULL, &v, &size) != ERROR_SUCCESS || v > Theme_Dark)
        v = Theme_Auto;

    return (int)v;
}

static bool windows_uses_dark (void)
{
    DWORD v = 1, size = sizeof(v);

    RegGetValueA(HKEY_CURRENT_USER, "Software\\Microsoft\\Windows\\CurrentVersion\\Themes\\Personalize",
                 "AppsUseLightTheme", RRF_RT_REG_DWORD, NULL, &v, &size);

    return v == 0;
}

// Dark title bar (Windows 10 1809 and later; attribute 19 before 20H1)
static void dark_title_bar (HWND h, BOOL dark)
{
    typedef HRESULT (WINAPI *set_attr_t)(HWND, DWORD, LPCVOID, DWORD);
    static set_attr_t set_attr;
    static bool looked_up;

    if(!looked_up) {
        HMODULE dwm = LoadLibraryA("dwmapi.dll");
        set_attr = dwm ? (set_attr_t)(void *)GetProcAddress(dwm, "DwmSetWindowAttribute") : NULL;
        looked_up = true;
    }
    if(set_attr && set_attr(h, 20, &dark, sizeof(dark)) != S_OK)
        set_attr(h, 19, &dark, sizeof(dark));
    // Windows 10 only repaints the title bar with its new colours on an activation change
    BOOL is_active = GetActiveWindow() == h;
    SendMessageA(h, WM_NCACTIVATE, !is_active, 0);
    SendMessageA(h, WM_NCACTIVATE, is_active, 0);
}

static void apply_theme (void)
{
    bool dark = theme_choice == Theme_Dark || (theme_choice == Theme_Auto && windows_uses_dark());

    pal = dark ? &dark_palette : &light_palette;
    if(bg_brush)
        DeleteObject(bg_brush);
    bg_brush = CreateSolidBrush(pal->bg);
    dark_title_bar(hwnd, dark);
    SetWindowTheme(list, dark ? L"DarkMode_Explorer" : L"Explorer", NULL);
    ListView_SetBkColor(list, pal->bg);
    ListView_SetTextBkColor(list, pal->bg);

    HMENU menu = GetSystemMenu(hwnd, FALSE);
    CheckMenuRadioItem(menu, SYS_THEME_AUTO, SYS_THEME_DARK, SYS_THEME_AUTO + 0x10 * theme_choice, MF_BYCOMMAND);
    RedrawWindow(hwnd, NULL, NULL, RDW_INVALIDATE | RDW_ERASE | RDW_FRAME | RDW_ALLCHILDREN);
}

static void set_theme_choice (int choice)
{
    DWORD v = (DWORD)choice;

    theme_choice = choice;
    RegSetKeyValueA(HKEY_CURRENT_USER, SETTINGS_KEY, "Theme", REG_DWORD, &v, sizeof(v));
    apply_theme();
}

// Flat rounded buttons, drawn the same way in both themes. "All messages" is a toggle.
static void draw_button (const DRAWITEMSTRUCT *di)
{
    HDC dc = di->hDC;
    RECT r = di->rcItem;
    bool down = di->itemState & ODS_SELECTED, disabled = di->itemState & ODS_DISABLED;
    bool focus = (di->itemState & ODS_FOCUS) && !(di->itemState & ODS_NOFOCUSRECT);
    bool on = di->CtlID == ID_ALL && list_all;
    char text[64];

    FillRect(dc, &r, bg_brush);
    HBRUSH face = CreateSolidBrush(on ? pal->accent : down ? pal->surface_down : pal->surface);
    HPEN pen = CreatePen(PS_SOLID, 1, on || focus ? pal->accent : pal->line);
    HGDIOBJ old_brush = SelectObject(dc, face), old_pen = SelectObject(dc, pen);
    RoundRect(dc, r.left, r.top, r.right, r.bottom, S(10), S(10));
    SelectObject(dc, old_brush);
    SelectObject(dc, old_pen);
    DeleteObject(face);
    DeleteObject(pen);

    GetWindowTextA(di->hwndItem, text, sizeof(text));
    SetBkMode(dc, TRANSPARENT);
    SelectObject(dc, font_text);
    SetTextColor(dc, disabled ? pal->muted : on ? RGB(255, 255, 255) : pal->text);
    DrawTextA(dc, text, -1, &r, DT_CENTER | DT_VCENTER | DT_SINGLELINE | DT_NOPREFIX);
}

static HRESULT CALLBACK about_callback (HWND h, UINT note, WPARAM wp, LPARAM lp, LONG_PTR ref)
{
    (void)wp;
    (void)ref;
    if(note == TDN_HYPERLINK_CLICKED)
        ShellExecuteW(h, L"open", (LPCWSTR)lp, NULL, NULL, SW_SHOWNORMAL);

    return S_OK;
}

static void on_about (void)
{
    typedef HRESULT (WINAPI *task_dialog_t)(const TASKDIALOGCONFIG *, int *, int *, BOOL *);
    task_dialog_t task_dialog = (task_dialog_t)(void *)GetProcAddress(GetModuleHandleA("comctl32.dll"), "TaskDialogIndirect");
    wchar_t version[32], title[64], content[700];

    MultiByteToWideChar(CP_ACP, 0, GRBL2MACH_VERSION, -1, version, 32);
    _snwprintf(title, 63, L"GRBL2MACH %ls", version);
    title[63] = 0;
    _snwprintf(content, 699,
             L"Makes an RNR Eco Motion 2.0 USB controller usable from a grbl sender such as gSender. "
             L"The grblHAL core runs on this PC (based on the grblHAL Simulator).\n\n"
             L"Source code, documentation and issues:\n<a href=\"%ls\">%ls</a>\n\n"
             L"Free software under the GNU GPL v3 or later, without any warranty.\n\n"
             L"Theme: right-click the title bar.", REPO_URL, REPO_URL);
    content[699] = 0;

    if(!task_dialog) {
        MessageBoxW(hwnd, content, title, MB_ICONINFORMATION);
        return;
    }
    TASKDIALOGCONFIG tc = {
        .cbSize = sizeof(tc), .hwndParent = hwnd, .hInstance = GetModuleHandleA(NULL),
        .dwFlags = TDF_ENABLE_HYPERLINKS | TDF_ALLOW_DIALOG_CANCELLATION | TDF_POSITION_RELATIVE_TO_WINDOW | TDF_USE_HICON_MAIN,
        .dwCommonButtons = TDCBF_OK_BUTTON, .pszWindowTitle = L"About GRBL2MACH",
        .hMainIcon = (HICON)LoadImageA(GetModuleHandleA(NULL), MAKEINTRESOURCEA(1), IMAGE_ICON, S(32), S(32), 0),
        .pszMainInstruction = title, .pszContent = content, .pfCallback = about_callback
    };
    task_dialog(&tc, NULL, NULL, NULL);
}

/* --- Window ------------------------------------------------------------------------------------ */

static void update_tray (void)
{
    bool notify = false;

    EnterCriticalSection(&lock);
    if(problems_unseen > problems_notified) {
        notify = true;
        copy_str(tray.szInfo, sizeof(tray.szInfo), last_problem);
    }
    problems_notified = problems_unseen;
    LeaveCriticalSection(&lock);

    if(shown.led == tray_led && !notify && tray.cbSize)
        return;
    tray_led = shown.led;
    tray.hIcon = tray_icons[shown.led];
    snprintf(tray.szTip, sizeof(tray.szTip), "GRBL2MACH: %s", shown.title);
    tray.uFlags = NIF_ICON | NIF_TIP | NIF_MESSAGE;
    if(notify && GetForegroundWindow() != hwnd) {
        tray.uFlags |= NIF_INFO;
        copy_str(tray.szInfoTitle, sizeof(tray.szInfoTitle), "GRBL2MACH: problem reported");
        tray.dwInfoFlags = NIIF_WARNING;
    }
    Shell_NotifyIconA(NIM_MODIFY, &tray);
}

static void refresh (void)
{
    view_t v;
    char title[120];

    compute_view(&v);
    if(memcmp(&v, &shown, sizeof(v))) {
        shown = v;
        InvalidateRect(hwnd, NULL, FALSE);
        snprintf(title, sizeof(title), "GRBL2MACH %s - %s", GRBL2MACH_VERSION, v.title);
        SetWindowTextA(hwnd, title);
    }
    EnableWindow(btn_reconnect, v.running && v.can_reconnect);
    EnableWindow(btn_profile, phase != Gui_Starting && !closing);
    update_tray();
}

static LRESULT CALLBACK window_proc (HWND h, UINT msg, WPARAM wp, LPARAM lp)
{
    switch(msg) {

        case WM_CREATE: {
            hwnd = h;
            HINSTANCE inst = GetModuleHandleA(NULL);
            DWORD bs = WS_CHILD | WS_VISIBLE | WS_TABSTOP | BS_OWNERDRAW;
            btn_reconnect = CreateWindowA("BUTTON", "Reconnect controller", bs, 0, 0, 0, 0, h, (HMENU)ID_RECONNECT, inst, NULL);
            btn_profile = CreateWindowA("BUTTON", "Load profile...", bs, 0, 0, 0, 0, h, (HMENU)ID_PROFILE, inst, NULL);
            btn_edit = CreateWindowA("BUTTON", "Edit profile", bs, 0, 0, 0, 0, h, (HMENU)ID_EDIT, inst, NULL);
            btn_log = CreateWindowA("BUTTON", "Open log", bs, 0, 0, 0, 0, h, (HMENU)ID_LOG, inst, NULL);
            btn_about = CreateWindowA("BUTTON", "About", bs, 0, 0, 0, 0, h, (HMENU)ID_ABOUT, inst, NULL);
            btn_clear = CreateWindowA("BUTTON", "Clear", bs, 0, 0, 0, 0, h, (HMENU)ID_CLEAR, inst, NULL);
            chk_all = CreateWindowA("BUTTON", "All messages", bs, 0, 0, 0, 0, h, (HMENU)ID_ALL, inst, NULL);
            HMENU menu = GetSystemMenu(h, FALSE);
            AppendMenuA(menu, MF_SEPARATOR, 0, NULL);
            AppendMenuA(menu, MF_STRING, SYS_THEME_AUTO, "Theme: as Windows");
            AppendMenuA(menu, MF_STRING, SYS_THEME_LIGHT, "Theme: light");
            AppendMenuA(menu, MF_STRING, SYS_THEME_DARK, "Theme: dark");
            AppendMenuA(menu, MF_SEPARATOR, 0, NULL);
            AppendMenuA(menu, MF_STRING, SYS_ABOUT, "About GRBL2MACH...");
            list = CreateWindowExA(0, WC_LISTVIEWA, "", WS_CHILD | WS_VISIBLE | WS_TABSTOP | LVS_REPORT | LVS_NOCOLUMNHEADER | LVS_SINGLESEL,
                                   0, 0, 0, 0, h, (HMENU)ID_LIST, inst, NULL);
            ListView_SetExtendedListViewStyle(list, LVS_EX_FULLROWSELECT | LVS_EX_DOUBLEBUFFER | LVS_EX_INFOTIP);
            LVCOLUMNA col = { .mask = LVCF_WIDTH, .cx = S(400) };
            ListView_InsertColumn(list, 0, &col);
            theme_choice = read_theme_choice();
            apply_theme();
            HWND controls[] = { btn_reconnect, btn_profile, btn_edit, btn_log, btn_about, btn_clear, chk_all, list };
            for(size_t i = 0; i < sizeof(controls) / sizeof(controls[0]); i++)
                SendMessageA(controls[i], WM_SETFONT, (WPARAM)font_text, TRUE);
            char editor[MAX_PATH];
            editor_path(editor, sizeof(editor));
            EnableWindow(btn_edit, GetFileAttributesA(editor) != INVALID_FILE_ATTRIBUTES);

            for(int i = 0; i < Led_Count; i++)
                tray_icons[i] = make_tray_icon(i);
            tray.cbSize = sizeof(tray);
            tray.hWnd = h;
            tray.uID = 1;
            tray.uFlags = NIF_ICON | NIF_TIP | NIF_MESSAGE;
            tray.uCallbackMessage = WM_APP_TRAY;
            tray.hIcon = tray_icons[Led_Grey];
            copy_str(tray.szTip, sizeof(tray.szTip), "GRBL2MACH");
            Shell_NotifyIconA(NIM_ADD, &tray);

            compute_view(&shown);
            fill_list();
            SetTimer(h, 1, 250, NULL);
            return 0;
        }

        case WM_SIZE:
            layout(LOWORD(lp), HIWORD(lp));
            InvalidateRect(h, NULL, FALSE);
            return 0;

        case WM_GETMINMAXINFO: {
            MINMAXINFO *mm = (MINMAXINFO *)lp;
            mm->ptMinTrackSize.x = S(660);
            mm->ptMinTrackSize.y = S(440);
            return 0;
        }

        case WM_ERASEBKGND:
            return 1;

        case WM_PAINT: {
            PAINTSTRUCT ps;
            RECT r;
            HDC dc = BeginPaint(h, &ps);
            GetClientRect(h, &r);
            HDC mem = CreateCompatibleDC(dc);
            HBITMAP bmp = CreateCompatibleBitmap(dc, r.right, r.bottom);
            HGDIOBJ old = SelectObject(mem, bmp);
            paint(mem, r.right, r.bottom);
            BitBlt(dc, 0, 0, r.right, r.bottom, mem, 0, 0, SRCCOPY);
            SelectObject(mem, old);
            DeleteObject(bmp);
            DeleteDC(mem);
            EndPaint(h, &ps);
            return 0;
        }

        case WM_CTLCOLORSTATIC:
        case WM_CTLCOLORBTN:
            SetBkColor((HDC)wp, COL_BG);
            return (LRESULT)bg_brush;

        case WM_DRAWITEM:
            if(((DRAWITEMSTRUCT *)lp)->CtlType == ODT_BUTTON) {
                draw_button((DRAWITEMSTRUCT *)lp);
                return TRUE;
            }
            break;

        case WM_SYSCOMMAND:
            switch(wp & 0xFFF0) {
                case SYS_THEME_AUTO: set_theme_choice(Theme_Auto); return 0;
                case SYS_THEME_LIGHT: set_theme_choice(Theme_Light); return 0;
                case SYS_THEME_DARK: set_theme_choice(Theme_Dark); return 0;
                case SYS_ABOUT: on_about(); return 0;
            }
            break;

        case WM_SETTINGCHANGE:
            if(lp && !strcmp((const char *)lp, "ImmersiveColorSet") && theme_choice == Theme_Auto)
                apply_theme();
            break;

        case WM_TIMER:
            refresh();
            if(list_serial != entry_serial)
                fill_list();
            return 0;

        case WM_APP_LOG:
            refresh();
            return 0;

        case WM_NOTIFY: {
            NMHDR *nh = (NMHDR *)lp;
            if(nh->idFrom == ID_LIST && nh->code == NM_CUSTOMDRAW) {
                NMLVCUSTOMDRAW *cd = (NMLVCUSTOMDRAW *)lp;
                if(cd->nmcd.dwDrawStage == CDDS_PREPAINT)
                    return CDRF_NOTIFYITEMDRAW;
                if(cd->nmcd.dwDrawStage == CDDS_ITEMPREPAINT) {
                    LPARAM level = cd->nmcd.lItemlParam;
                    cd->clrText = level == Lvl_Problem ? COL_RED : level == Lvl_Warning ? COL_WARN :
                                  level == Lvl_Info ? COL_TEXT : COL_MUTED;
                    return CDRF_DODEFAULT;
                }
            }
            break;
        }

        case WM_COMMAND:
            switch(LOWORD(wp)) {
                case ID_RECONNECT: on_reconnect(); break;
                case ID_PROFILE: on_load_profile(); break;
                case ID_LOG: ShellExecuteA(h, "open", log_path, NULL, NULL, SW_SHOWNORMAL); break;
                case ID_EDIT: {
                    char editor[MAX_PATH];
                    editor_path(editor, sizeof(editor));
                    ShellExecuteA(h, "open", editor, NULL, NULL, SW_SHOWNORMAL);
                    break;
                }
                case ID_CLEAR:
                    EnterCriticalSection(&lock);
                    entry_count = 0;
                    entry_serial++;
                    problems_unseen = problems_notified = 0;
                    last_problem[0] = 0;
                    LeaveCriticalSection(&lock);
                    fill_list();
                    refresh();
                    break;
                case ID_ALL:
                    list_all = !list_all;
                    InvalidateRect(chk_all, NULL, FALSE);
                    fill_list();
                    break;
                case ID_ABOUT:
                    on_about();
                    break;
            }
            return 0;

        case WM_APP_TRAY:
            if(lp == WM_LBUTTONUP || lp == WM_LBUTTONDBLCLK || lp == NIN_BALLOONUSERCLICK) {
                ShowWindow(h, IsIconic(h) ? SW_RESTORE : SW_SHOW);
                SetForegroundWindow(h);
            }
            return 0;

        case WM_CLOSE:
            on_close();
            return 0;

        case WM_QUERYENDSESSION:
            return TRUE;

        case WM_ENDSESSION:
            if(wp) {
                Shell_NotifyIconA(NIM_DELETE, &tray);
                if(phase == Gui_Running && hooks.stop_machine)
                    hooks.stop_machine();
                exit(EXIT_SUCCESS);
            }
            return 0;

        case WM_DESTROY:
            Shell_NotifyIconA(NIM_DELETE, &tray);
            PostQuitMessage(0);
            return 0;
    }

    return DefWindowProcA(h, msg, wp, lp);
}

static HFONT make_font (int pt, int weight)
{
    return CreateFontA(-MulDiv(pt, dpi, 72), 0, 0, 0, weight, FALSE, FALSE, FALSE, DEFAULT_CHARSET,
                       OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS, CLEARTYPE_QUALITY, DEFAULT_PITCH, "Segoe UI");
}

static DWORD WINAPI window_main (LPVOID arg)
{
    (void)arg;
    SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_LOWEST);   // no higher than the sender's threads

    INITCOMMONCONTROLSEX icc = { sizeof(icc), ICC_LISTVIEW_CLASSES | ICC_STANDARD_CLASSES };
    InitCommonControlsEx(&icc);

    HDC dc = GetDC(NULL);
    dpi = GetDeviceCaps(dc, LOGPIXELSY);
    ReleaseDC(NULL, dc);
    font_title = make_font(15, FW_SEMIBOLD);
    font_text = make_font(9, FW_NORMAL);
    font_bold = make_font(10, FW_SEMIBOLD);

    WNDCLASSEXA wc = { .cbSize = sizeof(wc), .lpfnWndProc = window_proc, .hInstance = GetModuleHandleA(NULL),
                       .hIcon = LoadIconA(GetModuleHandleA(NULL), MAKEINTRESOURCEA(1)),
                       .hCursor = LoadCursorA(NULL, (LPCSTR)IDC_ARROW), .lpszClassName = WINDOW_CLASS };
    RegisterClassExA(&wc);
    HWND w = CreateWindowExA(0, WINDOW_CLASS, "GRBL2MACH " GRBL2MACH_VERSION, WS_OVERLAPPEDWINDOW | WS_CLIPCHILDREN,
                             CW_USEDEFAULT, CW_USEDEFAULT, S(720), S(560), NULL, NULL, wc.hInstance, NULL);
    SetEvent(window_ready);
    if(!w)
        return 1;
    ShowWindow(w, SW_SHOWNORMAL);
    dark_title_bar(w, pal->dark);
    UpdateWindow(w);

    MSG m;
    while(GetMessageA(&m, NULL, 0, 0) > 0) {
        if(!IsDialogMessageA(w, &m)) {
            TranslateMessage(&m);
            DispatchMessageA(&m);
        }
    }

    return 0;
}

/* --- Interface --------------------------------------------------------------------------------- */

bool gui_start (const gui_hooks_t *h)
{
    hooks = *h;
    InitializeCriticalSection(&lock);
    GetModuleFileNameA(NULL, exe_path, sizeof(exe_path));
    copy_str(exe_dir, sizeof(exe_dir), exe_path);
    char *slash = strrchr(exe_dir, '\\');
    if(slash)
        slash[1] = 0;

    // Restarted from the window: the previous bridge must have released the controller and the port
    if(restart_of) {
        HANDLE p = OpenProcess(SYNCHRONIZE, FALSE, restart_of);
        if(p) {
            WaitForSingleObject(p, 15000);
            CloseHandle(p);
        }
    }

    // One bridge window at a time (the controller and the port can only be opened once)
    CreateMutexA(NULL, FALSE, INSTANCE_MUTEX);
    if(GetLastError() == ERROR_ALREADY_EXISTS) {
        HWND other = FindWindowA(WINDOW_CLASS, NULL);
        if(other) {
            ShowWindow(other, SW_RESTORE);
            SetForegroundWindow(other);
        }
        return false;
    }

    open_log();
    capture_output();
    active = true;
    printf("GRBL2MACH %s, log file %s\n", GRBL2MACH_VERSION, log_path);

    window_ready = CreateEventA(NULL, TRUE, FALSE, NULL);
    window_thread = CreateThread(NULL, 0, window_main, NULL, 0, NULL);
    WaitForSingleObject(window_ready, 5000);

    return true;
}

bool gui_active (void)
{
    return active;
}

void gui_set_phase (gui_phase_t p)
{
    if(active) {
        InterlockedExchange(&phase, p);
        if(hwnd)
            PostMessageA(hwnd, WM_APP_LOG, 0, 0);
    }
}

void gui_set_profile (const char *path, bool given)
{
    if(!active)
        return;
    EnterCriticalSection(&lock);
    if(!GetFullPathNameA(path, sizeof(profile_path), profile_path, NULL))
        copy_str(profile_path, sizeof(profile_path), path);
    profile_given = given;
    LeaveCriticalSection(&lock);
}

void gui_set_listen (const char *addr, unsigned port)
{
    if(!active)
        return;
    EnterCriticalSection(&lock);
    snprintf(listen_str, sizeof(listen_str), "%s, port %u", addr, port);
    LeaveCriticalSection(&lock);
}

void gui_set_controller (const char *info)
{
    if(!active)
        return;
    EnterCriticalSection(&lock);
    copy_str(controller_str, sizeof(controller_str), info);
    LeaveCriticalSection(&lock);
}

int gui_finish (int rc)
{
    if(!active)
        return rc;
    exit_code = rc;
    printf("GRBL2MACH has stopped (exit code %d)\n", rc);
    gui_set_phase(Gui_Stopped);
    WaitForSingleObject(window_thread, INFINITE);

    return rc;
}
