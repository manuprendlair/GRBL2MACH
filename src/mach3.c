/*
  mach3.c - the machine's Mach3 profile (XML) as GRBL2MACH's configuration

  Reading: the profile is a flat XML file written by Mach3 (<Steps0>801.5</Steps0>...). Only the
  value of known tags is read; nothing else is interpreted.

  Start-up: the values the profile describes are applied to grblHAL's settings ($100...) once the
  core runs, and to the bridge's options (inputs, outputs, step/dir inversion, pulse rate) before
  the controller is opened. The profile is the reference: a setting it describes is overwritten
  at every start.

  Write-back: when a setting the profile describes changes (from gSender or any sender), the
  profile is updated 2 s after the last change, a batch of changes making one write. Only the
  values of the tags concerned are replaced: every other byte of the file stays as it was, so
  that Mach3 can still read it. A timestamped backup is made before each write (the 10 most
  recent are kept). If the file was changed on disk since it was read, it is not overwritten.

  Part of GRBL2MACH. GPL-3.0-or-later.
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <sys/stat.h>
#include <dirent.h>

#ifdef _WIN32
#include <windows.h>
#else
#include <unistd.h>
#endif

#include "mach3.h"

#include "grbl/hal.h"
#include "grbl/settings.h"

#define AXES          4         // X, Y, Z, A: the controller's axes
#define WRITE_DELAY_MS 2000     // write-back once the settings have not changed for that long
#define MAX_BACKUPS   10

static char path[1024];
static char *xml = NULL;        // file content, NUL-terminated
static size_t xml_len = 0;
static struct stat xml_stat;    // size and time of the file as last read or written
static bool loaded = false;
static double unit = 1.0;       // mm per profile unit for linear axes (Mach3 "Units": 0 mm, 1 inch)
static bool apply_pending = false, write_pending = false;
static uint32_t last_change_ms = 0;
static on_execute_realtime_ptr prev_execute_realtime = NULL;

// --- reading -------------------------------------------------------------------------------

// Span of the value of the first <key>...</key>, as offsets in xml
static bool tag_span (const char *key, size_t *start, size_t *end)
{
    char open[80], close[80];
    const char *p, *q;

    snprintf(open, sizeof(open), "<%s>", key);
    snprintf(close, sizeof(close), "</%s>", key);
    if(!xml || !(p = strstr(xml, open)))
        return false;
    p += strlen(open);
    if(!(q = strstr(p, close)))
        return false;
    *start = (size_t)(p - xml);
    *end = (size_t)(q - xml);

    return true;
}

static bool get_num (const char *key, double *value)
{
    size_t s, e;
    char buf[64], *end;

    if(!tag_span(key, &s, &e) || e - s == 0 || e - s >= sizeof(buf))
        return false;
    memcpy(buf, xml + s, e - s);
    buf[e - s] = 0;
    *value = strtod(buf, &end);

    return end != buf;
}

static double num (const char *fmt, int i, double dflt)
{
    char key[64];
    double v;

    snprintf(key, sizeof(key), fmt, i);

    return get_num(key, &v) ? v : dflt;
}

static int inum (const char *fmt, int i, int dflt)
{
    return (int)lround(num(fmt, i, dflt));
}

// Mach3 input signal index (Ports & Pins > Input Signals order)
enum {
    In_XPos = 0, In_XNeg = 1, In_XHome = 2,    // then Y, Z, A by steps of 3
    In_Probe = 22, In_EStop = 25
};

// An input as Mach3 declares it; port 3 pin n = controller input INn
static bool read_input (int index, rnr_input_t *in, char *err, size_t err_size)
{
    bool active = inum("Input%dActive", index, 0) != 0;
    int port = inum("Input%dPort", index, 0), pin = inum("Input%dPin", index, 0);

    in->active_low = inum("Input%dNeg", index, 0) != 0;     // Mach3 sends the polarity even when not used
    in->pin = 0;
    if(!active)
        return true;
    if(port != 3 || pin < 1 || pin > 15) {
        snprintf(err, err_size, "input signal %d is on port %d pin %d: only the controller's inputs (port 3, pins 1-15) are supported",
                  index, port, pin);
        return false;
    }
    in->pin = (uint8_t)pin;

    return true;
}

// Controller output OUTn of a Mach3 output signal index (Output Signals: port 3 pin n), 0 = none
static uint8_t output_number (int index)
{
    if(index <= 0 || !inum("Output%dActive", index, 0) || inum("Output%dPort", index, 0) != 3)
        return 0;

    int pin = inum("Output%dPin", index, 0);

    return pin >= 1 && pin <= 4 ? (uint8_t)pin : 0;
}

void mach3_default_path (char *out, size_t size, const char *argv0)
{
    char exe[1024] = "";

#ifdef _WIN32
    GetModuleFileNameA(NULL, exe, sizeof(exe));
#else
    ssize_t n = readlink("/proc/self/exe", exe, sizeof(exe) - 1);
    if(n > 0)
        exe[n] = 0;
    else
        snprintf(exe, sizeof(exe), "%s", argv0 ? argv0 : "");
#endif
    char *slash = strrchr(exe, '/');
#ifdef _WIN32
    char *bslash = strrchr(exe, '\\');
    if(bslash > slash)
        slash = bslash;
#endif
    if(slash)
        slash[1] = 0;
    else
        exe[0] = 0;
    snprintf(out, size, "%s%s", exe, MACH3_PROFILE_NAME);
}

bool mach3_loaded (void)
{
    return loaded;
}

bool mach3_load (const char *file, char *err, size_t err_size)
{
    FILE *f;
    long len;

    snprintf(path, sizeof(path), "%s", file);
    if(!(f = fopen(path, "rb"))) {
        snprintf(err, err_size, "no Mach3 profile at %s: copy your machine's Mach3 profile (.xml) there as %s, "
                  "or give its path with --mach3", path, MACH3_PROFILE_NAME);
        return false;
    }
    fseek(f, 0, SEEK_END);
    len = ftell(f);
    fseek(f, 0, SEEK_SET);
    free(xml);
    if(len <= 0 || !(xml = malloc((size_t)len + 1)) || fread(xml, 1, (size_t)len, f) != (size_t)len) {
        fclose(f);
        snprintf(err, err_size, "cannot read %s", path);
        return false;
    }
    fclose(f);
    xml[len] = 0;
    xml_len = (size_t)len;
    stat(path, &xml_stat);

    // Checks: what the bridge cannot do is refused rather than half done
    double v;
    if(!strstr(xml, "<profile>") || !get_num("Steps0", &v)) {
        snprintf(err, err_size, "%s does not look like a Mach3 profile (no <profile> or <Steps0>)", path);
        return false;
    }
    for(int a = 0; a < AXES; a++)
        if(inum("AxisToMotor%d", a, a) != a) {
            snprintf(err, err_size, "axis %c is mapped to motor %d (AxisToMotor%d): only the plain mapping X-0 Y-1 Z-2 A-3 is supported",
                      "XYZA"[a], inum("AxisToMotor%d", a, a), a);
            return false;
        }
    if(inum("MultiStep", 0, 0)) {
        snprintf(err, err_size, "the controller plugin's Multi-Step Speed (spindle speed on several outputs) is not supported");
        return false;
    }
    unit = inum("Units", 0, 0) ? 25.4 : 1.0;

    rnr_options_t check;
    char in_err[200] = "";
    memset(&check, 0, sizeof(check));
    for(int i = 0; i <= In_EStop; i++) {
        rnr_input_t in;
        if((i < 12 || i == In_Probe || i == In_EStop) && !read_input(i, &in, in_err, sizeof(in_err))) {
            snprintf(err, err_size, "%s", in_err);
            return false;
        }
    }

    loaded = true;
    fprintf(stderr, "[mach3] machine profile: %s\n", path);
    fprintf(stderr, "[mach3] for information (not applied): debounce %d x 40 us, look-ahead %d lines, G28 position %g %g %g\n",
             inum("Debounce", 0, 0), inum("LookAhead", 0, 0), num("G20X", 0, 0), num("G20Y", 0, 0), num("G20Z", 0, 0));

    return true;
}

// --- bridge options ------------------------------------------------------------------------

void mach3_bridge_options (rnr_options_t *opt)
{
    char err[200];

    if(!loaded)
        return;

    opt->step_dir_invert = 0;
    for(int a = 0; a < AXES; a++) {
        if(inum("Motor%dStepNeg", a, 0))
            opt->step_dir_invert |= (uint8_t)(1u << a);
        if(inum("Motor%dDirNeg", a, 0))
            opt->step_dir_invert |= (uint8_t)(0x10u << a);
        read_input(In_XPos + 3 * a, &opt->inputs.limit_pos[a], err, sizeof(err));
        read_input(In_XNeg + 3 * a, &opt->inputs.limit_neg[a], err, sizeof(err));
        read_input(In_XHome + 3 * a, &opt->inputs.home[a], err, sizeof(err));
    }
    read_input(In_Probe, &opt->inputs.probe, err, sizeof(err));
    read_input(In_EStop, &opt->inputs.estop, err, sizeof(err));
    opt->pins_raw = false;

    int cw = inum("SpinCW", 0, 0), ccw = inum("SpinCCW", 0, 0);
    opt->spindle_out = inum("NoSpindle", 0, 0) ? 0 : output_number(cw);
    opt->spindle_ccw_out = ccw != cw && !inum("NoSpindle", 0, 0) ? output_number(ccw) : 0;
    opt->flood_out = inum("NoFlood", 0, 0) ? 0 : output_number(inum("Flood", 0, 0));
    opt->mist_out = inum("NoFlood", 0, 0) ? 0 : output_number(inum("Mist", 0, 0));

    int khz = inum("PulseRates", 0, 100);
    opt->pulse_khz = (uint8_t)(khz >= 1 && khz <= 100 ? khz : 100);

    fprintf(stderr, "[mach3] controller: step/dir invert 0x%02X, pulse %u kHz, spindle OUT%u (ccw OUT%u), flood OUT%u, mist OUT%u (0 = none)\n",
             opt->step_dir_invert, opt->pulse_khz, opt->spindle_out, opt->spindle_ccw_out, opt->flood_out, opt->mist_out);
}

// --- grblHAL settings ----------------------------------------------------------------------

static bool get_setting (setting_id_t id, double *value)
{
    setting_details_t *set;
    const setting_detail_t *d = setting_get_details(id, &set);
    char *s;

    if(!d || !(s = setting_get_value(d, id - d->id)))
        return false;
    *value = strtod(s, NULL);

    return true;
}

static bool differs (double a, double b)
{
    return fabs(a - b) > 1e-9 * fmax(1.0, fmax(fabs(a), fabs(b)));
}

// True when the value equals the setting as grblHAL reports it: within half a unit of the last
// decimal it shows ("400.0" for 399.984) and within float precision (803.9 is stored 803.90002)
static bool matches_setting (setting_id_t id, double value)
{
    setting_details_t *set;
    const setting_detail_t *d = setting_get_details(id, &set);
    char *s;

    if(!d || !(s = setting_get_value(d, id - d->id)))
        return false;
    const char *dot = strchr(s, '.');
    double setting = strtod(s, NULL);
    double tolerance = 0.5 * pow(10.0, -(dot ? (int)strlen(dot + 1) : 0)) + 1e-6 * fabs(value);

    return fabs(setting - value) <= tolerance;
}

// Stores a setting if it differs; logs it
static void put_setting (setting_id_t id, double value, char *log, size_t log_size)
{
    char s[40];
    status_code_t status;

    if(matches_setting(id, value))
        return;
    snprintf(s, sizeof(s), "%.9g", value);
    status = settings_store_setting(id, s);
    size_t n = strlen(log);
    if(status == Status_OK)
        snprintf(log + n, log_size - n, " $%d=%s", (int)id, s);
    else
        snprintf(log + n, log_size - n, " $%d=%s(refused: error %d)", (int)id, s, (int)status);
}

// grblHAL setting values the profile describes
typedef struct {
    double steps[AXES], rate[AXES], accel[AXES], travel[AXES], seek[AXES];
    int step_invert, dir_invert, home_dir, soft_limits;
    double pulloff, rpm_max, rpm_min;
} profile_settings_t;

static double axis_unit (int a)
{
    return a < 3 ? unit : 1.0;      // A is angular: degrees whatever the units
}

static void profile_settings (profile_settings_t *p)
{
    int pulley = inum("PULLEY", 0, 0) + 1;
    double ratio = num("SpindleRatio", 0, 1.0);

    memset(p, 0, sizeof(*p));
    for(int a = 0; a < AXES; a++) {
        double u = axis_unit(a);
        p->steps[a] = num("Steps%d", a, 1.0) / u;
        p->rate[a] = num("Vel%d", a, 1.0) * 60.0 * u;
        p->accel[a] = num("Acc%d", a, 1.0) * u;
        p->travel[a] = (num("M%dMax", a, 0.0) - num("M%dMin", a, 0.0)) * u;
        p->seek[a] = p->rate[a] * inum("RefSpeed%d", a, 20) / 100.0;
        p->step_invert |= inum("Motor%dStepNeg", a, 0) << a;
        p->dir_invert |= inum("Motor%dDirNeg", a, 0) << a;
        p->home_dir |= inum("M%dNeg", a, 0) << a;
    }
    p->soft_limits = inum("SoftLimit", 0, 0) != 0;
    p->pulloff = fmax(num("XPullOff", 0, 0.5), fmax(num("YPullOff", 0, 0.5), num("ZPullOff", 0, 0.5))) * unit;
    p->rpm_max = num("SPEED%d", pulley, 1000.0) / (ratio > 0.0 ? ratio : 1.0);
    p->rpm_min = num("LSPEED%d", pulley, 0.0) / (ratio > 0.0 ? ratio : 1.0);
}

static void apply_settings (void)
{
    profile_settings_t p;
    char log[1200] = "";
    double v;
    int homing = 0;

    profile_settings(&p);
    if(get_setting(Setting_HomingEnable, &v))
        homing = (int)v;
    bool first_homing = !(homing & 1), first_rates = !(homing & 512);

    for(int a = 0; a < AXES && a < N_AXIS; a++) {
        put_setting((setting_id_t)(Setting_AxisStepsPerMM + a), p.steps[a], log, sizeof(log));
        put_setting((setting_id_t)(Setting_AxisMaxRate + a), p.rate[a], log, sizeof(log));
        put_setting((setting_id_t)(Setting_AxisAcceleration + a), p.accel[a], log, sizeof(log));
        put_setting((setting_id_t)(Setting_AxisMaxTravel + a), p.travel[a], log, sizeof(log));
    }
    put_setting(Setting_StepInvertMask, p.step_invert, log, sizeof(log));
    put_setting(Setting_DirInvertMask, p.dir_invert, log, sizeof(log));
    put_setting(Setting_HomingDirMask, p.home_dir, log, sizeof(log));
    // homing on, single axis commands ($HX), per axis rates; the sender may set the other bits
    put_setting(Setting_HomingEnable, homing | 1 | 2 | 512, log, sizeof(log));
    for(int a = 0; a < AXES && a < N_AXIS; a++) {
        put_setting((setting_id_t)(Setting_AxisHomingSeekRate + a), p.seek[a], log, sizeof(log));
        if(first_rates)     // not in the profile: Mach3's slow approach, then the sender's choice
            put_setting((setting_id_t)(Setting_AxisHomingFeedRate + a), 45.0, log, sizeof(log));
    }
    if(first_homing) {      // not in the profile: Mach3's Ref All order, one axis per cycle
        put_setting(Setting_HomingCycle_1, 4, log, sizeof(log));
        put_setting(Setting_HomingCycle_2, 1, log, sizeof(log));
        put_setting(Setting_HomingCycle_3, 2, log, sizeof(log));
    }
    put_setting(Setting_HomingPulloff, p.pulloff, log, sizeof(log));
    put_setting(Setting_SoftLimitsEnable, p.soft_limits, log, sizeof(log));
    put_setting(Setting_RpmMax, p.rpm_max, log, sizeof(log));
    put_setting(Setting_RpmMin, p.rpm_min, log, sizeof(log));

    fprintf(stderr, "[mach3] settings from the profile:%s\n", log[0] ? log : " all already set");
    if(N_AXIS < AXES)
        fprintf(stderr, "[mach3] this build has %d axes: the profile's A axis is ignored\n", N_AXIS);
    for(int a = 0; a < 3; a++)
        if(differs(num("M%dRefHome", a, 0.0), 0.0) || differs(num("M%dMax", a, 0.0), 0.0))
            fprintf(stderr, "[mach3] axis %c: home offset or soft max not 0 in the profile: grblHAL sets the machine "
                             "position to 0 at the home switch\n", "XYZ"[a]);
}

// --- write-back ----------------------------------------------------------------------------

typedef struct {
    char tag[32];
    char value[48];
} patch_t;

static int add_patch (patch_t *patches, int n, const char *tag, const char *value)
{
    size_t s, e;

    if(n >= 64 || !tag_span(tag, &s, &e))
        return n;
    if(e - s == strlen(value) && !memcmp(xml + s, value, e - s))
        return n;   // unchanged
    snprintf(patches[n].tag, sizeof(patches[n].tag), "%s", tag);
    snprintf(patches[n].value, sizeof(patches[n].value), "%s", value);

    return n + 1;
}

// A number as Mach3 writes it: "801.5", "45." for a whole value
static void mach3_number (char *out, size_t size, double v)
{
    snprintf(out, size, "%.10g", v);
    if(!strpbrk(out, ".e"))
        strncat(out, ".", size - strlen(out) - 1);
}

// Tag value to write when the setting differs from what the profile gives
static int patch_num (patch_t *p, int n, const char *fmt, int i, setting_id_t id, double from_profile, double to_profile)
{
    char tag[32], value[48];

    if(matches_setting(id, from_profile))
        return n;
    snprintf(tag, sizeof(tag), fmt, i);
    mach3_number(value, sizeof(value), to_profile);

    return add_patch(p, n, tag, value);
}

static int patch_int (patch_t *p, int n, const char *fmt, int i, int value)
{
    char tag[32], s[16];
    double cur;

    snprintf(tag, sizeof(tag), fmt, i);
    if(get_num(tag, &cur) && (int)lround(cur) == value)
        return n;
    snprintf(s, sizeof(s), "%d", value);

    return add_patch(p, n, tag, s);
}

static int collect_patches (patch_t *p)
{
    profile_settings_t prof;
    double v;
    int n = 0;
    int pulley = inum("PULLEY", 0, 0) + 1;
    double ratio = num("SpindleRatio", 0, 1.0);

    profile_settings(&prof);
    for(int a = 0; a < AXES && a < N_AXIS; a++) {
        double u = axis_unit(a);
        if(get_setting((setting_id_t)(Setting_AxisStepsPerMM + a), &v))
            n = patch_num(p, n, "Steps%d", a, (setting_id_t)(Setting_AxisStepsPerMM + a), prof.steps[a], v * u);
        if(get_setting((setting_id_t)(Setting_AxisMaxRate + a), &v)) {
            n = patch_num(p, n, "Vel%d", a, (setting_id_t)(Setting_AxisMaxRate + a), prof.rate[a], v / 60.0 / u);
            prof.rate[a] = v;   // the homing speed % below is relative to the new rate
        }
        if(get_setting((setting_id_t)(Setting_AxisAcceleration + a), &v))
            n = patch_num(p, n, "Acc%d", a, (setting_id_t)(Setting_AxisAcceleration + a), prof.accel[a], v / u);
        if(get_setting((setting_id_t)(Setting_AxisMaxTravel + a), &v))
            n = patch_num(p, n, "M%dMin", a, (setting_id_t)(Setting_AxisMaxTravel + a), prof.travel[a], num("M%dMax", a, 0.0) - v / u);
        if(get_setting((setting_id_t)(Setting_AxisHomingSeekRate + a), &v) && prof.rate[a] > 0.0 &&
            !matches_setting((setting_id_t)(Setting_AxisHomingSeekRate + a), prof.rate[a] * inum("RefSpeed%d", a, 20) / 100.0))
            n = patch_int(p, n, "RefSpeed%d", a, (int)lround(v * 100.0 / prof.rate[a]));
    }
    if(get_setting(Setting_StepInvertMask, &v))
        for(int a = 0; a < AXES && a < N_AXIS; a++)
            n = patch_int(p, n, "Motor%dStepNeg", a, ((int)v >> a) & 1);
    if(get_setting(Setting_DirInvertMask, &v))
        for(int a = 0; a < AXES && a < N_AXIS; a++)
            n = patch_int(p, n, "Motor%dDirNeg", a, ((int)v >> a) & 1);
    if(get_setting(Setting_HomingDirMask, &v))
        for(int a = 0; a < AXES && a < N_AXIS; a++)
            n = patch_int(p, n, "M%dNeg", a, ((int)v >> a) & 1);
    if(get_setting(Setting_SoftLimitsEnable, &v))
        n = patch_int(p, n, "SoftLimit%.0d", 0, (int)v);
    if(get_setting(Setting_HomingPulloff, &v) && !matches_setting(Setting_HomingPulloff, prof.pulloff)) {
        char s[16];
        snprintf(s, sizeof(s), "%.3f", v / unit);   // as the plugin writes it: "0.500"
        n = add_patch(p, n, "XPullOff", s);
        n = add_patch(p, n, "YPullOff", s);
        n = add_patch(p, n, "ZPullOff", s);
    }
    if(get_setting(Setting_RpmMax, &v) && !matches_setting(Setting_RpmMax, prof.rpm_max))
        n = patch_int(p, n, "SPEED%d", pulley, (int)lround(v * ratio));
    if(get_setting(Setting_RpmMin, &v) && !matches_setting(Setting_RpmMin, prof.rpm_min))
        n = patch_int(p, n, "LSPEED%d", pulley, (int)lround(v * ratio));

    return n;
}

static void remove_old_backups (void)
{
    char dir[1024], base[256], names[64][300];
    int count = 0;
    const char *slash = strrchr(path, '/'), *bslash = strrchr(path, '\\');

    if(bslash > slash)
        slash = bslash;
    snprintf(dir, sizeof(dir), "%.*s", slash ? (int)(slash - path) : 1, slash ? path : ".");
    snprintf(base, sizeof(base), "%s.", slash ? slash + 1 : path);

    DIR *d = opendir(dir);
    struct dirent *e;
    if(!d)
        return;
    while((e = readdir(d)) && count < 64) {
        size_t len = strlen(e->d_name);
        if(!strncmp(e->d_name, base, strlen(base)) && len > 4 && !strcmp(e->d_name + len - 4, ".bak"))
            snprintf(names[count++], sizeof(names[0]), "%s", e->d_name);
    }
    closedir(d);
    qsort(names, (size_t)count, sizeof(names[0]), (int (*)(const void *, const void *))strcmp);
    for(int i = 0; i < count - MAX_BACKUPS; i++) {     // timestamped names: oldest first
        char old[1400];
        snprintf(old, sizeof(old), "%s/%s", dir, names[i]);
        remove(old);
    }
}

static bool write_file (const char *file, const char *data, size_t len)
{
    FILE *f = fopen(file, "wb");
    bool ok = f && fwrite(data, 1, len, f) == len;

    if(f && fclose(f) != 0)
        ok = false;

    return ok;
}

static void write_back (void)
{
    patch_t patches[64];
    int n = collect_patches(patches);
    struct stat now;

    if(n == 0)
        return;

    if(stat(path, &now) != 0 || now.st_size != xml_stat.st_size || now.st_mtime != xml_stat.st_mtime) {
        fprintf(stderr, "[mach3] %s was changed on disk since it was read: settings NOT written to it "
                         "(restart the bridge to use the file as it is now)\n", path);
        return;
    }

    // Backup of the file as it is on disk (= xml, checked just above)
    char backup[1100], stamp[32];
    time_t t = time(NULL);
    strftime(stamp, sizeof(stamp), "%Y-%m-%d_%H-%M-%S", localtime(&t));
    snprintf(backup, sizeof(backup), "%s.%s.bak", path, stamp);
    if(!write_file(backup, xml, xml_len)) {
        fprintf(stderr, "[mach3] cannot write the backup %s: profile not updated\n", backup);
        return;
    }

    // New content: the old one with only the values of the patched tags replaced
    size_t cap = xml_len + 64 * 48 + 1, len = xml_len;
    char *out = malloc(cap), log[1200] = "";
    if(!out)
        return;
    memcpy(out, xml, xml_len + 1);
    for(int i = 0; i < n; i++) {
        size_t s, e;
        char *saved = xml;
        xml = out;                      // find the tag in the content being built
        bool found = tag_span(patches[i].tag, &s, &e);
        xml = saved;
        if(!found)
            continue;
        size_t vlen = strlen(patches[i].value);
        memmove(out + s + vlen, out + e, len - e + 1);
        memcpy(out + s, patches[i].value, vlen);
        len = len - (e - s) + vlen;
        size_t l = strlen(log);
        snprintf(log + l, sizeof(log) - l, " %s=%s", patches[i].tag, patches[i].value);
    }

    char tmp[1100];
    snprintf(tmp, sizeof(tmp), "%s.tmp", path);
    if(!write_file(tmp, out, len) || (remove(path), rename(tmp, path) != 0)) {
        fprintf(stderr, "[mach3] cannot write %s (backup kept in %s)\n", path, backup);
        free(out);
        return;
    }
    free(xml);
    xml = out;
    xml_len = len;
    stat(path, &xml_stat);
    remove_old_backups();
    fprintf(stderr, "[mach3] profile updated (backup %s):%s\n", backup, log);
}

// --- core hooks ----------------------------------------------------------------------------

static void execute_realtime (sys_state_t state)
{
    if(apply_pending) {
        apply_pending = false;
        apply_settings();
        write_pending = false;          // the changes above come from the profile itself
    } else if(write_pending && hal.get_elapsed_ticks() - last_change_ms >= WRITE_DELAY_MS) {
        write_pending = false;
        write_back();
    }

    if(prev_execute_realtime)
        prev_execute_realtime(state);
}

void mach3_attach_core (void)
{
    if(!loaded)
        return;
    if(grbl.on_execute_realtime != execute_realtime) {
        prev_execute_realtime = grbl.on_execute_realtime;
        grbl.on_execute_realtime = execute_realtime;
        apply_pending = true;
    }
}

void mach3_settings_changed (void)
{
    if(!loaded || apply_pending)
        return;
    write_pending = true;
    last_change_ms = hal.get_elapsed_ticks();
}
