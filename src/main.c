/*
  main.c - main grbl simulator program

  Part of Grbl Simulator

  Copyright (c) 2012 Jens Geisler
  Copyright (c) 2014-2015 Adam Shelly

  2020 - modified for grblHAL by Terje Io

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#ifdef WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#else
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#endif

#include "simulator.h"
#include "eeprom.h"
#include "grbl_interface.h"
#include "build_info.h"
#include "fs_posix.h"
#include "slicer.h"
#include "mach3.h"
#include "rnr/sink_rnr.h"
#include "irq_lock.h"

#include "grbl/grbllib.h"

extern void fs_stream_init (void);

arg_vars_t args;
const char* progname;

#ifdef WIN32
static SOCKET socket_fd;
#else
static int socket_fd = 0;
#endif
static fd_set rfds;
static char *fsdir;

// GRBL2MACH options
static const char *listen_addr = "127.0.0.1";   // TCP listens on this machine only unless -l is given
static const char *slice_file = NULL;           // -o
static const char *fake_log = NULL;             // -f
static bool use_usb = false;                    // -u
static bool speedup_given = false;
static bool irq_lock_on = true;                 // --no-irq-lock
// --rnr-invert, --rnr-out, --rnr-in, --rnr-estop-in, --rnr-pins. Default: spindle on OUT1, e-stop
// on IN1 active high (Mach3's EStop on port 3 pin 1), declared to the controller so that it stops
// by itself.
static rnr_options_t rnr_opt = { .spindle_out = 1, .inputs.estop = { .pin = 1 } };
// Options given on the command line win over the Mach3 profile (tests, trials)
static const char *mach3_path = NULL;           // --mach3
static bool cli_invert = false, cli_out = false, cli_in = false, cli_estop = false;

// "<n>" or "<n>low": input number (0 = not used, 1..15) and polarity
static bool parse_rnr_input (const char *s, rnr_input_t *in)
{
    char *end;
    unsigned long n = strtoul(s, &end, 10);

    if(end == s || n > 15 || (*end && strcmp(end, "low")))
        return false;
    in->pin = (uint8_t)n;
    in->active_low = *end != 0;

    return true;
}

// --rnr-in xhome=3low,yhome=3low,zhome=3low,probe=2,x+=4low,x-=5low,estop=1
static bool parse_rnr_inputs (const char *arg)
{
    static const char axes[] = "xyza";
    char buf[256], *save = NULL;

    snprintf(buf, sizeof(buf), "%s", arg);
    for(char *item = strtok_r(buf, ",", &save); item; item = strtok_r(NULL, ",", &save)) {
        char *eq = strchr(item, '=');
        rnr_input_t *in = NULL;
        if(!eq)
            return false;
        *eq = 0;
        if(!strcmp(item, "probe"))
            in = &rnr_opt.inputs.probe;
        else if(!strcmp(item, "estop"))
            in = &rnr_opt.inputs.estop;
        else if(item[0] && strchr(axes, item[0])) {
            int a = (int)(strchr(axes, item[0]) - axes);
            if(!strcmp(item + 1, "home")) in = &rnr_opt.inputs.home[a];
            else if(!strcmp(item + 1, "+")) in = &rnr_opt.inputs.limit_pos[a];
            else if(!strcmp(item + 1, "-")) in = &rnr_opt.inputs.limit_neg[a];
        }
        if(!in || !parse_rnr_input(eq + 1, in))
            return false;
    }

    return true;
}

// --rnr-out spindle=1,ccw=0,flood=2,mist=0
static bool parse_rnr_outputs (const char *arg)
{
    char buf[128], *save = NULL;

    snprintf(buf, sizeof(buf), "%s", arg);
    for(char *item = strtok_r(buf, ",", &save); item; item = strtok_r(NULL, ",", &save)) {
        char *eq = strchr(item, '=');
        if(!eq || eq[1] < '0' || eq[1] > '4' || eq[2])
            return false;
        *eq = 0;
        uint8_t n = (uint8_t)(eq[1] - '0');
        if(!strcmp(item, "spindle")) rnr_opt.spindle_out = n;
        else if(!strcmp(item, "ccw")) rnr_opt.spindle_ccw_out = n;
        else if(!strcmp(item, "flood")) rnr_opt.flood_out = n;
        else if(!strcmp(item, "mist")) rnr_opt.mist_out = n;
        else return false;
    }

    return true;
}

void print_usage(const char* badarg)
{
    if (badarg)
        printf("Unrecognized option %s\n",badarg);

    printf("Usage: \n"
      "%s [options] [time_step] [block_file]\n"
      "  Options:\n"
      "    -r <report time>   : minimum time step for printing stepper values. Default=0=no print.\n"
      "    -t <time factor>   : multiplier to realtime clock. Default=1.0; 0=\"as fast as possible\"\n"
      "    -g <response file> : file to report responses from grbl.  default = stdout\n"
      "    -b <block file>    : file to report each block executed.  default = stdout\n"
      "    -s <step file>     : file to report each step executed.  default = stderr\n"
      "    -e <EEPROM file>   : file containing grblHAL settings.  default = EEPROM.DAT\n"
      "    -p <port>          : port to open raw telnet communication.\n"
      "    -l <address>       : GRBL2MACH: address to listen on with -p. Default=127.0.0.1 (this machine only).\n"
      "    -o <slice file>    : GRBL2MACH: write 5 ms step slices (CSV) to file.\n"
      "    -u                 : GRBL2MACH: drive the RNR Eco Motion USB controller.\n"
      "    -f <log file>      : GRBL2MACH: drive a fake RNR controller (tests), logging every frame.\n"
      "    --no-irq-lock      : GRBL2MACH: let simulated interrupts run concurrently with the core (upstream behaviour).\n"
      "    --mach3 <file>     : GRBL2MACH: the machine's Mach3 profile. Default: MACH3.xml next to this program,\n"
      "                         required with -u. The --rnr-* options below override what it describes.\n"
      "    --rnr-invert <hex> : GRBL2MACH: controller step/dir inversion byte (bits 0-3: step active low X..A,\n"
      "                         bits 4-7: direction inverted X..A; Mach3 example 0x8F). Default 0.\n"
      "    --rnr-out <function>=<n>[,...] : GRBL2MACH: controller output OUT<n> (1-4, 0 = none) driven by\n"
      "                         spindle (M3/M4), ccw (M4, when apart from spindle), flood (M8), mist (M7).\n"
      "                         Default spindle=1 (Mach3 Spindle CW on OUT1), coolant on no output.\n"
      "    --rnr-in <input>=<n>[low][,...] : GRBL2MACH: controller inputs IN<n> (0 = not used, \"low\": active\n"
      "                         low), as in Mach3's Input Signals page (port 3 pin n): x+ x- xhome, y+ y- yhome,\n"
      "                         z+ z- zhome, a+ a- ahome, probe, estop. Example: xhome=3low,yhome=3low,zhome=3low,probe=2\n"
      "    --rnr-estop-in <n>[low] : GRBL2MACH: same as --rnr-in estop=<n>[low]. The e-stop input is declared to\n"
      "                         the controller (it stops by itself) and watched by the bridge. Default 1 (active\n"
      "                         high, as Mach3's EStop on port 3 pin 1); 0 = none.\n"
      "    --rnr-pins <20 hex digits> : GRBL2MACH: configuration frame bytes 5-14 copied as is from a Mach3\n"
      "                         capture (replaces the bytes computed from --rnr-in), e.g. 00730073007300701200.\n"
	  "    -d <directory>     : directory to mount as local filing system.\n"
      "    -c<comment_char>   : character to print before each line from grbl.  default = '#'\n"
      "    -n                 : no comments before grbl response lines.\n"
      "    -v                 : print version and build information.\n"
      "    -h                 : this help.\n"
      "\n"
      "  <time_step> and <block_file> can be specifed with option flags or positional parameters\n"
      "\n"
      "  ^-F to shutdown cleanly\n"
      "\n",
      progname);
}

void print_version(void)
{
    printf("GRBL2MACH: grblHAL core driving an RNR Eco Motion USB controller (based on the grblHAL simulator)\n"
      "  Build type : %s\n"
      "  Compiler   : %s\n"
      "  Target     : %s\n"
      "  C flags    : %s\n"
      "  Built      : %s %s\n",
      SIM_BUILD_TYPE,
      SIM_COMPILER,
      SIM_TARGET,
      SIM_C_FLAGS[0] ? SIM_C_FLAGS : "(none)",
      __DATE__, __TIME__);
}

//wrapper for thread interface
PLAT_THREAD_FUNC(grbl_main_thread, exit)
{
    irq_lock_core_start();  // GRBL2MACH: the core thread holds the interrupt lock while it runs
    grbl_enter();

    return 0; //NULL;
}

#ifdef WIN32

//return char if one available.
uint8_t sim_socket_in()
{
    char c = 0;
    int retval;
    DWORD t = 0;

    if(sim.socket_fd == INVALID_SOCKET) {
        sim.socket_fd = accept(socket_fd, NULL, NULL);
        setsockopt(sim.socket_fd, SOL_SOCKET, SO_RCVTIMEO, (char *)&t, sizeof(t));
        // Disable Nagle: senders (gSender & co) poll with single-byte '?' and
        // responses are small; Nagle + delayed ACK adds up to ~200 ms per line.
        BOOL nodelay = TRUE;
        setsockopt(sim.socket_fd, IPPROTO_TCP, TCP_NODELAY, (char *)&nodelay, sizeof(nodelay));
        u_long mode = 1;
        ioctlsocket(sim.socket_fd, FIONBIO, &mode); // set non-blocking
    } else if((retval = recv(sim.socket_fd, &c, 1, 0)) < 1) {
        if(retval == 0)
            sim.socket_fd = INVALID_SOCKET;
        else if((retval = WSAGetLastError()) != WSAEWOULDBLOCK) {
            printf("Fatal: socket error %d\n", retval);
            exit(-5);
        }
    }

    return c;
}

#else

//return char if one available.
uint8_t sim_socket_in()
{
    char c = 0;
    int retval;

    struct timeval tv = {
        .tv_sec = 0,
        .tv_usec = 0
    };

    // Use select() ONLY to detect a new incoming connection on the listening
    // socket. Data on the accepted connection is read with a single
    // non-blocking read() below (the accepted socket is put in O_NONBLOCK once
    // right after accept), instead of select()ing on it every poll.
    fd_set cfds;
    FD_ZERO(&cfds);
    FD_SET(socket_fd, &cfds);

    int accepted = 0;

    if((retval = select(socket_fd + 1, &cfds, NULL, NULL, &tv)) > 0 && FD_ISSET(socket_fd, &cfds)) {

        socklen_t addrlen = sizeof(struct sockaddr_in);
        struct sockaddr_in client_addr;

        sim.socket_fd = accept(socket_fd, (struct sockaddr *) &client_addr, &addrlen);
        if (sim.socket_fd < 0) {
            printf("Fatal: Error on socket accept.\n");
            exit(-5);
        }

        FD_SET(sim.socket_fd, &rfds);

        // Put the accepted socket into non-blocking mode once so subsequent
        // reads return immediately when there is no data.
        int flags = fcntl(sim.socket_fd, F_GETFL, 0);
        if (flags != -1)
            fcntl(sim.socket_fd, F_SETFL, flags | O_NONBLOCK);

        // Disable Nagle: status polls and responses are tiny; Nagle + delayed
        // ACK can hold the tail of a response line back by up to ~200 ms.
        int nodelay = 1;
        setsockopt(sim.socket_fd, IPPROTO_TCP, TCP_NODELAY, &nodelay, sizeof(nodelay));

        accepted = 1;
    }

    // Read incoming data from the accepted connection. Skip on the poll where
    // we just accepted (a brand-new connection has no data yet), matching the
    // old select()-gated behavior.
    if(!accepted && sim.socket_fd > 0) {

        retval = read(sim.socket_fd, &c, 1);

        if(retval == 0) {
            close(sim.socket_fd);
            FD_CLR(sim.socket_fd, &rfds);
            sim.socket_fd = 0;
            c = 0;
        } else if(retval < 0) {
            c = 0;   // EAGAIN/EWOULDBLOCK: no data available
        }
//            if(c && c != '?' && c >= ' ')
//                sim_serial_out(c == '\r' ? '\n' : c);
    }

    return c;
}

#endif

static void exithandler (int signum)
{
    eeprom_close();
}

void board_init (void)
{
	if(fsdir) {
		fs_stream_init();
        if(!fs_posix_mount("/", fsdir))
			printf("No such directory: %s\n", fsdir);
	}
}

int main(int argc, char *argv[])
{
    int positional_args = 0;

    //defaults
    args.step_out_file = stderr;
    args.block_out_file = stdout;
    args.serial_out_file = stdout;
    args.comment_char = '#';
    args.speedup = 1.0f;

    args.step_time = 0.0f;
    // Get the minimum time step for printing stepper values.
    // If not given or the command line cannot be parsed to a float than
    // step_time= 0.0; This means to not print stepper values at all

    set_eeprom_name("EEPROM.DAT");

    progname = argv[0];

    while (argc > 1) {
        argv++; argc--;
        if (argv[0][0] == '-') {

            if (!strcmp(argv[0], "--version")) {
                print_version();
                return EXIT_SUCCESS;
            }

            if (!strcmp(argv[0], "--no-irq-lock")) {
                irq_lock_on = false;
                continue;
            }

            if (!strcmp(argv[0], "--mach3") && argc > 1) {
                argv++; argc--;
                mach3_path = *argv;
                continue;
            }

            if (!strcmp(argv[0], "--rnr-pins") && argc > 1) {
                argv++; argc--;
                const char *h = *argv;
                bool ok = strlen(h) == 2 * sizeof(rnr_opt.pins);
                for (size_t i = 0; ok && i < sizeof(rnr_opt.pins); i++) {
                    char byte[3] = { h[2 * i], h[2 * i + 1], 0 }, *end;
                    rnr_opt.pins[i] = (uint8_t)strtoul(byte, &end, 16);
                    ok = *end == 0;
                }
                if (!ok) {
                    print_usage(*argv);
                    exit(-1);
                }
                rnr_opt.pins_raw = true;
                continue;
            }

            if (!strcmp(argv[0], "--rnr-in") && argc > 1) {
                argv++; argc--;
                if (!parse_rnr_inputs(*argv)) {
                    print_usage(*argv);
                    exit(-1);
                }
                cli_in = true;
                continue;
            }

            if (!strcmp(argv[0], "--rnr-estop-in") && argc > 1) {
                argv++; argc--;
                if (!parse_rnr_input(*argv, &rnr_opt.inputs.estop)) {
                    print_usage(*argv);
                    exit(-1);
                }
                cli_estop = true;
                continue;
            }

            if (!strcmp(argv[0], "--rnr-out") && argc > 1) {
                argv++; argc--;
                if (!parse_rnr_outputs(*argv)) {
                    print_usage(*argv);
                    exit(-1);
                }
                cli_out = true;
                continue;
            }

            if (!strcmp(argv[0], "--rnr-invert") && argc > 1) {
                argv++; argc--;
                rnr_opt.step_dir_invert = (uint8_t)strtoul(*argv, NULL, 16);
                cli_invert = true;
                continue;
            }

            switch(argv[0][1]){

                case 'c':  //set Comment char 
                    args.comment_char = argv[0][2];
                    break;

                case 'n': //No comment char on grbl responses
                    args.comment_char = 0;
                    break;

                case 't': //Tick rate
                    argv++; argc--;
                    args.speedup = atof(*argv);
                    speedup_given = true;
                    break;

                case 'b': //Block file
                    argv++; argc--;
                    args.block_out_file = fopen(*argv,"w");
                    if (!args.block_out_file) {
                        perror("fopen");
                        printf("Error opening : %s\n",*argv);
                        return EXIT_FAILURE;
                    }
                    break;

                case 'e': //EEPROM file
                    argv++; argc--;
                    set_eeprom_name(*argv);
                    break;

                case 's': //Step out file.
                    argv++; argc--;
                    args.step_out_file = fopen(*argv,"w");
                    if (!args.step_out_file) {
                        perror("fopen");
                        printf("Error opening : %s\n",*argv);
                        return EXIT_FAILURE;
                    }
                    break;

                case 'g': //Grbl output
                    argv++; argc--;
                    args.serial_out_file = fopen(*argv,"w");
                    if (!args.serial_out_file) {
                        perror("fopen");
                        printf("Error opening : %s\n",*argv);
                        return EXIT_FAILURE;
                    }
                    break;

                case 'r':  //step_time for Reporting
                    argv++; argc--;
                    args.step_time = atof(*argv);
                    break;

                case 'p':  // Raw telnet port
                    argv++; argc--;
                    args.port = atoi(*argv);
                    break;

                case 'o':  // GRBL2MACH: slice file
                    argv++; argc--;
                    slice_file = *argv;
                    break;

                case 'f':  // GRBL2MACH: fake RNR controller
                    argv++; argc--;
                    fake_log = *argv;
                    break;

                case 'u':  // GRBL2MACH: RNR USB controller
                    use_usb = true;
                    break;

                case 'l':  // GRBL2MACH: listen address
                    argv++; argc--;
                    listen_addr = *argv;
                    break;

                case 'v':
                    print_version();
                    return EXIT_SUCCESS;

                case 'h':
                    print_usage(NULL);
                    return EXIT_SUCCESS;

                case 'd':
                    argv++; argc--;
					if((fsdir = malloc(strlen(*argv) + 1)))
						strcpy(fsdir, *argv);
					break;

                default:
                    print_usage(*argv);
                    return EXIT_FAILURE;
            }
        }
        else { //handle old positional argument interface

            switch(++positional_args) {

                case 1:
                    args.step_time= atof(*argv);
                    break;

                case 2:  //block out and grbl out to same file, like before.
                    args.block_out_file = fopen(*argv,"w");
                    if (!args.block_out_file) {
                        perror("fopen");
                        printf("Error opening : %s\n",*argv);
                        return EXIT_FAILURE;
                    }
                    args.serial_out_file = args.block_out_file;
                    break;

                default:
                    print_usage(*argv);
                    return EXIT_FAILURE;
            }
        }
    }

    // GRBL2MACH: select the slice sink (at most one)
    if ((slice_file != NULL) + (fake_log != NULL) + use_usb > 1) {
        printf("Options -o, -f and -u are mutually exclusive.\n");
        return EXIT_FAILURE;
    }
    if (slice_file) {
        slice_sink_t *sink = sink_file_open(slice_file);
        if (!sink) {
            perror("fopen");
            printf("Error opening : %s\n", slice_file);
            return EXIT_FAILURE;
        }
        slicer_init(sink);
    }
    // GRBL2MACH: the machine's Mach3 profile describes it (axes, homing, inputs, outputs, spindle).
    // Required to drive the real controller; optional with the fake one (--mach3).
    if (use_usb || mach3_path) {
        char profile[1024], err[600];
        if (mach3_path)
            snprintf(profile, sizeof(profile), "%s", mach3_path);
        else
            mach3_default_path(profile, sizeof(profile), progname);
        if (!mach3_load(profile, err, sizeof(err))) {
            printf("Mach3 profile: %s\n", err);
            return EXIT_FAILURE;
        }
        rnr_options_t from_profile = rnr_opt;
        mach3_bridge_options(&from_profile);
        if (!cli_invert)
            rnr_opt.step_dir_invert = from_profile.step_dir_invert;
        if (!cli_out) {
            rnr_opt.spindle_out = from_profile.spindle_out;
            rnr_opt.spindle_ccw_out = from_profile.spindle_ccw_out;
            rnr_opt.flood_out = from_profile.flood_out;
            rnr_opt.mist_out = from_profile.mist_out;
        }
        if (!cli_in) {
            rnr_input_t estop = rnr_opt.inputs.estop;
            rnr_opt.inputs = from_profile.inputs;
            if (cli_estop)
                rnr_opt.inputs.estop = estop;
        }
        rnr_opt.pulse_khz = from_profile.pulse_khz;
    }

    if (fake_log || use_usb) {
        char info[256] = "";
        rnr_transport_t *t = use_usb ? rnr_hid_open(info, sizeof(info)) : rnr_fake_open(fake_log);
        if (!t) {
            printf(use_usb ? "No RNR Eco Motion controller found (VID 0x%04X, PID 0x%04X).\n" : "Cannot open fake controller log.\n", RNR_VID, RNR_PID);
            return EXIT_FAILURE;
        }
        if (use_usb) {
            printf("RNR controller: %s\n", info);
            fflush(stdout);
        }
        slice_sink_t *sink = sink_rnr_open(t, &rnr_opt);
        if (!sink) {
            printf("The RNR controller does not answer.\n");
            return EXIT_FAILURE;
        }
        slicer_init(sink);
        // Run the simulation slightly faster than real time: slice back-pressure then locks it to
        // the machine, and the controller never starves. TODO: dwells (G4) run 5% short this way.
        if (!speedup_given)
            args.speedup = 1.05f;
    }

    // Make sure the output streams are flushed immediately.
    // This is important when using the simulator inside another application in parallel
    // to the real grbl.
    // Theoretically flushing could be limited to complete lines. Unfortunately Windows
    // does not know line buffered streams. So for now we stick to flushing every character.
    //setvbuf(stdout, NULL, _IONBF, 1);
    //setvbuf(stderr, NULL, _IONBF, 1);
    //( Files are now closed cleanly when sim gets EOF or CTRL-F.)
    platform_init(); 

    sim.on_init = grbl_app_init;
    sim.on_shutdown = grbl_app_exit;
    sim.on_tick = grbl_per_tick;
    sim.on_byte = grbl_per_byte;

    init_simulator();

    if(args.port) {

#ifdef WIN32
        char port[10];
        WSADATA wsaData;
        struct addrinfo *result = NULL, *ptr = NULL, hints;

        ZeroMemory(&hints, sizeof (hints));
        hints.ai_family = AF_INET;
        hints.ai_socktype = SOCK_STREAM;
        hints.ai_protocol = IPPROTO_TCP;
        hints.ai_flags = AI_PASSIVE;

        if (WSAStartup(MAKEWORD(2,2), &wsaData) != 0) {
            printf("Fatal: Unable to create socket.\n");
            exit(-5);
        }

        if (getaddrinfo(listen_addr, itoa(args.port, port, 10), &hints, &result) != 0) {
            WSACleanup();
            printf("Fatal: Unable to create socket.\n");
            exit(-5);
        }

        if ((socket_fd = socket(result->ai_family, result->ai_socktype, result->ai_protocol)) == INVALID_SOCKET) {
            WSACleanup();
            freeaddrinfo(result);
            printf("Fatal: Unable to create socket.\n");
            exit(-5);
        }

        if (bind(socket_fd, result->ai_addr, (int)result->ai_addrlen) == SOCKET_ERROR) {
            freeaddrinfo(result);
            closesocket(socket_fd);
            WSACleanup();
            printf("Fatal: Unable to bind socket.\n");
            exit(-5);
        }

        freeaddrinfo(result);

        if (listen(socket_fd, SOMAXCONN ) == SOCKET_ERROR ) {
            closesocket(socket_fd);
            WSACleanup();
            exit(-5);
        }
        
        sim.socket_fd = INVALID_SOCKET;
#else

        socklen_t addrlen = sizeof(struct sockaddr_in);
        struct sockaddr_in server_addr = {0};

        if ((socket_fd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
            printf("Fatal: Unable to create socket.\n");
            exit(-5);
        }

        server_addr.sin_family = AF_INET;
        server_addr.sin_addr.s_addr = inet_addr(listen_addr);
        server_addr.sin_port = htons(args.port);

        if (bind(socket_fd, (struct sockaddr *) &server_addr, addrlen) < 0) {
            printf("Fatal: Unable to bind socket.\n");
            exit(-5);
        }

        FD_ZERO(&rfds);
        FD_SET(socket_fd, &rfds);

        listen(socket_fd, 1);
/*
        struct sockaddr_in client_addr;

        if ((sim.socket_fd = accept(socket_fd, (struct sockaddr *) &client_addr, &addrlen)) < 0) {
            printf("Fatal: Error on socket accept.\n");
            exit(-5);
        }

        FD_SET(sim.socket_fd, &rfds);

        struct timeval t = {
            .tv_sec = 0,
            .tv_usec = 0
        }; 

        setsockopt(sim.socket_fd, SOL_SOCKET, SO_RCVTIMEO, &t, sizeof(t));
*/
#endif
        printf("Listening on %s:%u\n", listen_addr, args.port);
        if (strcmp(listen_addr, "127.0.0.1") && strcmp(listen_addr, "localhost"))
            printf("WARNING: this port is reachable from the network. GRBL has no authentication:\n"
                   "         anyone who can reach it can move the machine.\n");
        fflush(stdout);

        sim.getchar = sim_socket_in;
        sim.putchar = sim_socket_out;

    } else {
        sim.getchar = platform_poll_stdin;
        sim.putchar = sim_serial_out;
    }

    //launch a thread with the original grbl code.
    irq_lock_init(irq_lock_on);
    plat_thread_t *th = platform_start_thread(grbl_main_thread);
    if (!th){
        printf("Fatal: Unable to start hardware thread.\n");
        exit(-5);
    }

    // Do not leave EEPROM file in an inconsistent state on ^C.
    atexit(eeprom_close);
    signal(SIGTERM, exithandler);

    // All the stream io and interrupt happen in this thread.
    sim_loop();

    // Graceful exit
    shutdown_simulator();

    platform_kill_thread(th); //need force kill since original main has no return.

    // close the files we opened
	if (args.block_out_file != stdout && args.block_out_file != args.serial_out_file)
		fclose(args.block_out_file);
	if (args.step_out_file != stderr)
		fclose(args.step_out_file);
	if (args.serial_out_file != stdout && args.serial_out_file != args.block_out_file)
		fclose(args.serial_out_file);

    if(args.port) {
#ifdef WIN32
        closesocket(socket_fd);
        WSACleanup();
#else
        if(sim.socket_fd)
            close(sim.socket_fd);
        close(socket_fd);
#endif
    }

    platform_terminate();

    exit(EXIT_SUCCESS);
}
