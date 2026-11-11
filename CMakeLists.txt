cmake_minimum_required(VERSION 3.10)
project(GRBL2MACH VERSION 0.1.3 LANGUAGES C)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS Debug Release RelWithDebInfo MinSizeRel)
endif()

if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    # ISR handlers run on a separate host thread; keep type-punning-heavy
    # embedded code out of strict-aliasing territory.
    add_compile_options(-fno-strict-aliasing)
endif()

if(MINGW)
    # GRBL2MACH: MinGW's <string.h> declares the legacy strset(char *, int), which clashes
    # with the core's strset() (grbl/strutils.h). See cmake/mingw_strset_fix.h.
    add_compile_options(-include ${CMAKE_CURRENT_LIST_DIR}/cmake/mingw_strset_fix.h)
endif()

add_compile_definitions(F_CPU=16000000)
# GRBL2MACH: X, Y, Z and A, the four axes of the RNR controller (the upstream simulator builds 3)
add_compile_definitions(N_AXIS=4)
#add_compile_definitions(SQUARING_ENABLED=1)

# Capture the effective build-time flags for the -v/--version output.
string(TOUPPER "${CMAKE_BUILD_TYPE}" _sim_build_type)
if(_sim_build_type STREQUAL "")
    set(SIM_BUILD_TYPE "None")
else()
    set(SIM_BUILD_TYPE "${CMAKE_BUILD_TYPE}")
endif()
string(STRIP "${CMAKE_C_FLAGS} ${CMAKE_C_FLAGS_${_sim_build_type}}" SIM_C_FLAGS)
configure_file(
    ${CMAKE_CURRENT_LIST_DIR}/src/build_info.h.in
    ${CMAKE_CURRENT_BINARY_DIR}/build_info.h
    @ONLY
)

include(src/grbl/CMakeLists.txt)
include(src/sdcard/CMakeLists.txt)

if (WIN32)
    add_compile_definitions(PLATFORM_WINDOWS)

    set(platform_SRC
        src/platform_windows.h
        src/platform_windows.c
    )

    set(platform_LIB
        ws2_32
        winmm  # timeBeginPeriod/timeEndPeriod (1 ms Sleep granularity)
        hid setupapi  # GRBL2MACH: RNR USB HID controller
    )
endif(WIN32)

if(UNIX)
    add_compile_definitions(PLATFORM_LINUX)

    set(platform_SRC
        src/platform_linux.h
        src/platform_linux.c
    )

    if(APPLE)
        set(platform_LIB
            pthread
        )
    else(APPLE)
        set(platform_LIB
            rt
            pthread
        )
    endif(APPLE)
endif(UNIX)

add_executable(grbl2mach 
    src/main.c 
    src/simulator.c
    src/driver.c
    src/eeprom.c
    src/grbl_eeprom_extensions.c
    src/mcu.c
    src/serial.c
    src/grbl_interface.c
	src/fs_posix.c
    src/slicer.c
    src/mach3.c
    src/irq_lock.c
    src/sink_file.c
    src/rnr/rnr_protocol.c
    src/rnr/rnr_os.c
    src/rnr/rnr_hid_win.c
    src/rnr/rnr_fake.c
    src/rnr/sink_rnr.c
    ${platform_SRC}
)

if (WIN32)
    # GRBL2MACH: icon and version shown by Windows
    enable_language(RC)
    configure_file(${CMAKE_CURRENT_LIST_DIR}/src/grbl2mach.rc.in ${CMAKE_CURRENT_BINARY_DIR}/grbl2mach.rc @ONLY)
    target_sources(grbl2mach PRIVATE ${CMAKE_CURRENT_BINARY_DIR}/grbl2mach.rc src/gui_win.c)
    # GRBL2MACH: status window
    target_link_libraries(grbl2mach PRIVATE comctl32 comdlg32 uxtheme)
    set_source_files_properties(${CMAKE_CURRENT_BINARY_DIR}/grbl2mach.rc PROPERTIES
        OBJECT_DEPENDS "${CMAKE_CURRENT_LIST_DIR}/packaging/grbl2mach.ico;${CMAKE_CURRENT_LIST_DIR}/src/grbl2mach.manifest")
endif(WIN32)

target_link_libraries(grbl2mach PRIVATE
    m
    grbl
	sdcard
    ${platform_LIB}
)

# Generated build_info.h lands in the build dir.
target_include_directories(grbl2mach PRIVATE src ${CMAKE_CURRENT_BINARY_DIR})

add_executable(grbl2mach_validator
    src/eeprom.c
    src/grbl_eeprom_extensions.c
    src/validator.c 
    src/validator_driver.c 
    ${platform_SRC}
)

target_link_libraries(grbl2mach_validator PRIVATE 
    m
    grbl
    ${platform_LIB}
)
