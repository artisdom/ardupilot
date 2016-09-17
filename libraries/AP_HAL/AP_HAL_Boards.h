/**
 * C preprocesor enumeration of the boards supported by the AP_HAL.
 * This list exists so HAL_BOARD == HAL_BOARD_xxx preprocessor blocks
 * can be used to exclude HAL boards from the build when appropriate.
 * Its not an elegant solution but we can improve it in future.
 */
#pragma once

#define HAL_BOARD_SITL     3
#define HAL_BOARD_SMACCM   4  // unused
#define HAL_BOARD_PX4      5
#define HAL_BOARD_LINUX    7
#define HAL_BOARD_VRBRAIN  8
#define HAL_BOARD_QURT     9
#define HAL_BOARD_EMPTY   99

/* Default board subtype is -1 */
#define HAL_BOARD_SUBTYPE_NONE -1

/* Heat Types */
#define HAL_LINUX_HEAT_PWM 1

/* CPU classes, used to select if CPU intensive algorithms should be used
 * Note that these are only approximate, not exact CPU speeds. */

/* DEPRECATED: 16Mhz: AVR2560 or similar */
#define HAL_CPU_CLASS_16   1
/* 150Mhz: PX4 or similar. Assumes:
 *  - hardware floating point
 *  - tens of kilobytes of memory available */
#define HAL_CPU_CLASS_150  3
/* GigaHz class: SITL, BeagleBone etc. Assumes megabytes of memory available. */
#define HAL_CPU_CLASS_1000 4

//----------------------specific boards --------------------

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL

   #include <AP_HAL/boards/HALSITL.h>

#elif CONFIG_HAL_BOARD == HAL_BOARD_PX4

   #include <AP_HAL/boards/PX4.h>

#elif CONFIG_HAL_BOARD == HAL_BOARD_LINUX

   #include <AP_HAL/boards/Linux.h>

#elif CONFIG_HAL_BOARD == HAL_BOARD_EMPTY

   #include <AP_HAL/boards/EMPTY.h>

#elif CONFIG_HAL_BOARD == HAL_BOARD_QURT

   #include <AP_HAL/boards/Qurt.h>

#elif CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN

  #include <AP_HAL/boards/VRBRAIN.h>

#else

  #error "Unknown CONFIG_HAL_BOARD type"

#endif

//--------------------------- ~ specific boards ---------------

#ifndef CONFIG_HAL_BOARD_SUBTYPE

   #error "No CONFIG_HAL_BOARD_SUBTYPE set"

#endif

// seems unused
//#ifndef HAL_COMPASS_DEFAULT
//#define HAL_COMPASS_DEFAULT -1
//#endif

// operating system
#ifndef HAL_OS_POSIX_IO
#define HAL_OS_POSIX_IO 0
#endif

#ifndef HAL_OS_SOCKETS
#define HAL_OS_SOCKETS 0
#endif

// if there is a file sytem
#ifndef HAL_PARAM_DEFAULTS_PATH
#define HAL_PARAM_DEFAULTS_PATH NULL
#endif
