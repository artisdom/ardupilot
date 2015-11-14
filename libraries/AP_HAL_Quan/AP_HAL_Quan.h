
#ifndef __AP_HAL_QUAN_H__
#define __AP_HAL_QUAN_H__

/* Your layer exports should depend on AP_HAL.h ONLY. */
#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_QUAN
/**
 * Umbrella header for AP_HAL_Quan module.
 * The module header exports singleton instances which must conform the
 * AP_HAL::HAL interface. It may only expose implementation details (class
 * names, headers) via the Quan namespace.
 * The class implementing AP_HAL::HAL should be called HAL_Quan and exist
 * in the global namespace. There should be a single const instance of the
 * HAL_Quan class called AP_HAL_Quan, instantiated in the HAL_Quan_Class.cpp
 * and exported as `extern const HAL_Quan AP_HAL_Quan;` in HAL_Quan_Class.h
 *
 * All declaration and compilation should be guarded by CONFIG_HAL_BOARD macros.
 * In this case, we're using CONFIG_HAL_BOARD == HAL_BOARD_QUAN.
 * When creating a new HAL, declare a new HAL_BOARD_ in AP_HAL/AP_HAL_Boards.h
 *
 * The module should also export an appropriate AP_HAL_MAIN() macro iff the
 * appropriate CONFIG_HAL_BOARD value is set.
 * The AP_HAL_MAIN macro expands to a main function (either an `int main (void)`
 * or `int main (int argc, const char * argv[]), depending on platform) of an
 * ArduPilot application, whose entry points are the c++ functions
 * `void setup()` and `void loop()`, ala Arduino.
 */

#include <quan/min.hpp>
#include <quan/max.hpp>
#include "HAL_Quan_Class.h"
#include "AP_HAL_Quan_Main.h"
#endif
#endif //__AP_HAL_QUAN_H__

