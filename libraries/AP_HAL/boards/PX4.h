#ifndef ARDUPILOT_LIBRARIES_AP_HAL_BOARDS_PX4_H_INCLUDED
#define ARDUPILOT_LIBRARIES_AP_HAL_BOARDS_PX4_H_INCLUDED

#define HAL_BOARD_SUBTYPE_PX4_V1           2000
#define HAL_BOARD_SUBTYPE_PX4_V2           2001

// my hunch is that these can be removed for PX4 eventually
// since all peripherals are PX4
#include <AP_HAL/sensors/Barometer.h>
#include <AP_HAL/sensors/Compass.h>
#include <AP_HAL/sensors/InertialSensor.h>

namespace PX4{
     struct Tag_BoardType;
}

namespace AP_HAL{
  typedef ::PX4::Tag_BoardType Tag_BoardType;
}

#define HAL_BOARD_NAME "PX4"
#define HAL_CPU_CLASS HAL_CPU_CLASS_150
#define HAL_OS_POSIX_IO 1
#define HAL_BOARD_LOG_DIRECTORY "/fs/microsd/APM/LOGS"
#define HAL_BOARD_TERRAIN_DIRECTORY "/fs/microsd/APM/TERRAIN"
#define HAL_PARAM_DEFAULTS_PATH "/etc/defaults.parm"
#define HAL_INS_DEFAULT HAL_INS_PX4
#define HAL_BARO_DEFAULT HAL_BARO_PX4
#define HAL_COMPASS_DEFAULT HAL_COMPASS_PX4
#define HAL_SERIAL0_BAUD_DEFAULT 115200
#ifdef CONFIG_ARCH_BOARD_PX4FMU_V1
#define CONFIG_HAL_BOARD_SUBTYPE HAL_BOARD_SUBTYPE_PX4_V1
#define HAL_STORAGE_SIZE            8192
#else
#define CONFIG_HAL_BOARD_SUBTYPE HAL_BOARD_SUBTYPE_PX4_V2
#define HAL_STORAGE_SIZE            16384
#define HAL_HAVE_IMU_HEATER         1 // for Pixhawk2
#define HAL_IMU_TEMP_DEFAULT       -1 // disabled
#endif
#define HAL_GPIO_A_LED_PIN        27
#define HAL_GPIO_B_LED_PIN        26
#define HAL_GPIO_C_LED_PIN        25
#define HAL_GPIO_LED_ON           LOW
#define HAL_GPIO_LED_OFF          HIGH




#endif // ARDUPILOT_LIBRARIES_AP_HAL_BOARDS_PX4_H_INCLUDED
