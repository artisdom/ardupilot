// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// This file is just a placeholder for your configuration file.  If
// you wish to change any of the setup parameters from their default
// values, place the appropriate #define statements here.

// If you used to define your CONFIG_APM_HARDWARE setting here, it is no
// longer valid! You should switch to using CONFIG_HAL_BOARD via the HAL_BOARD
// flag in your local config.mk instead.

#define GPS_PROTOCOL  GPS_PROTOCOL_AUTO

#define RANGEFINDER_ENABLED DISABLED

#define MAGNETOMETER		         ENABLED
#define MAG_ORIENTATION	         AP_COMPASS_COMPONENTS_DOWN_PINS_FORWARD

// does nothing
#define AIRSPEED_SENSOR          ENABLED
/*
default 
airspeed_sensor.enabled  in AP_Airspeed.cpp
*/

#define LOGGING_ENABLED          DISABLED
#define MOUNT                    DISABLED
#define CAMERA                   DISABLED
#define FRSKY_TELEM_ENABLED      DISABLED
#define OBC_FAILSAFE             DISABLED
#define OPTFLOW                  DISABLED

