// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
//
// Test for AP_GPS_AUTO
//

#include <stdlib.h>

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_GPS/AP_GPS.h>
#include <DataFlash/DataFlash.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_ADC/AP_ADC.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <AP_Baro/AP_Baro.h>
#include <Filter/Filter.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Compass/AP_Compass.h>
#include <AP_Declination/AP_Declination.h>
#include <AP_Airspeed/AP_Airspeed.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <AP_ADC_AnalogSource/AP_ADC_AnalogSource.h>
#include <AP_Mission/AP_Mission.h>
#include <StorageManager/StorageManager.h>
#include <AP_Terrain/AP_Terrain.h>
#include <AP_Math/AP_Math.h>
#include <AP_Notify/AP_Notify.h>
#include <AP_Notify/AP_BoardLED.h>
#include <AP_NavEKF/AP_NavEKF.h>
#include <AP_Rally/AP_Rally.h>
#include <AP_Scheduler/AP_Scheduler.h>
#include <AP_BattMonitor/AP_BattMonitor.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_RangeFinder/AP_RangeFinder.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

// create board led object
AP_BoardLED board_led;

// This example uses GPS system. Create it.
AP_GPS gps;

// Serial manager is needed for UART comunications
AP_SerialManager serial_manager;

#define T6 1000000
#define T7 10000000

void setup()
{
    hal.console->println("GPS AUTO library test");

    // Initialise the leds
    board_led.init();

    // Initialize the UART for GPS system
    serial_manager.init();
    gps.init(NULL, serial_manager);
}

namespace {

    AP_GPS::GPS_Status df_gps_status = AP_GPS::NO_GPS;

    uint8_t led_update_count = 0;
}

#if CONFIG_HAL_BOARD == HAL_BOARD_QUAN
void quan::uav::osd::on_draw() 
{ 
/*
  NO_GPS = 0,             ///< No GPS connected/detected
        NO_FIX = 1,             ///< Receiving valid GPS messages but no lock
        GPS_OK_FIX_2D = 2,      ///< Receiving valid messages and 2D lock
        GPS_OK_FIX_3D = 3,      ///< Receiving valid messages and 3D lock
        GPS_OK_FIX_3D_DGPS = 4, ///< Receiving valid messages and 3D lock with differential improvements
        GPS_OK_FIX_3D_RTK = 5,  ///<
*/
   pxp_type pos{-140,50};
   switch ( df_gps_status){
       case AP_GPS::NO_GPS:
         draw_text("No GPS",pos);
         break;
       case AP_GPS::NO_FIX:
         draw_text("No Fix",pos);
         break;
       case AP_GPS::GPS_OK_FIX_2D:
         draw_text("2D Fix",pos);
         break;
       case AP_GPS::GPS_OK_FIX_3D:
         draw_text("3D Fix",pos);
         break;
       case AP_GPS::GPS_OK_FIX_3D_DGPS:
         draw_text("3D Fix dgps",pos);
         break;
       case AP_GPS::GPS_OK_FIX_3D_RTK:
         draw_text("3D Fix RTK",pos);
         break;
       default:
         draw_text("GPS state out of range",pos);
         break;
   }
   
}
#endif

void loop()
{
    static uint32_t last_msg_ms;

    // Update GPS state based on possible bytes received from the module.
    gps.update();

    // If new GPS data is received, output it's contents to the console
    // Here we rely on the time of the message in GPS class and the time of last message
    // saved in static variable last_msg_ms. When new message is received, the time
    // in GPS class will be updated.
    if (last_msg_ms != gps.last_message_time_ms()) {
        // Reset the time of message
        last_msg_ms = gps.last_message_time_ms();

        // Acquire location
        const Location &loc = gps.location();

        // Print the contents of message
        hal.console->print("Lat: ");
        print_latlon(hal.console, loc.lat);
        hal.console->print(" Lon: ");
        print_latlon(hal.console, loc.lng);
        hal.console->printf(" Alt: %.2fm GSP: %.2fm/s CoG: %d SAT: %d TIM: %u/%lu STATUS: %u\n",
                            loc.alt * 0.01f,
                            gps.ground_speed(),
                            (int)gps.ground_course_cd() / 100,
                            gps.num_sats(),
                            gps.time_week(),
                            (unsigned long)gps.time_week_ms(),
                            gps.status());

        df_gps_status = gps.status();
       
    }
    if ( ++led_update_count ==2){
       led_update_count = 0;
       board_led.update();
    }
    // Delay for 10 mS will give us 100 Hz invocation rate
    hal.scheduler->delay(10);
}

// Register above functions in HAL board level
AP_HAL_MAIN();
