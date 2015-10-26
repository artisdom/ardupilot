
#include <cstdio>

#include <quantracker/osd/osd.hpp>
#include <task.h>

#include <cstring>
#include <stm32f4xx.h>
#include <quan/uav/osd/api.hpp>
#include <AP_OSD/AP_OSD_dequeue.h>
#include <AP_GPS/AP_GPS.h>

namespace {

   // the structure to receive the osd data
   AP_OSD::dequeue::osd_info_t aircraft_info;

   void do_gps(quan::uav::osd::pxp_type const & pos);

}

// do something on osd to check its running ok
void quan::uav::osd::on_draw() 
{ 
    AP_OSD::dequeue::read_stream(aircraft_info);

    pxp_type pos{-150,100};
    
    do_gps(pos);
    pos.y -= 20;

    char buf [100];

    sprintf(buf,"pitch   = %8.3f deg",static_cast<double>(aircraft_info.attitude.x));
    draw_text(buf,pos);
    pos.y -= 20;

    sprintf(buf,"roll    = %8.3f deg",static_cast<double>(aircraft_info.attitude.y));
    draw_text(buf,pos);
    pos.y -= 20;

    sprintf(buf,"yaw     = %8.3f deg",static_cast<double>(aircraft_info.attitude.z));
    draw_text(buf,pos);
    pos.y -= 20;

    sprintf(buf,"heading = %8.3f deg",static_cast<double>(aircraft_info.heading));
    draw_text(buf,pos);
    pos.y -= 20;

    sprintf(buf,"lat     = %8.3f deg",static_cast<double>(aircraft_info.gps_location.x)* 1e-7);
    draw_text(buf,pos);
    pos.y -= 20;

    sprintf(buf,"lon     = %8.3f deg",static_cast<double>(aircraft_info.gps_location.y)*1e-7);
    draw_text(buf,pos);
    pos.y -= 20;

    sprintf(buf,"alt     = %8.3f m",static_cast<double>(aircraft_info.gps_location.z) * 1e-2);
    draw_text(buf,pos);
    pos.y -= 20;

    sprintf(buf,"baroalt = %8.3f m",static_cast<double>(aircraft_info.baro_altitude) );
    draw_text(buf,pos);
    pos.y -= 20;

    sprintf(buf,"aispeed = %8.3f m/s",static_cast<double>(aircraft_info.airspeed) );
    draw_text(buf,pos);
      

}

namespace {

   void do_gps(quan::uav::osd::pxp_type const & pos)
   {
      using quan::uav::osd::draw_text;
      switch ( static_cast<AP_GPS::GPS_Status>(aircraft_info.gps_status) ){
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
}

