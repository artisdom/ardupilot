
#if CONFIG_HAL_BOARD == HAL_BOARD_QUAN
#include "Plane.h"

#include <quan/where.hpp>
#include <task.h>
#include <quantracker/osd/osd.hpp>

#include <AP_OSD/AP_OSD_dequeue.h>

namespace{

   AP_OSD::dequeue::osd_info_t info;
}

namespace Quan{
   AP_OSD::dequeue::osd_info_t const & get_osd_info();
}

AP_OSD::dequeue::osd_info_t const & Quan::get_osd_info()
   {return info;}

namespace {

   void do_startup_screen()
   {
      quan::uav::osd::pxp_type pos {-100,20};
      quan::uav::osd::draw_text("Air Flight Controller V1.0",pos); 
      pos.y -= 20;
      quan::uav::osd::draw_text("Press return * 3 for cli",pos); 
   }

   // ideally hand over to osd
   // need a way for osd to take over
   void do_cli_setup()
   {
      quan::uav::osd::pxp_type pos {-100,20};
      quan::uav::osd::draw_text("CLI setup",pos); 
   }

   void do_initialising_sensors()
   {
      quan::uav::osd::pxp_type pos {-100,20};
      quan::uav::osd::draw_text("initialising sensors",pos); 
   } 

   void do_waiting_for_gps()
   {
      quan::uav::osd::pxp_type pos {-100,20};
      quan::uav::osd::draw_text("waiting for gps fix",pos); 
   }

   void do_ready_to_fly()
   {
      quan::uav::osd::pxp_type pos {-100,20};
      quan::uav::osd::draw_text("ready_to_fly",pos); 
   }

   void do_flying()
   {
      quan::uav::osd::pxp_type pos {-100,20};
      quan::uav::osd::draw_text("flying",pos); 
   }
   
   void do_system_crashed()
   {
       quan::uav::osd::pxp_type pos {-100,20};
      quan::uav::osd::draw_text("system crashed",pos); 
    }

   void do_unknown()
   {
      quan::uav::osd::pxp_type pos {-100,20};
      quan::uav::osd::draw_text("unknown system state",pos); 
   }

}
 
void quan::uav::osd::on_draw() 
{ 
   AP_OSD::dequeue::read_stream(info);

   switch(Quan::get_osd_info().system_status){
     case AP_OSD::system_status_t::bootup:
         do_startup_screen();
         break;
     case AP_OSD::system_status_t::in_cli_setup:
         do_cli_setup();
         break;
     case  AP_OSD::system_status_t::initialising_sensors:
         do_initialising_sensors();
         break;
     case AP_OSD::system_status_t::waiting_for_gps:
         do_waiting_for_gps();
         break;
     case AP_OSD::system_status_t::ready_to_fly:
         do_ready_to_fly();
         break;
     case AP_OSD::system_status_t::flying:
         do_flying();
         break;
     case AP_OSD::system_status_t::system_crashed:
         do_system_crashed();
         break;
     default:
         do_unknown();
         break;

   }
    
}

#endif  // #if CONFIG_HAL_BOARD == HAL_BOARD_QUAN