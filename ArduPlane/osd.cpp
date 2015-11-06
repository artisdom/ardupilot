
#if CONFIG_HAL_BOARD == HAL_BOARD_QUAN
#include "Plane.h"

#include <quan/where.hpp>
#include <task.h>
#include <quantracker/osd/osd.hpp>

#include <AP_OSD/AP_OSD_dequeue.h>
#include <AP_OSD/fonts.hpp>

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
      quan::uav::osd::draw_text("Press return * 3 for cli",pos,Quan::FontID::MWOSD); 
   }

   // ideally hand over to osd
   // need a way for osd to take over
   void do_cli()
   {
      quan::uav::osd::pxp_type pos {-100,20};
      quan::uav::osd::draw_text("CLI setup",pos,Quan::FontID::MWOSD); 
   }

   void do_initialising()
   {
      quan::uav::osd::pxp_type pos {-100,20};
      quan::uav::osd::draw_text("initialising",pos); 
      pos.y -= 20;
      quan::uav::osd::draw_text("Press return * 3 for cli",pos,Quan::FontID::MWOSD); 
   } 

   void do_running()
   {
      quan::uav::osd::pxp_type pos {-100,20};
      quan::uav::osd::draw_text("running",pos); 
   }

   void do_unknown()
   {
      quan::uav::osd::pxp_type pos {-100,20};
      quan::uav::osd::draw_text("unknown system state - report",pos); 
   }
   
}
 
void quan::uav::osd::on_draw() 
{ 
   AP_OSD::dequeue::read_stream(info);

   switch(Quan::get_osd_info().system_status){
     case AP_OSD::system_status_t::starting:
         do_startup_screen();
         break;
     case AP_OSD::system_status_t::in_cli:
         do_cli();
         break;
     case  AP_OSD::system_status_t::initialising:
         do_initialising();
         break;
     case AP_OSD::system_status_t::running:
         do_running();
         break;
     default:
         do_unknown();
         break;

   }
    
}

#endif  // #if CONFIG_HAL_BOARD == HAL_BOARD_QUAN