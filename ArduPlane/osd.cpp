/*
 Copyright (c) 2003-2014 Andy Little.

 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program. If not, see http://www.gnu.org/licenses./
 */

#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_QUAN
#include "Plane.h"

#include <quan/where.hpp>
#include <task.h>
#include <quantracker/osd/osd.hpp>

#include <AP_OSD/AP_OSD_dequeue.h>
#include <AP_OSD/fonts.hpp>

namespace{

   AP_OSD::dequeue::osd_info_t info;
   AP_OSD::OSD_params osd;
}

namespace Quan{
   AP_OSD::dequeue::osd_info_t const & get_osd_info();
}

AP_OSD::dequeue::osd_info_t const & Quan::get_osd_info()
   {return info;}

namespace {

   void do_startup_screen()
   {
      quan::uav::osd::pxp_type pos {-140,20};
      quan::uav::osd::draw_text("Air Flight Controller V1.0",pos); 
      pos.y -= 20;
      quan::uav::osd::draw_text("Press return x 3 CLI",pos,Quan::FontID::MWOSD); 
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
      quan::uav::osd::pxp_type pos {-140,20};
      quan::uav::osd::draw_text("initialising",pos); 
     
      pos.y -= 20;
      quan::uav::osd::draw_text("Press return x 3 for CLI",pos,Quan::FontID::MWOSD); 
   } 

   void do_running()
   {
     AP_OSD::draw_artificial_horizon(info,osd);
     AP_OSD::draw_compass(info,osd);
     AP_OSD::draw_batteries(info,osd);
     AP_OSD::draw_gps(info,osd);
     AP_OSD::draw_control_mode(info,osd);
     AP_OSD::draw_airspeed(info,osd);
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

   AP_OSD::dequeue::update(info);

   switch(Quan::get_osd_info().system_status){
     case AP_OSD::system_status_t::starting:
         do_startup_screen();
         break;
     case AP_OSD::system_status_t::in_cli:
         do_cli();
         break;
     case AP_OSD::system_status_t::initialising:
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
