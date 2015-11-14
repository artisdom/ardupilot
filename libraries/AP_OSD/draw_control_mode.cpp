#include <quan/uav/osd/api.hpp>

#include <AP_OSD/AP_OSD_dequeue.h>
#include "bitmaps.hpp"
#include "fonts.hpp"

namespace {

/*
enum FlightMode {
    MANUAL        = 0,
    CIRCLE        = 1,
    STABILIZE     = 2,
    TRAINING      = 3,
    ACRO          = 4,
    FLY_BY_WIRE_A = 5,
    FLY_BY_WIRE_B = 6,
    CRUISE        = 7,
    AUTOTUNE      = 8,
    AUTO          = 10,
    RTL           = 11,
    LOITER        = 12,
    GUIDED        = 15,
    INITIALISING  = 16
};
*/

   constexpr char strings [] [7] = {
      "MANUAL"
      ,"CIRCLE"   
      ,"STABIL"    
      ,"TRAING"     
      ,"ACRO  "        
      ,"FLYBWA"
      ,"FLYBWB"
      ,"CRUISE"       
      ,"AUTUNE" 
      ,"UNDEF1"    
      ,"AUTO  "        
      ,"RETL  "           
      ,"LOITER"  
      ,"UNDEF2"
      ,"UNDEF3"
      ,"GUIDED"      
      ,"INITIA"  
   };

}

void AP_OSD::draw_control_mode(dequeue::osd_info_t const & info,OSD_params const & osd)
{
   uint32_t strid = info.control_mode;
   if ( strid < (sizeof(strings) / sizeof(strings[0])) ){

      quan::uav::osd::pxp_type const pos = 
            {osd.control_mode_pos.x,
            (( quan::uav::osd::get_video_mode() == quan::uav::osd::video_mode::pal)
               ?osd.control_mode_pos.y
               :osd.control_mode_pos.z)
            };

      quan::uav::osd::draw_text(strings[strid],pos,Quan::FontID::MWOSD);

   }

}