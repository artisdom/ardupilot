
#include <cstdio>
#include <quan/uav/osd/api.hpp>
#include <AP_OSD/AP_OSD_dequeue.h>
#include "fonts.hpp"

using namespace quan::uav::osd;

void AP_OSD::draw_airspeed(
      AP_OSD::dequeue::osd_info_t const & info,
      AP_OSD::OSD_params const & osd)
{
  // IAS

   pxp_type pos       {osd.airspeed_pos.x,
      (( get_video_mode() == video_mode::pal)?osd.airspeed_pos.y:osd.airspeed_pos.z)};
   char buf [30];
   sprintf(buf,"IAS %5.1f m/s",static_cast<double>(info.airspeed.numeric_value()));
   draw_text(buf,pos,Quan::FontID::MWOSD);
   
}

//param is 