
#include "Plane.h"

extern const AP_HAL::HAL& hal;

namespace {

   uint8_t constexpr port_v_tail = 0;
   uint8_t constexpr stbd_v_tail = 6;
   uint8_t constexpr throttle = 1;
   uint8_t constexpr stbd_ail_outer = 2;
   uint8_t constexpr stbd_ail_inner = 3;
   uint8_t constexpr port_ail_inner = 4;
   uint8_t constexpr port_ail_outer = 5;

   float constexpr stbd_ail_inner_dir = -1.f;
   float constexpr stbd_ail_outer_dir = -1.f;
   float constexpr port_ail_inner_dir = 1.f;
   float constexpr port_ail_outer_dir = 1.f;

   float constexpr ail_inner_horn_len = 0.7f;
   float constexpr ail_outer_horn_len = 1.f;

   float constexpr up_ail_gain = 0.75f;
   float constexpr down_ail_gain = 0.5f;

   float constexpr up_ail_inner_gain = up_ail_gain * ail_inner_horn_len;
   float constexpr up_ail_outer_gain = up_ail_gain * ail_outer_horn_len;
   float constexpr down_ail_inner_gain = down_ail_gain * ail_inner_horn_len;
   float constexpr down_ail_outer_gain = down_ail_gain * ail_outer_horn_len;

   float constexpr up_flap_gain = 0.1f;
   float constexpr down_flap_gain = 0.3f;

   float constexpr up_flap_inner_gain = up_flap_gain * ail_inner_horn_len;
   float constexpr down_flap_inner_gain = down_flap_gain * ail_inner_horn_len;
   float constexpr up_flap_outer_gain = up_flap_gain * ail_outer_horn_len;
   float constexpr down_flap_outer_gain = down_flap_gain * ail_outer_horn_len;

   float constexpr rudder_roll_gain = -0.3f;
   float constexpr elev_flap_gain = 0.1f;

   float constexpr crow_up_gain = 0.3f;
   float constexpr crow_down_gain = 0.5f;
   float constexpr elev_to_crow_gain = 0.1f;

  //################################
   
   float output[7] = {0.f,0.f,0.f,0.f,0.f,0.f,0.f};

   void output_action(uint8_t channel)
   {
       float const v1 = (output[channel] + 3.f) * 500.f;
       uint16_t const out = quan::constrain(static_cast<uint16_t>(v1),static_cast<uint16_t>(1000U),static_cast<uint16_t>(2000U)); 
       hal.rcout->write(channel,out);
   }

   //###############################

   void mixer_eval()
   {
     bool const manual_mode = plane.get_control_mode() == MANUAL;
     bool const crow_mode = manual_mode && (hal.rcin->read(6) > 1750);
     bool const crow_active = crow_mode && (plane.get_thrust_demand() < 0.f);

     float const roll = -plane.get_roll_demand();
     bool const positive_roll = roll > 0.f;
     float const port_inner_ail = roll * (positive_roll? up_ail_inner_gain: down_ail_inner_gain);
     float const port_outer_ail = roll * (positive_roll? up_ail_outer_gain: down_ail_outer_gain);
     float const stbd_inner_ail = -roll * (positive_roll? down_ail_inner_gain: up_ail_inner_gain);
     float const stbd_outer_ail = -roll * (positive_roll? down_ail_outer_gain: up_ail_outer_gain);
   
     float const thrust_in = plane.get_thrust_demand();
     if ( crow_mode){
        output[throttle] = thrust_in * 2.f - 1.f;
     }else{
        output[throttle] = thrust_in;
     }
     float const crow = (crow_active ?-thrust_in * 2.f : 0.f);
     float const inner_crow = (crow - 0.5f) * crow_down_gain;
     float const outer_crow = (crow - 0.5f) * crow_up_gain;
     
     float const flap_in = plane.get_flap_demand();
     float const flap = (crow_mode == false) 
      ? -flap_in
      : (crow_active ? 1.f - crow : 1.f);
     bool const  up_flap = flap < 0.f;
     float const inner_flap = flap * (up_flap?up_flap_inner_gain:down_flap_inner_gain);
     float const outer_flap = flap * (up_flap?up_flap_outer_gain:down_flap_outer_gain);
     
     output[port_ail_inner] = (port_inner_ail - inner_flap - inner_crow) * port_ail_inner_dir;
     output[stbd_ail_inner] = (stbd_inner_ail - inner_flap - inner_crow) * stbd_ail_inner_dir;
     output[port_ail_outer] = (port_outer_ail - outer_flap + outer_crow) * port_ail_outer_dir;
     output[stbd_ail_outer] = (stbd_outer_ail - outer_flap + outer_crow) * stbd_ail_outer_dir;

     float const elev_to_flap = (up_flap?up_flap_gain:down_flap_gain) * elev_flap_gain * flap;
     float const roll_to_rudder = roll * rudder_roll_gain;
     float const elev_to_crow = elev_to_crow_gain * crow;
     float const pitch = plane.get_pitch_demand();
     float const yaw = plane.get_yaw_demand();
     output[port_v_tail] = pitch * 0.5f + yaw * 0.5f + roll_to_rudder + elev_to_flap + elev_to_crow;
     output[stbd_v_tail] = -pitch * 0.5f + yaw * 0.5f + roll_to_rudder - elev_to_flap - elev_to_crow;

     for ( uint8_t i = 0; i < 7; ++i){
         output_action(i);
     }

   }

}

bool Plane::create_mixer()
{
   return true;
}

void Plane::mix()
{
    mixer_eval();
}









