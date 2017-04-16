
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

   float constexpr stbd_ail_inner_dir = 1.f;
   float constexpr stbd_ail_outer_dir = 1.f;
   float constexpr port_ail_inner_dir = -1.f;
   float constexpr port_ail_outer_dir = -1.f;

   float constexpr ail_inner_horn_len = 1.f;
   float constexpr ail_outer_horn_len = 1.f;

   float constexpr up_ail_gain = 0.75f;
   float constexpr down_ail_gain = 0.5f;

   float constexpr up_ail_inner_gain = up_ail_gain * ail_inner_horn_len;
   float constexpr up_ail_outer_gain = up_ail_gain * ail_outer_horn_len;
   float constexpr down_ail_inner_gain = down_ail_gain * ail_inner_horn_len;
   float constexpr down_ail_outer_gain = down_ail_gain * ail_outer_horn_len;

   float constexpr up_flap_gain = 0.1f;
   float constexpr down_flap_gain = 0.4f;

   float constexpr up_flap_inner_gain = up_flap_gain * ail_inner_horn_len;
   float constexpr down_flap_inner_gain = down_flap_gain * ail_inner_horn_len;
   float constexpr up_flap_outer_gain = up_flap_gain * ail_outer_horn_len;
   float constexpr down_flap_outer_gain = down_flap_gain * ail_outer_horn_len;

   float constexpr rudder_roll_gain = -0.3f;
   float constexpr elev_flap_gain = -0.1f;

   float output[7] = {0.f,0.f,0.f,0.f,0.f,0.f,0.f};

   void output_action(uint8_t channel)
   {
       float const v1 = (((output[channel] + 1.f)/2.f) + 1.f) * 1000.f;
       uint16_t const out = quan::constrain(static_cast<uint16_t>(v1),static_cast<uint16_t>(1000U),static_cast<uint16_t>(2000U)); 
       hal.rcout->write(channel,out);
   }

   void mixer_eval()
   {
     float const roll = -plane.get_roll_demand();
     bool const positive_roll = roll > 0.f;
     float const port_inner_ail = roll * (positive_roll? up_ail_inner_gain: down_ail_inner_gain);
     float const port_outer_ail = roll * (positive_roll? up_ail_outer_gain: down_ail_outer_gain);
     float const stbd_inner_ail = -roll * (positive_roll? down_ail_inner_gain: up_ail_inner_gain);
     float const stbd_outer_ail = -roll * (positive_roll? down_ail_outer_gain: up_ail_outer_gain);
     float const flap = - plane.get_flap_demand();
     bool const  up_flap = flap < 0.f;
     float const inner_flap = flap * (up_flap?up_flap_inner_gain:down_flap_inner_gain);
     float const outer_flap = flap * (up_flap?up_flap_outer_gain:down_flap_outer_gain);
     output[throttle] = plane.get_thrust_demand();
     output[port_ail_inner] = (port_inner_ail - inner_flap) * port_ail_inner_dir;
     output[port_ail_outer] = (port_outer_ail - outer_flap) * port_ail_outer_dir;
     output[stbd_ail_inner] = (stbd_inner_ail - inner_flap) * stbd_ail_inner_dir;
     output[stbd_ail_outer] = (stbd_outer_ail - outer_flap) * stbd_ail_outer_dir;
     float const elev_to_flap = (up_flap?up_flap_gain:down_flap_gain) * elev_flap_gain * flap;
     float const roll_to_rudder = roll * rudder_roll_gain;
     float const pitch = plane.get_pitch_demand();
     float const yaw = plane.get_yaw_demand();
     output[port_v_tail] = pitch * 0.5f + yaw * 0.5f + roll_to_rudder + elev_to_flap;
     output[stbd_v_tail] = -pitch * 0.5f + yaw * 0.5f + roll_to_rudder - elev_to_flap;

     for ( uint8_t i = 0; i < 7; ++i){
         output_action(i);
     }

   }

}

bool Plane::create_mixer()
{
   return true;
//   apm_lexer::cstrstream_t stream{mixer_string,500}; 
//
//   return apm_mix::mixer_create(
//      &stream
//      ,inputs, sizeof(inputs)/sizeof(inputs[0])
//      ,outputs, sizeof(outputs)/sizeof(outputs[0])
//   ); 
}

void Plane::mix()
{
  //  apm_mix::mixer_eval();
    mixer_eval();
}









