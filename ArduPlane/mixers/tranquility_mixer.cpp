
namespace {

   // servo output channels
   uint8_t constexpr stbd_aileron_channel = 0U;  // OutCh1
   uint8_t constexpr stbd_v_tail_channel = 1U;   //OutCh2
   uint8_t constexpr throttle_channel = 2U;      //OutCh3
   // power_supply on ch                         // PSU on CH 4
   uint8_t constexpr port_v_tail_channel = 4U;    //OutCh5
   uint8_t constexpr port_aileron_channel = 5U;   // OutCh6

   float constexpr stbd_aileron_dir = 1.f;
   float constexpr port_aileron_dir = -stbd_aileron_dir;
   float constexpr port_v_tail_dir = 1;
   float constexpr stbd_v_tail_dir = - port_v_tail_dir;

   float constexpr up_ail_gain = 0.75f;
   float constexpr down_ail_gain = 0.5f;

   float constexpr up_flap_gain = 0.1f;
   float constexpr down_flap_gain = 0.3f;

   float constexpr rudder_roll_gain = -0.3f;
   float constexpr elev_flap_gain = 0.1f;

   float constexpr crow_gain = 0.715f;
   float constexpr elev_to_crow_gain = -0.2f;

   uint8_t constexpr num_outputs = 6;
   float output[num_outputs] = {0.f,0.f,0.f,0.f,0.f,0.f};

   void mixer_eval()
   {
     bool const manual_mode = plane.get_control_mode() == MANUAL;
     bool const crow_mode = manual_mode && (hal.rcin->read(6) > 1750);
     bool const crow_active = crow_mode && (plane.get_thrust_demand() < 0.f);

     float const roll = -plane.get_roll_demand();
     bool const positive_roll = roll > 0.f;
     float const port_aileron = roll * (positive_roll? up_ail_gain: down_ail_gain);
     float const stbd_aileron = -roll * (positive_roll? down_ail_gain: up_ail_gain);
   
     float const thrust_in = plane.get_thrust_demand();
     if ( crow_mode){
        output[throttle_channel] = thrust_in * 2.f - 1.f;
     }else{
        output[throttle_channel] = thrust_in;
     }
     float const crow = (crow_active ?-thrust_in * 2.f : 0.f);
     float const ail_crow = (crow - 0.5f) * crow_gain;
     
     float const flap_in = plane.get_flap_demand();
     /*
       flap only works in manual mode else it is 0
       In crow mode, bottom half of throttle controls amount of crow
       In manual mode, if crow is on then flap goes from full down flap at minimum crow ( around half throttle)
       as crow increases(thorttle sticl towards min throttle) the flap effect reduces . 
       This gives a smooth flap movement from min to max crow in crow mode
       Crow flaps are only effective when throttle is in bottom half.
       With throttle in upper half, throttle increses as throttle stick increases
       In top half of throttle flap stays on full down flap, to give a good climb out if landing is aborted
      */
     float const flap0 = 
         (manual_mode == true)
         ?((crow_mode == false) 
            ? -flap_in
            : (crow_active ? 1.f - crow : 1.f))
         : 0.f;
     bool const  up_flap = flap0 < 0.f;
     float const flap = flap0 * (up_flap?up_flap_gain:down_flap_gain);
     
     output[port_aileron_channel] = (port_aileron - flap - ail_crow) * port_aileron_dir;
     output[stbd_aileron_channel] = (stbd_aileron - flap - ail_crow) * stbd_aileron_dir;

     float const elev_to_flap = (up_flap?up_flap_gain:down_flap_gain) * elev_flap_gain * flap;
     float const roll_to_rudder = roll * rudder_roll_gain;
     float const elev_to_crow = elev_to_crow_gain * crow;
     float const pitch = plane.get_pitch_demand();
     float const yaw = plane.get_yaw_demand();
  
     output[port_v_tail_channel] = (pitch * 0.5f + yaw * 0.5f + roll_to_rudder + elev_to_flap + elev_to_crow ) * port_v_tail_dir;
    // output[stbd_v_tail] = -pitch * 0.5f + yaw * 0.5f + roll_to_rudder - elev_to_flap - elev_to_crow;
     output[stbd_v_tail_channel] = (pitch * 0.5f - yaw * 0.5f - roll_to_rudder + elev_to_flap + elev_to_crow ) * stbd_v_tail_dir;

     for ( uint8_t i = 0; i < num_outputs; ++i){
         output_action(i);
     }
   }
}
