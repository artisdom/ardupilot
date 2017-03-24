// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#if 0
#include "Plane.h"

/*
 *  failsafe support
 *  Andrew Tridgell, December 2011
 */

/*
 *  our failsafe strategy is to detect main loop lockup and switch to
 *  passing inputs straight from the RC inputs to RC outputs.
 */


namespace {
   typedef quan::time_<int32_t>::us usec;

   template <typename Axis>
   void set_axis_output_from_rc_in(JoystickInput<Axis> const & in, FltCtrlOutput<Axis> & out)
   {
      usec rcin = usec{hal.rcin->read(joystick_roll.get_rcin_index())} - in.get_trim();

      float val = quan::constrain(rcin.numeric_value() / static_cast<float>(in.get_range().numeric_value()),-1,f,1.f);
      
      out.set(val);
   }
}

/*
   in fact this is not called atm
   Could do in loop
   return a bool if in failsafe
   Or could be on a watchdog timer
   If watchdog is not reset then call this or some reimpl of it
   at the update rate
   Or do separate thread
   Howveever if main loop not running is there any point?
*/

  
void Plane::failsafe_check(void)
{
    static uint16_t last_mainLoop_count;
    static uint32_t last_timestamp;
    static bool in_failsafe;
    uint32_t tnow = micros();

    if (mainLoop_count != last_mainLoop_count) {
        // the main loop is running, all is OK
        last_mainLoop_count = mainLoop_count;
        last_timestamp = tnow;
        in_failsafe = false;
        return;
    }

    if (tnow - last_timestamp > 200000) {
        // we have gone at least 0.2 seconds since the main loop
        // ran. That means we're in trouble, or perhaps are in
        // an initialisation routine or log erase. Start passing RC
        // inputs through to outputs
        in_failsafe = true;
    }

    if (in_failsafe && tnow - last_timestamp > 20000) {
        last_timestamp = tnow;


//        if (hal.rcin->num_channels() < 5) {
//            // we don't have any RC input to pass through
//            return;
//        }

        // pass RC inputs to outputs every 20ms
        hal.rcin->clear_overrides();


/*
 read comes from rcin
  so radio_out is equivalent to rcin
*/
        //  channel_roll.set_output_usec(channel_roll.read_joystick_usec());
        output_roll.set();
        //  channel_pitch.set_output_usec(channel_pitch.read_joystick_usec());
        output_pitch.set(usec{hal.rcin->read(joystick_pitch.get_rcin_index())});
        if (hal.util->get_soft_armed()) {
           // channel_thrust.set_output_usec(channel_thrust.read_joystick_usec());
         //    hal.console->printf("fail_safe check  output_thrust -> -1.f\n");
            output_thrust.set(usec{hal.rcin->read(joystick_thrust.get_rcin_index())});
        }
        // channel_yaw.set_output_usec(channel_yaw.read_joystick_usec());
        output_yaw.set(usec{hal.rcin->read(joystick_yaw.get_rcin_index())});
        mix();

    }
}
#endif
