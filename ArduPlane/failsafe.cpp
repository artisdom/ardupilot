// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Plane.h"

/*
 *  failsafe support
 *  Andrew Tridgell, December 2011
 */

/*
 *  our failsafe strategy is to detect main loop lockup and switch to
 *  passing inputs straight from the RC inputs to RC outputs.
 */

/*
 *  this failsafe_check function is called from the core timer interrupt
 *  at 1kHz.
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

#if OBC_FAILSAFE == ENABLED
        if (in_calibration) {
            // tell the failsafe system that we are calibrating
            // sensors, so don't trigger failsafe
            obc.heartbeat();
        }
#endif

        if (hal.rcin->num_channels() < 5) {
            // we don't have any RC input to pass through
            return;
        }

        // pass RC inputs to outputs every 20ms
        hal.rcin->clear_overrides();


/*
 read comes from rcin
  so radio_out is equivalent to rcin

*/
        channel_roll.set_output_usec(channel_roll.read_joystick_usec());
        channel_pitch.set_output_usec(channel_pitch.read_joystick_usec());
        if (hal.util->get_soft_armed()) {
            channel_thrust.set_output_usec(channel_thrust.read_joystick_usec());
        }
        channel_yaw.set_output_usec(channel_yaw.read_joystick_usec());

#if OBC_FAILSAFE == ENABLED
        // this is to allow the failsafe module to deliberately crash 
        // the plane. Only used in extreme circumstances to meet the
        // OBC rules
        obc.check_crash_plane();
#endif

        if (!demoing_servos) {
            channel_roll.write_output_usec();
            channel_pitch.write_output_usec();
        }
        channel_thrust.write_output_usec();
        channel_yaw.write_output_usec();
    }
}
