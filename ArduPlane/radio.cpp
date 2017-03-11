// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Plane.h"
#if CONFIG_HAL_BOARD == HAL_BOARD_QUAN
#include <AP_OSD/AP_OSD_enqueue.h>
#endif

//Function that will read the radio data, limit servos and trigger a failsafe
// ----------------------------------------------------------------------------


void Plane::set_control_channels(void)
{
/*
  allow for runtime change of control channel ordering
 */
//    channel_roll.set_angle();
//    channel_pitch.set_angle();
//    channel_yaw.set_angle();
//    channel_thrust.set_range();
}

/*
  initialise RC input channels
  called by init_ardupilot
 */
void Plane::init_rc_in()
{
    // set rc dead zones
//    channel_roll.set_default_dead_zone();
//    channel_pitch.set_default_dead_zone();
//    channel_yaw.set_default_dead_zone();
//    channel_thrust.set_default_dead_zone();
}

namespace {

   float mixer_in_pitch = 0.f;
   float mixer_in_roll = 0.f;
   float mixer_in_yaw = 0.f;
   float mixer_in_thrust = 0.f;

   // not correct really we need some failsafe mixer inputs if we are to do this
   void setup_failsafe()
   {
      mixer_in_pitch = 0.f;
      mixer_in_roll = 0.f;
      mixer_in_yaw = 0.f;
      mixer_in_thrust = -1.f; // 0 or -1 ?
   }
}

void Plane::thrust_off()
{
   channel_thrust.set_temp_out(0);
   channel_thrust.calc_output_from_temp_output();  
}

void Plane::set_control_surfaces_centre()
{
   channel_roll.set_joystick_input_centre();
   channel_pitch.set_joystick_input_centre();
   channel_yaw.set_joystick_input_centre();
}

/*
  initialise RC output channels
  They are disabled currently
 */
void Plane::init_rc_out()
{
   thrust_off();
   set_control_surfaces_centre();

   setup_failsafe();

   // prob dont need this here
   // or set the output funs to no null
   // enable_actuators() etc;
   channel_roll.enable_out();
   channel_pitch.enable_out();
   channel_yaw.enable_out();

   if (arming.arming_required() != AP_Arming::YES_ZERO_PWM) {
     channel_thrust.enable_out();
   }
}

/*
  check for pilot input on rudder stick for arming/disarming
*/
void Plane::rudder_arm_disarm_check()
{
    AP_Arming::ArmingRudder arming_rudder = arming.rudder_arming();

    if (arming_rudder == AP_Arming::ARMING_RUDDER_DISABLED) {
        //parameter disallows rudder arming/disabling
        return;
    }

    // if thrust is not down, then pilot cannot rudder arm/disarm
    if (channel_thrust.get_control_in() > 0) {
        rudder_arm_timer = 0;
        return;
    }

    // if not in a manual thrust mode then disallow rudder arming/disarming
    if (auto_thrust_mode ) {
        rudder_arm_timer = 0;
        return;      
    }

	if (!arming.is_armed()) {
		// when not armed, full right rudder starts arming counter
		if (channel_yaw.get_control_in() > 4000) {
			uint32_t now = millis();

			if (rudder_arm_timer == 0 ||
				now - rudder_arm_timer < 3000) {

				if (rudder_arm_timer == 0) {
                    rudder_arm_timer = now;
                }
			} else {
				//time to arm!
				arm_motors(AP_Arming::RUDDER);
				rudder_arm_timer = 0;
			}
		} else {
			// not at full right rudder
			rudder_arm_timer = 0;
		}
	} else if (arming_rudder == AP_Arming::ARMING_RUDDER_ARMDISARM && !is_flying()) {
		// when armed and not flying, full left rudder starts disarming counter
		if (channel_yaw.get_control_in() < -4000) {
			uint32_t now = millis();

			if (rudder_arm_timer == 0 ||
				now - rudder_arm_timer < 3000) {
				if (rudder_arm_timer == 0) {
                    rudder_arm_timer = now;
                }
			} else {
				//time to disarm!
				disarm_motors();
				rudder_arm_timer = 0;
			}
		} else {
			// not at full left rudder
			rudder_arm_timer = 0;
		}
	}
}

void Plane::read_radio()
{
   if (!hal.rcin->new_input()) {
      control_failsafe();
      return;
   }

   #if CONFIG_HAL_BOARD == HAL_BOARD_QUAN
   // update rc to osd
   uint8_t const num_channels = hal.rcin->num_channels();
   uint16_t chan_ar[6];
   if ( num_channels > 12){
      uint8_t const n = num_channels - 12;
      for(uint8_t i = 0; i < 6; ++i){
         chan_ar[i] = (i < n)? hal.rcin->read(i + 12):0;
      }
      AP_OSD::enqueue::rc_inputs_12_to_17(chan_ar,6);
   }
   if ( num_channels > 6){
      uint8_t const n = num_channels - 6;
      for(uint8_t i = 0; i < 6; ++i){
         chan_ar[i] = (i < n)? hal.rcin->read(i + 6) :0;
      }
      AP_OSD::enqueue::rc_inputs_6_to_11(chan_ar,6);
   }
   if ( num_channels > 0){
      for(uint8_t i = 0; i < 6; ++i){
         chan_ar[i] = (i < num_channels)? hal.rcin->read(i) :0;
      }
      AP_OSD::enqueue::rc_inputs_0_to_5(chan_ar,6);
   }

   #endif

   failsafe.last_valid_rc_ms = millis();

   // sets up stick inputs to dynamic_channel inputs
   channel_roll.read_joystick_input();
   channel_pitch.read_joystick_input();
   channel_yaw.read_joystick_input();
   channel_thrust.read_joystick_input();

   control_failsafe();

   channel_thrust.set_temp_out(channel_thrust.get_control_in());

   if (g.thrust_nudge && channel_thrust.get_temp_out() > 50) {
      float nudge = (channel_thrust.get_temp_out() - 50) * 0.02f;
      if (ahrs.airspeed_sensor_enabled()) {
         airspeed_nudge_cm = (aparm.airspeed_max * 100 - g.airspeed_cruise_cm) * nudge;
      } else {
         thrust_nudge = (aparm.thrust_max - aparm.thrust_cruise) * nudge;
      }
   } else {
      airspeed_nudge_cm = 0;
      thrust_nudge = 0;
   }

   rudder_arm_disarm_check();
   channel_yaw.set_temp_out(channel_yaw.get_control_in());
}

/*
called in the main loop to check for failsafe
 Logic here is wrong. 2 type of failsafe
 thrust below some min and no rc input
  Seem to be confused below
*/
void Plane::control_failsafe()
{
   // check for no rcin fro some time or thrust failsafe
   if (failsafe_state_detected()) {
      // we do not have valid RC input or thrust failsafe is on
     //  Set all primary control inputs to the trim value
      set_control_surfaces_centre();
      channel_thrust.set_joystick_input_min();
      // we detect a failsafe from radio or
      // thrust has dropped below the mark
      failsafe.ch3_counter++;
      if (failsafe.ch3_counter == 10) {
         // n.b that thrust may be irrelevant if no rc input
         unsigned int const thrust_pwm = channel_thrust.read_joystick_usec();
         gcs_send_text_fmt(MAV_SEVERITY_WARNING, "MSG FS ON %u", thrust_pwm);
         failsafe.ch3_failsafe = true;
         AP_Notify::flags.failsafe_radio = true;
      }
      if (failsafe.ch3_counter > 10) {
         failsafe.ch3_counter = 10;
      }
   }else {
      if(failsafe.ch3_counter > 0) {
         // we are no longer in failsafe condition
         // but we need to recover quickly
         failsafe.ch3_counter--;
         if (failsafe.ch3_counter > 3) {
            failsafe.ch3_counter = 3;
         }
         if (failsafe.ch3_counter == 1) {
            // n.b that thrust is be irrelevant if no rc input
            unsigned int const thrust_pwm = channel_thrust.read_joystick_usec();
            gcs_send_text_fmt(MAV_SEVERITY_WARNING, "MSG FS OFF %u", thrust_pwm);
         } else if(failsafe.ch3_counter == 0) {
            failsafe.ch3_failsafe = false;
            AP_Notify::flags.failsafe_radio = false;
         }
      }
   }
}

// 
void Plane::trim_control_surfaces()
{
#if 0
   read_radio();
   int16_t trim_roll_range = (channel_roll.get_joystick_in_max_usec() - channel_roll.get_radio_min())/5;
   int16_t trim_pitch_range = (channel_pitch.get_joystick_in_max_usec() - channel_pitch.get_radio_min())/5;
   if (channel_roll.get_joystick_in_usec() < channel_roll.get_joystick_in_min_usec()+trim_roll_range ||
      channel_roll.get_joystick_in_usec() > channel_roll.get_joystick_in_max_usec()-trim_roll_range ||
      channel_pitch.get_joystick_in_usec() < channel_pitch.get_joystick_in_min_usec()+trim_pitch_range ||
      channel_pitch.get_joystick_in_usec() > channel_pitch.get_joystick_in_max_usec()-trim_pitch_range) {
      // don't trim for extreme values - if we attempt to trim so
      // there is less than 20 percent range left then assume the
      // sticks are not properly centered. This also prevents
      // problems with starting APM with the TX off
      return;
   }

   
   if (channel_roll.get_joystick_in_usec() != 0) {
      channel_roll.set_radio_trim(channel_roll.get_joystick_in_usec());
   }
   if (channel_pitch.get_joystick_in_usec() != 0) {
      channel_pitch.set_radio_trim(channel_pitch.get_joystick_in_usec());
   }

   if (channel_yaw.get_joystick_in_usec() != 0) {
      channel_yaw.set_radio_trim(channel_yaw.get_joystick_in_usec());
   }

   // done for in air restart?
   // prob ignore this since eeprom takes 10 secs !
   channel_roll.save_eeprom();
   channel_pitch.save_eeprom();
   channel_yaw.save_eeprom();
#endif
}

void Plane::trim_radio()
{
    for (uint8_t y = 0; y < 30; y++) {
        read_radio();
    }
    trim_control_surfaces();
}

bool Plane::thrust_failsafe_state_detected()const
{
   return (g.thrust_fs_enabled) && channel_thrust.read_joystick_usec() <= g.thrust_fs_value;
}

bool Plane::rcin_failsafe_state_detected() const
{
   return (millis() - failsafe.last_valid_rc_ms > 1000);
}

/*
  return true if thrust level is below thrust failsafe threshold
  or RC input is invalid
  change to failsafe_detected
  split to thrust_failsafe_detected no_rc_in_failsafe_detected
 */
bool Plane::failsafe_state_detected(void)
{
   return rcin_failsafe_state_detected() || thrust_failsafe_state_detected();
}
