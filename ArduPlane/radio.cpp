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
//    channel_rudder.set_angle();
//    channel_throttle.set_range();
}

/*
  initialise RC input channels
  called by init_ardupilot
 */
void Plane::init_rc_in()
{

    channel_roll->set_default_dead_zone(30);
    channel_pitch->set_default_dead_zone(30);
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    channel_pitch->set_reverse(true);
#endif
    channel_rudder->set_default_dead_zone(30);
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    channel_rudder->set_reverse(true);
#endif
    channel_throttle->set_default_dead_zone(30);

}

namespace {

   float mixer_in_pitch = 0.f;
   float mixer_in_roll = 0.f;
   float mixer_in_yaw = 0.f;
   float mixer_in_throttle = 0.f;

   // not correct really we need some failsafe mixer inputs if we are to do this
   void setup_failsafe()
   {
      mixer_in_pitch = 0.f;
      mixer_in_roll = 0.f;
      mixer_in_yaw = 0.f;
      mixer_in_throttle = -1.f; // 0 or -1 ?
   }
}

void Plane::throttle_off()
{
   channel_throttle.set_servo_out(0);
   channel_throttle.calc_pwm();  
}

void Plane::set_control_surfaces_centre()
{
   // sets radio_in and control_in
   channel_roll.set_pwm(channel_roll.get_radio_trim());
   channel_pitch.set_pwm(channel_pitch.get_radio_trim());
   channel_rudder.set_pwm(channel_rudder.get_radio_trim());
}

/*
  initialise RC output channels
  They are disabled currently
 */
void Plane::init_rc_out()
{
   throttle_off();
   set_control_surfaces_centre();

   setup_failsafe();

   // prob dont need this here
   // or set the output funs to no null
   // enable_actuators() etc;
   channel_roll.enable_out();
   channel_pitch.enable_out();
   channel_rudder.enable_out();

   if (arming.arming_required() != AP_Arming::YES_ZERO_PWM) {
     channel_throttle.enable_out();
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

    // if throttle is not down, then pilot cannot rudder arm/disarm
    if (channel_throttle.get_control_in() > 0) {
        rudder_arm_timer = 0;
        return;
    }

    // if not in a manual throttle mode then disallow rudder arming/disarming
    if (auto_throttle_mode ) {
        rudder_arm_timer = 0;
        return;      
    }

	if (!arming.is_armed()) {
		// when not armed, full right rudder starts arming counter
		if (channel_rudder.get_control_in() > 4000) {
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
		if (channel_rudder.get_control_in() < -4000) {
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

   channel_roll.set_pwm(channel_roll.read());
   channel_pitch.set_pwm(channel_pitch.read());
   channel_throttle.set_pwm(channel_throttle.read());
   channel_rudder.set_pwm(channel_rudder.read());

   control_failsafe();

   channel_throttle.set_servo_out(channel_throttle.get_control_in());

   if (g.throttle_nudge && channel_throttle.get_servo_out() > 50) {
      float nudge = (channel_throttle.get_servo_out() - 50) * 0.02f;
      if (ahrs.airspeed_sensor_enabled()) {
         airspeed_nudge_cm = (aparm.airspeed_max * 100 - g.airspeed_cruise_cm) * nudge;
      } else {
         throttle_nudge = (aparm.throttle_max - aparm.throttle_cruise) * nudge;
      }
   } else {
      airspeed_nudge_cm = 0;
      throttle_nudge = 0;
   }

   rudder_arm_disarm_check();
   channel_rudder.set_servo_out(channel_rudder.get_control_in());
}

/*
called in the main loop to check for failsafe
 Logic here is wrong. 2 type of failsafe
 throttle below some min and no rc input
  Seem to be confused below
*/
void Plane::control_failsafe()
{
   unsigned int const throttle_pwm = channel_throttle.get_radio_in();
   // check for no rcin fro some time or throttle failsafe
   if (failsafe_state_detected()) {
      // we do not have valid RC input or throttle failsafe is on
     //  Set all primary control inputs to the trim value
      channel_roll.set_pwm(channel_roll.get_radio_trim());
      channel_pitch.set_pwm(channel_pitch.get_radio_trim());
      channel_rudder.set_pwm(channel_rudder.get_radio_trim());

      channel_throttle.failsafe_set_control_in(0);
      // we detect a failsafe from radio or
      // throttle has dropped below the mark
      failsafe.ch3_counter++;
      if (failsafe.ch3_counter == 10) {
         // n.b that throttle may be irrelevant if no rc input
         gcs_send_text_fmt(MAV_SEVERITY_WARNING, "MSG FS ON %u", throttle_pwm);
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
            // n.b that throttle is be irrelevant if no rc input
            gcs_send_text_fmt(MAV_SEVERITY_WARNING, "MSG FS OFF %u", throttle_pwm);
         } else if(failsafe.ch3_counter == 0) {
            failsafe.ch3_failsafe = false;
            AP_Notify::flags.failsafe_radio = false;
         }
      }
   }
}

void Plane::trim_control_surfaces()
{
#if 0
   read_radio();
   int16_t trim_roll_range = (channel_roll.get_radio_max() - channel_roll.get_radio_min())/5;
   int16_t trim_pitch_range = (channel_pitch.get_radio_max() - channel_pitch.get_radio_min())/5;
   if (channel_roll.get_radio_in() < channel_roll.get_radio_min()+trim_roll_range ||
      channel_roll.get_radio_in() > channel_roll.get_radio_max()-trim_roll_range ||
      channel_pitch.get_radio_in() < channel_pitch.get_radio_min()+trim_pitch_range ||
      channel_pitch.get_radio_in() > channel_pitch.get_radio_max()-trim_pitch_range) {
      // don't trim for extreme values - if we attempt to trim so
      // there is less than 20 percent range left then assume the
      // sticks are not properly centered. This also prevents
      // problems with starting APM with the TX off
      return;
   }

   
   if (channel_roll.get_radio_in() != 0) {
      channel_roll.set_radio_trim(channel_roll.get_radio_in());
   }
   if (channel_pitch.get_radio_in() != 0) {
      channel_pitch.set_radio_trim(channel_pitch.get_radio_in());
   }

   if (channel_rudder.get_radio_in() != 0) {
      channel_rudder.set_radio_trim(channel_rudder.get_radio_in());
   }

   // done for in air restart?
   // prob ignore this since eeprom takes 10 secs !
   channel_roll.save_eeprom();
   channel_pitch.save_eeprom();
   channel_rudder.save_eeprom();
#endif
}

void Plane::trim_radio()
{
    for (uint8_t y = 0; y < 30; y++) {
        read_radio();
    }
    trim_control_surfaces();
}

bool Plane::throttle_failsafe_state_detected()const
{
   if (g.throttle_fs_enabled) {
      int16_t const throttle_joystick_input = channel_throttle.read();
      if (channel_throttle.get_reverse()) {
         return  throttle_joystick_input >= g.throttle_fs_value;
      }else{
         return throttle_joystick_input <= g.throttle_fs_value;
      }
   }else{
      return false;
   }
}

bool Plane::rcin_failsafe_state_detected() const
{
   return (millis() - failsafe.last_valid_rc_ms > 1000);
}

/*
  return true if throttle level is below throttle failsafe threshold
  or RC input is invalid
  change to failsafe_detected
  split to throttle_failsafe_detected no_rc_in_failsafe_detected
 */
bool Plane::failsafe_state_detected(void)
{
   return rcin_failsafe_state_detected() || throttle_failsafe_state_detected();
}
