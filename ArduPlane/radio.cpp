// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Plane.h"
#if CONFIG_HAL_BOARD == HAL_BOARD_QUAN
#include <AP_OSD/AP_OSD_enqueue.h>
#endif

//Function that will read the radio data, limit servos and trigger a failsafe
// ----------------------------------------------------------------------------

/*
  allow for runtime change of control channel ordering
 */
void Plane::set_control_channels(void)
{

    channel_roll     = RC_Channel::rc_channel(rcmap.roll()-1);
    channel_pitch    = RC_Channel::rc_channel(rcmap.pitch()-1);
    channel_throttle = RC_Channel::rc_channel(rcmap.throttle()-1);
    channel_rudder   = RC_Channel::rc_channel(rcmap.yaw()-1);

    // set rc channel ranges
    channel_roll->set_angle(SERVO_MAX);
    channel_pitch->set_angle(SERVO_MAX);
    channel_rudder->set_angle(SERVO_MAX);
    channel_throttle->set_range(0, 100);

    if (!arming.is_armed() && arming.arming_required() == AP_Arming::YES_MIN_PWM) {
        hal.rcout->set_safety_pwm(1UL<<(rcmap.throttle()-1), throttle_min());
    }

    // setup correct scaling for ESCs like the UAVCAN PX4ESC which
    // take a proportion of speed
    hal.rcout->set_esc_scaling(channel_throttle->get_radio_min(), channel_throttle->get_radio_max());
}

/*
  initialise RC input channels
 */
void Plane::init_rc_in()
{
    // set rc dead zones
    channel_roll->set_default_dead_zone(30);
    channel_pitch->set_default_dead_zone(30);
    channel_rudder->set_default_dead_zone(30);
    channel_throttle->set_default_dead_zone(30);
}

/*
  initialise RC output channels
 */
void Plane::init_rc_out()
{
    channel_roll->enable_out();
    channel_pitch->enable_out();

    /*
      change throttle trim to minimum throttle. This prevents a
      configuration error where the user sets CH3_TRIM incorrectly and
      the motor may start on power up
     */
    channel_throttle->set_radio_trim(throttle_min());
    
    if (arming.arming_required() != AP_Arming::YES_ZERO_PWM) {
        channel_throttle->enable_out();
    }
    channel_rudder->enable_out();

    // Initialization of servo outputs
    RC_Channel::output_trim_all();

    // setup PWM values to send if the FMU firmware dies
    RC_Channel::setup_failsafe_trim_all();  

    // setup PX4 to output the min throttle when safety off if arming
    // is setup for min on disarm
    if (arming.arming_required() == AP_Arming::YES_MIN_PWM) {
        hal.rcout->set_safety_pwm(1UL<<(rcmap.throttle()-1), throttle_min());
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
    if (channel_throttle->get_control_in() > 0) {
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
		if (channel_rudder->get_control_in() > 4000) {
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
		if (channel_rudder->get_control_in() < -4000) {
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
        control_failsafe(channel_throttle->get_radio_in());
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
   if ( num_channels > 0){;
      for(uint8_t i = 0; i < 6; ++i){
         chan_ar[i] = (i < num_channels)? hal.rcin->read(i) :0;
      }
      AP_OSD::enqueue::rc_inputs_0_to_5(chan_ar,6);
   }
      
#endif

    failsafe.last_valid_rc_ms = millis();
    uint16_t const pwm_roll = channel_roll->read();
    uint16_t const pwm_pitch = channel_pitch->read();

    RC_Channel::set_pwm_all();
    
    if (control_mode == TRAINING) {
        // in training mode we don't want to use a deadzone, as we
        // want manual pass through when not exceeding attitude limits
        channel_roll->set_pwm_no_deadzone(pwm_roll);
        channel_pitch->set_pwm_no_deadzone(pwm_pitch);
        channel_throttle->set_pwm_no_deadzone(channel_throttle->read());
        channel_rudder->set_pwm_no_deadzone(channel_rudder->read());
    } else {
        channel_roll->set_pwm(pwm_roll);
        channel_pitch->set_pwm(pwm_pitch);
    }

    control_failsafe(channel_throttle->get_radio_in());

    channel_throttle->set_servo_out(channel_throttle->get_control_in());

    if (g.throttle_nudge && channel_throttle->get_servo_out() > 50) {
        float nudge = (channel_throttle->get_servo_out() - 50) * 0.02f;
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
    rudder_input = channel_rudder->get_control_in();
    
}

void Plane::control_failsafe(uint16_t pwm)
{
    if (millis() - failsafe.last_valid_rc_ms > 1000 || rc_failsafe_active()) {
        // we do not have valid RC input. Set all primary channel
        // control inputs to the trim value and throttle to min
        channel_roll->set_radio_in(channel_roll->get_radio_trim());
        channel_pitch->set_radio_in(channel_pitch->get_radio_trim());
        channel_rudder->set_radio_in(channel_rudder->get_radio_trim());

        // note that we don't set channel_throttle->radio_in to radio_trim,
        // as that would cause throttle failsafe to not activate
        channel_roll->set_control_in(0);
        channel_pitch->set_control_in(0);
        channel_rudder->set_control_in(0);
        channel_throttle->set_control_in(0);
    }

    if(g.throttle_fs_enabled == 0)
        return;

    if (g.throttle_fs_enabled) {
        if (rc_failsafe_active()) {
            // we detect a failsafe from radio
            // throttle has dropped below the mark
            failsafe.ch3_counter++;
            if (failsafe.ch3_counter == 10) {
                gcs_send_text_fmt(MAV_SEVERITY_WARNING, "MSG FS ON %u", (unsigned)pwm);
                failsafe.ch3_failsafe = true;
                AP_Notify::flags.failsafe_radio = true;
            }
            if (failsafe.ch3_counter > 10) {
                failsafe.ch3_counter = 10;
            }

        }else if(failsafe.ch3_counter > 0) {
            // we are no longer in failsafe condition
            // but we need to recover quickly
            failsafe.ch3_counter--;
            if (failsafe.ch3_counter > 3) {
                failsafe.ch3_counter = 3;
            }
            if (failsafe.ch3_counter == 1) {
                gcs_send_text_fmt(MAV_SEVERITY_WARNING, "MSG FS OFF %u", (unsigned)pwm);
            } else if(failsafe.ch3_counter == 0) {
                failsafe.ch3_failsafe = false;
                AP_Notify::flags.failsafe_radio = false;
            }
        }
    }
}

void Plane::trim_control_surfaces()
{
   read_radio();
   int16_t trim_roll_range = (channel_roll->get_radio_max() - channel_roll->get_radio_min())/5;
   int16_t trim_pitch_range = (channel_pitch->get_radio_max() - channel_pitch->get_radio_min())/5;
   if (channel_roll->get_radio_in() < channel_roll->get_radio_min()+trim_roll_range ||
      channel_roll->get_radio_in() > channel_roll->get_radio_max()-trim_roll_range ||
      channel_pitch->get_radio_in() < channel_pitch->get_radio_min()+trim_pitch_range ||
      channel_pitch->get_radio_in() > channel_pitch->get_radio_max()-trim_pitch_range) {
      // don't trim for extreme values - if we attempt to trim so
      // there is less than 20 percent range left then assume the
      // sticks are not properly centered. This also prevents
      // problems with starting APM with the TX off
      return;
   }

   if (channel_roll->get_radio_in() != 0) {
      channel_roll->set_radio_trim(channel_roll->get_radio_in());
   }
   if (channel_pitch->get_radio_in() != 0) {
      channel_pitch->set_radio_trim(channel_pitch->get_radio_in());
   }

   if (channel_rudder->get_radio_in() != 0) {
      channel_rudder->set_radio_trim(channel_rudder->get_radio_in());
   }

   channel_roll->save_eeprom();
   channel_pitch->save_eeprom();
   channel_rudder->save_eeprom();
}

void Plane::trim_radio()
{
    for (uint8_t y = 0; y < 30; y++) {
        read_radio();
    }

    trim_control_surfaces();
}

/*
  return true if throttle level is below throttle failsafe threshold
  or RC input is invalid
 */
bool Plane::rc_failsafe_active(void)
{
    if (!g.throttle_fs_enabled) {
        return false;
    }
    if (millis() - failsafe.last_valid_rc_ms > 1000) {
        // we haven't had a valid RC frame for 1 seconds
        return true;
    }
    if (channel_throttle->get_reverse()) {
        return channel_throttle->get_radio_in() >= g.throttle_fs_value;
    }
    return channel_throttle->get_radio_in() <= g.throttle_fs_value;
}
