// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Plane.h"


#if CONFIG_HAL_BOARD == HAL_BOARD_QUAN
#include <AP_OSD/AP_OSD_enqueue.h>
#endif
#include <quan/length.hpp>

//Function that will read the radio data, limit servos and trigger a failsafe
// ----------------------------------------------------------------------------

void Plane::set_control_channels(void)
{
/*
  allow for runtime change of control channel ordering
   should do if ( get_joystick_channel !=
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
     joystick_yaw.set_reversed(true);
     joystick_roll.set_reversed(false);
     joystick_pitch.set_reversed(false);
}

namespace {

   QUAN_QUANTITY_LITERAL(force, N)
   QUAN_QUANTITY_LITERAL(time, us)
   QUAN_ANGLE_LITERAL(cdeg)

   typedef quan::angle_<int32_t>::cdeg cdeg;
   typedef quan::time_<int32_t>::us usec;

//   float mixer_in_pitch = 0.f;
//   float mixer_in_roll = 0.f;
//   float mixer_in_yaw = 0.f;
//   float mixer_in_thrust = 0.f;

   // not correct really we need some failsafe mixer inputs if we are to do this
//   void setup_failsafe()
//   {
////      mixer_in_pitch = 0.f;
////      mixer_in_roll = 0.f;
////      mixer_in_yaw = 0.f;
////      mixer_in_thrust = -1.f; // 0 or -1 ?
//   }
}

void Plane::thrust_off()
{
  // channel_thrust.set_temp_out(0);
   autopilot_thrust.set(0_N);
   hal.console->printf("thrust off : output_thrust ->  0N (autopilot thrust)\n");
  // channel_thrust.calc_output_from_temp_output();  
   output_thrust.set(autopilot_thrust);
}

void Plane::set_control_surfaces_centre()
{
  // channel_roll.set_joystick_input_centre();
    joystick_roll.set_centre();
   //channel_pitch.set_joystick_input_centre();
    joystick_pitch.set_centre();
   //channel_yaw.set_joystick_input_centre();
    joystick_yaw.set_centre();
}

/*
  initialise RC output channels
  They are disabled currently
 */
void Plane::init_rc_out()
{
   thrust_off();
   set_control_surfaces_centre();

   for ( uint8_t i = 0; i < 7; ++i){
      hal.rcout->enable_ch(i);
   }
   //setup_failsafe();
   if (arming.arming_required() != AP_Arming::YES_ZERO_PWM) {
     output_thrust.enable();
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
  //  if (channel_thrust.get_control_in() > 0) {
    if (joystick_thrust.as_force() > 0_N){
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
	//	if (channel_yaw.get_control_in() > 4000) {
      if ( joystick_yaw.as_angle() > 4000_cdeg){
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
		//if (channel_yaw.get_control_in() < -4000) {
      if ( joystick_yaw.as_angle() < -4000_cdeg){
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
  // channel_roll.read_joystick_input();
   joystick_roll.update();

  // channel_pitch.read_joystick_input();
   joystick_pitch.update();

  // channel_yaw.read_joystick_input();
   joystick_yaw.update();

  // channel_thrust.read_joystick_input();
   joystick_thrust.update();


   control_failsafe();

  // channel_thrust.set_temp_out(channel_thrust.get_control_in());
   autopilot_thrust.set(joystick_thrust);

  // if (g.thrust_nudge && channel_thrust.get_temp_out() > 50) {
   if (g.thrust_nudge && (autopilot_thrust.get() > 50_N)) {
      float nudge = (autopilot_thrust.get() - 50_N).numeric_value() * 0.02f;
      if (ahrs.airspeed_sensor_enabled()) {
         airspeed_nudge_cm = (aparm.airspeed_max * 100 - g.airspeed_cruise_cm) * nudge;
      } else {
         thrust_nudge = (aparm.thrust_max - aparm.thrust_cruise) * nudge;
      }
   }else{
      airspeed_nudge_cm = 0;
      thrust_nudge = 0;
   }
   rudder_arm_disarm_check();
  // channel_yaw.set_temp_out(channel_yaw.get_control_in());
   autopilot_yaw.set(joystick_yaw);
}

/*
called in the main loop to check for failsafe
 Logic here is wrong. 2 type of failsafe
 thrust below some min and no rc input
  Seem to be confused below
*/
void Plane::control_failsafe()
{
   // check for no rcin for some time or thrust failsafe
   if (failsafe_state_detected()) {
      // we do not have valid RC input or thrust failsafe is on
     //  Set all primary control inputs to the trim value
      set_control_surfaces_centre();

     // channel_thrust.set_joystick_input_min();
      hal.console->printf("control_failsafe : set thrust to min\n");
      joystick_thrust.set_min();

      // we detect a failsafe from radio or
      // thrust has dropped below the mark
      failsafe.ch3_counter++;
      if (failsafe.ch3_counter == 10) {
         // n.b that thrust may be irrelevant if no rc input
       //  unsigned int const thrust_pwm = channel_thrust.read_joystick_usec();
         unsigned int const thrust_pwm = hal.rcin->read(joystick_thrust.get_rcin_index());  
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
           // unsigned int const thrust_pwm = channel_thrust.read_joystick_usec();
            unsigned int const thrust_pwm = hal.rcin->read(joystick_thrust.get_rcin_index());
            gcs_send_text_fmt(MAV_SEVERITY_WARNING, "MSG FS OFF %u", thrust_pwm);
         } else if(failsafe.ch3_counter == 0) {
            failsafe.ch3_failsafe = false;
            AP_Notify::flags.failsafe_radio = false;
         }
      }
   }
}

/*
Called in startup ground only. We could do in air trim, but only if specifically commanded by user

  On entry. Assume that the joystick inputs are at neutral trim for flying
   On first read check that the pitch roll and yaw inputs are within a limit
   Say 10 % of centre
    Then read the inputs for some time
   Check they arent moving after the first read ( so they are the same same within some range)
   If they have moved significantly then fail. The user is probably unaware that trimming is in progress
   Prob need a message
*/

bool Plane::setup_joystick_trims()
{
   read_radio();

   usec const init_trim_pitch = joystick_pitch.as_usec();
   usec const init_trim_roll  = joystick_roll.as_usec();
   usec const init_trim_yaw   = joystick_yaw.as_usec();

   constexpr usec max_trim_offset = 50_us;

   if ( abs(init_trim_pitch) >= max_trim_offset ){
      hal.console->printf("Trim pitch out of range\n");
      return false;
   }

   if ( abs( init_trim_roll) >= max_trim_offset ){
      hal.console->printf("Trim roll out of range\n");
      return false;
   }

   if ( abs( init_trim_yaw) >= max_trim_offset ){
      hal.console->printf("Trim yaw out of range\n");
      return false;
   }

   usec constexpr max_error = 10_us;  // max error +- 1% of range approx

   usec pitch_sum = init_trim_pitch;
   usec roll_sum = init_trim_roll;
   usec yaw_sum = init_trim_yaw;

   int count = 1;
   auto now = millis();
   while (( millis() - now ) < 1000){

       hal.scheduler->delay(20); 
       read_radio();

       usec const pitch = joystick_pitch.as_usec();
       usec const yaw   = joystick_yaw.as_usec();
       usec const roll  = joystick_roll.as_usec();

       if ( (abs(pitch - init_trim_pitch) > max_error) ||
            (abs(roll  - init_trim_roll) > max_error)  ||
            (abs(yaw   - init_trim_yaw) > max_error) ){

          hal.console->printf("detected too much movement while setting trims\n");
          return false;
       }

       pitch_sum += pitch;
       yaw_sum  += yaw;
       roll_sum += roll;
       ++count;
   }
   joystick_pitch.set_trim(pitch_sum/count);
   joystick_roll.set_trim(roll_sum/count);
   joystick_pitch.set_trim(yaw_sum/count);
   return true;

}

bool Plane::thrust_failsafe_state_detected()const
{
  // return (g.thrust_fs_enabled) && channel_thrust.read_joystick_usec() <= g.thrust_fs_value;
   return (g.thrust_fs_enabled) && (joystick_thrust.as_usec() <= usec{g.thrust_fs_value.get()});
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
