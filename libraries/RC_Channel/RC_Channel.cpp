// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 *       RC_Channel.cpp - Radio library for Arduino
 *       Code by Jason Short. DIYDrones.com
 *
 */

#include <stdlib.h>
#include <math.h>
#include <quan/constrain.hpp>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include "RC_Channel.h"

extern const AP_HAL::HAL& hal;

// TODO for input only
bool
RC_Channel::input_is_reversed(void) const
{
    return m_is_reversed ;
}

void
RC_Channel::set_joystick_pwm_usec(int16_t v)
{
   int16_t v1 = (input_is_reversed()) ? (get_joystick_in_max_usec() + get_joystick_in_max_usec()) - v : v;
   m_joystick_input_usec =
     quan::constrain( v1
         , get_joystick_in_min_usec()
         , get_joystick_in_max_usec()
     );
}

// these two set the radio_in
// the use that to set control_in
// therefore control_in is a function of radio_in
void
RC_Channel::set_joystick_input_usec(int16_t pwm_usec)
{
    set_joystick_pwm_usec(pwm_usec);

    if (m_channel_type == channel_type::range) {
        set_control_in(pwm_to_range());
    } else {
        set_control_in(pwm_to_angle());
    }
}

// private
// in angle mode
int16_t
RC_Channel::angle_to_pwm()const
{
    if(this->get_temp_out() > 0) {
        return ((int32_t)this->get_temp_out() * (int32_t)(get_output_max_usec() - get_output_trim_usec())) / (int32_t)angle_min_max;
    } else {
        return ((int32_t)this->get_temp_out() * (int32_t)(get_output_trim_usec() - get_output_min_usec())) / (int32_t)angle_min_max;
    }
}
// private
// in range mode
int16_t
RC_Channel::range_to_pwm()const
{
    return ((int32_t)(this->get_temp_out() - range_low) * (int32_t)(get_output_max_usec() - get_output_min_usec())) / (int32_t)(range_high - range_low);
}

/*
   calculate the output_value value from the servo_out value
   The servo out value being in either centidegrees or percent for thrust
*/
void
RC_Channel::calc_output_from_temp_output(void)
{
    int16_t output_usec = 0;
    if(m_channel_type == channel_type::range) {
        output_usec = (get_output_min_usec() + range_to_pwm());
    }else{     // channel_type::angle
        output_usec = angle_to_pwm()+ get_output_trim_usec();
    }
    this->set_output_usec( constrain_int16(output_usec, this->get_output_min_usec(), this->get_output_max_usec()));
}

/*
  used by stick mixing
  return an "angle in centidegrees" (normally -4500 to 4500) from
  the current radio_in value
 */
int16_t
RC_Channel::pwm_to_angle()const
{
   //return pwm_to_angle_dz(_dead_zone);
   int16_t const joystick_trim_usec = get_joystick_in_trim_usec();
   int16_t const joystick_in_usec = get_joystick_in_usec();

   if(joystick_in_usec > joystick_trim_usec) {
      int32_t const diff_max_trim = get_joystick_in_max_usec() - joystick_trim_usec;
      if ( diff_max_trim == 0){
         return 0;
      }else{
         return ((int32_t)angle_min_max * (int32_t)(joystick_in_usec - joystick_trim_usec)) / diff_max_trim;
      }
   }else {
      if(joystick_in_usec < joystick_trim_usec) {
         int32_t const diff_trim_min = joystick_trim_usec - get_joystick_in_min_usec();
         if(diff_trim_min == 0){
            return 0;
         }else{
            return ((int32_t)angle_min_max * (int32_t)(joystick_in_usec - joystick_trim_usec)) / diff_trim_min;
         }
      }else{ 
         return 0;
      }
   }
}


/*
  private
  convert a pulse width modulation value to a value in the configured
  range
 */
int16_t
RC_Channel::pwm_to_range()const
{
   // return pwm_to_range_dz(_dead_zone);
    int16_t const joystick_min_usec = get_joystick_in_min_usec() ;
    int16_t const joystick_max_usec = get_joystick_in_max_usec();
     
    int16_t const joystick_in_usec = constrain_int16(get_joystick_in_usec(), joystick_min_usec, joystick_max_usec);

    if (joystick_in_usec > joystick_min_usec){
        int32_t const diff_max_min = (int32_t)(joystick_max_usec - joystick_min_usec);
        if ( diff_max_min == 0){
           return 0;
        }else{
           return (range_low + ((int32_t)(range_high - range_low) * (int32_t)(joystick_in_usec - joystick_min_usec)) / diff_max_min);
        }
    }else{
        return range_low;
    }
}

// ------------------------------------------
// convert radio_in to a value in range -1 to 1
// assumes its an angle type
float
RC_Channel::norm_input()const
{
    return quan::constrain(
      (this->get_joystick_in_usec() - get_joystick_in_trim_usec()) / ((get_joystick_in_max_usec() - get_joystick_in_min_usec())/2.f)
      ,-1.f, 1.f
    );
}

/*
only used by non essential functions
*/
float
RC_Channel::norm_output()const
{
   return quan::constrain(
      (this->get_output_usec() - get_output_trim_usec()) / ((get_output_max_usec() - get_output_min_usec())/2.f)
       ,-1.f, 1.f
   );
}

void RC_Channel::write_output_usec() const
{
    hal.rcout->write(m_rcout_idx, this->get_output_usec());
}

uint16_t
RC_Channel::read_joystick_usec() const
{
    return hal.rcin->read(m_rcin_idx);
}

void
RC_Channel::enable_out()const
{
    hal.rcout->enable_ch(m_rcout_idx);
}

void
RC_Channel::disable_out()const
{
    hal.rcout->disable_ch(m_rcout_idx);
}

void
RC_Channel::load_eeprom(void)
{
//    m_radio_min.load();
//    m_radio_trim.load();
//    m_joystick_in_max_usec.load();
//    m_reverse.load();
//    _dead_zone.load();
}

void
RC_Channel::save_eeprom(void)
{
//    m_radio_min.save();
//    m_radio_trim.save();
//    m_joystick_in_max_usec.save();
//    m_reverse.save();
  //  _dead_zone.save();
}
