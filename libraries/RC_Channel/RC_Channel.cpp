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

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include "RC_Channel.h"

extern const AP_HAL::HAL& hal;

/// global array with pointers to all APM RC channels, will be used by AP_Mount
/// and AP_Camera classes / It points to RC input channels.
RC_Channel *RC_Channel::rc_ch[RC_Channel::max_channels];

const AP_Param::GroupInfo RC_Channel::var_info[] = {
    // @Param: MIN
    // @DisplayName: RC min PWM
    // @Description: RC minimum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
    // @Units: pwm
    // @Range: 800 2200
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("MIN",  0, RC_Channel, m_radio_min, 1100),

    // @Param: TRIM
    // @DisplayName: RC trim PWM
    // @Description: RC trim (neutral) PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
    // @Units: pwm
    // @Range: 800 2200
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("TRIM", 1, RC_Channel, m_radio_trim, 1500),

    // @Param: MAX
    // @DisplayName: RC max PWM
    // @Description: RC maximum PWM pulse width. Typically 1000 is lower limit, 1500 is neutral and 2000 is upper limit.
    // @Units: pwm
    // @Range: 800 2200
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("MAX",  2, RC_Channel, m_radio_max, 1900),

    // @Param: REV
    // @DisplayName: RC reverse
    // @Description: Reverse servo operation. Set to 1 for normal (forward) operation. Set to -1 to reverse this channel.
    // @Values: -1:Reversed,1:Normal
    // @User: Advanced
    AP_GROUPINFO("REV",  3, RC_Channel, _reverse, 1),

    // Note: index 4 was used by the previous _dead_zone value. We
    // changed it to 5 as dead zone values had previously been
    // incorrectly saved, overriding user values. They were also
    // incorrectly interpreted for the throttle on APM:Plane

    // @Param: DZ
    // @DisplayName: RC dead-zone
    // @Description: dead zone around trim or bottom
    // @Units: pwm
    // @Range: 0 200
    // @User: Advanced
    AP_GROUPINFO("DZ",   5, RC_Channel, _dead_zone, 0),

    AP_GROUPEND
};

// setup the control preferences
void
RC_Channel::set_range()
{
    _type   = channel_type::range;
}

void
RC_Channel::set_angle()
{
    _type  = channel_type::angle;
}

void
RC_Channel::set_default_dead_zone()
{
    _dead_zone.set_default(default_dead_zone);
}

bool
RC_Channel::get_reverse(void) const
{
    if (_reverse == -1) {
        return true;
    }
    return false;
}

// read input from APM_RC - create a control_in value
void
RC_Channel::set_pwm(int16_t pwm)
{
    set_radio_in(pwm);

    if (_type == channel_type::range) {
        set_control_in( pwm_to_range());
    } else {
        //channel_type::angle
        set_control_in(pwm_to_angle());
    }
}

/*
  call read() and set_pwm() on all channels
 */
void
RC_Channel::set_pwm_all(void)
{
    for (uint8_t i=0; i<max_channels; i++) {
        if (rc_ch[i] != NULL) {
            rc_ch[i]->set_pwm(rc_ch[i]->read());
        }
    }
}

// read input from APM_RC - create a control_in value, but use a 
// zero value for the dead zone. When done this way the control_in
// value can be used as servo_out to give the same output as input
void
RC_Channel::set_pwm_no_deadzone(int16_t pwm)
{
    set_radio_in(pwm);

    if (_type == channel_type::range) {
        set_control_in(pwm_to_range_dz(0));
    } else {
        //angle
        set_control_in(pwm_to_angle_dz(0));
    }
}

// returns just the PWM without the offset from radio_min
void
RC_Channel::calc_pwm(void)
{
    if(_type == channel_type::range) {
        pwm_out         = range_to_pwm();
        this->set_radio_out((_reverse >= 0) ? (get_radio_min() + pwm_out) : (get_radio_max() - pwm_out) );

    }else{     // channel_type::angle
        pwm_out         = angle_to_pwm();
        this->set_radio_out(pwm_out + get_radio_trim());
    }
    // limit the values
    this->set_radio_out( constrain_int16(this->get_radio_out(), this->get_radio_min(), this->get_radio_max()));
}

void
RC_Channel::load_eeprom(void)
{
    m_radio_min.load();
    m_radio_trim.load();
    m_radio_max.load();
    _reverse.load();
    _dead_zone.load();
}

void
RC_Channel::save_eeprom(void)
{
    m_radio_min.save();
    m_radio_trim.save();
    m_radio_max.save();
    _reverse.save();
    _dead_zone.save();
}

/*
  return an "angle in centidegrees" (normally -4500 to 4500) from
  the current radio_in value using the specified dead_zone
  channel is in angle mode
 */
int16_t
RC_Channel::pwm_to_angle_dz(uint16_t dead_zone)const
{
    int16_t radio_trim_high = get_radio_trim() + dead_zone;
    int16_t radio_trim_low  = get_radio_trim() - dead_zone;

    // prevent div by 0
    if ((radio_trim_low - get_radio_min()) == 0 || (get_radio_max() - radio_trim_high) == 0)
        return 0;

    int16_t reverse_mul = (_reverse==-1?-1:1);
    if(get_radio_in() > radio_trim_high) {
        return reverse_mul * ((int32_t)angle_min_max * (int32_t)(get_radio_in() - radio_trim_high)) / (int32_t)(get_radio_max()  - radio_trim_high);
    }else if(get_radio_in() < radio_trim_low) {
        return reverse_mul * ((int32_t)angle_min_max * (int32_t)(get_radio_in() - radio_trim_low)) / (int32_t)(radio_trim_low - get_radio_min());
    }else
        return 0;
}

/*
  return an "angle in centidegrees" (normally -4500 to 4500) from
  the current radio_in value
 */
int16_t
RC_Channel::pwm_to_angle()const
{
	return pwm_to_angle_dz(_dead_zone);
}

// in angle mode
int16_t
RC_Channel::angle_to_pwm()const
{
    int16_t reverse_mul = (_reverse==-1?-1:1);
    if((this->get_servo_out() * reverse_mul) > 0) {
        return reverse_mul * ((int32_t)this->get_servo_out() * (int32_t)(get_radio_max() - get_radio_trim())) / (int32_t)angle_min_max;
    } else {
        return reverse_mul * ((int32_t)this->get_servo_out() * (int32_t)(get_radio_trim() - get_radio_min())) / (int32_t)angle_min_max;
    }
}

/*
  convert a pulse width modulation value to a value in the configured
  range, using the specified deadzone
 channel is in range mode
 */
int16_t
RC_Channel::pwm_to_range_dz(uint16_t dead_zone)const
{
    int16_t r_in = constrain_int16(get_radio_in(), get_radio_min(), get_radio_max());

    if (_reverse == -1) {
	    r_in = get_radio_max() - (r_in - get_radio_min());
    }

    int16_t radio_trim_low  = get_radio_min() + dead_zone;

    if (r_in > radio_trim_low)
        return (range_low + ((int32_t)(range_high - range_low) * (int32_t)(r_in - radio_trim_low)) / (int32_t)(get_radio_max() - radio_trim_low));
    else if (dead_zone > 0)
        return 0;
    else
        return range_low;
}

/*
  convert a pulse width modulation value to a value in the configured
  range
 */
int16_t
RC_Channel::pwm_to_range()const
{
    return pwm_to_range_dz(_dead_zone);
}


// only called if type is range
// private fyun
int16_t
RC_Channel::range_to_pwm()const
{

    return ((int32_t)(this->get_servo_out() - range_low) * (int32_t)(get_radio_max() - get_radio_min())) / (int32_t)(range_high - range_low);
}

// ------------------------------------------

float
RC_Channel::norm_input()
{
    float ret;
    int16_t reverse_mul = (_reverse==-1?-1:1);
    if (get_radio_in() < get_radio_trim()) {
        ret = reverse_mul * (float)(get_radio_in() - get_radio_trim()) / (float)(get_radio_trim() - get_radio_min());
    } else {
        ret = reverse_mul * (float)(get_radio_in() - get_radio_trim()) / (float)(get_radio_max()  - get_radio_trim());
    }
    return constrain_float(ret, -1.0f, 1.0f);
}


float
RC_Channel::norm_output()const
{
    int16_t mid = (get_radio_max() + get_radio_min()) / 2;
    float ret;
    if (mid <= get_radio_min()) {
        return 0;
    }
    if (this->get_radio_out() < mid) {
        ret = (float)(this->get_radio_out() - mid) / (float)(mid - get_radio_min());
    } else if (this->get_radio_out() > mid) {
        ret = (float)(this->get_radio_out() - mid) / (float)(get_radio_max()  - mid);
    } else {
        ret = 0;
    }
    if (_reverse == -1) {
	    ret = -ret;
    }
    return ret;
}

void RC_Channel::output() const
{
    hal.rcout->write(_ch_out, this->get_radio_out());
}

void RC_Channel::output_trim() const
{
    hal.rcout->write(_ch_out, get_radio_trim());
}

void RC_Channel::output_trim_all()
{
    for (uint8_t i=0; i<max_channels; i++) {
        if (rc_ch[i] != NULL) {
            rc_ch[i]->output_trim();
        }
    }
}

/*
  setup the failsafe value to the trim value for all channels
 */
void RC_Channel::setup_failsafe_trim_all()
{
    for (uint8_t i=0; i<max_channels; i++) {
        if (rc_ch[i] != NULL) {
            hal.rcout->set_failsafe_pwm(1U<<i, rc_ch[i]->get_radio_trim());
        }
    }
}

uint16_t
RC_Channel::read() const
{
    return hal.rcin->read(_ch_out);
}

void
RC_Channel::enable_out()
{
    hal.rcout->enable_ch(_ch_out);
}

void
RC_Channel::disable_out()
{
    hal.rcout->disable_ch(_ch_out);
}

RC_Channel *RC_Channel::rc_channel(uint8_t i)
{
    if (i >= max_channels) {
        return NULL;
    }
    return rc_ch[i];
}
