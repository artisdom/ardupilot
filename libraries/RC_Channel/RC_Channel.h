// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/// @file	RC_Channel.h
/// @brief	RC_Channel manager, with EEPROM-backed storage of constants.

#ifndef __RC_CHANNEL_H__
#define __RC_CHANNEL_H__

#include <AP_Param/AP_Param.h>

/// @class	RC_Channel
/// @brief	Object managing one RC channel
class RC_Channel {
public:
    static constexpr uint8_t max_channels = 14;

    /// Constructor
    ///
    /// @param key      EEPROM storage key for the channel trim parameters.
    /// @param name     Optional name for the group.
    ///
    enum class channel_type : bool { angle,range };

    RC_Channel(uint8_t ch_in, uint8_t ch_out, channel_type type_in) :
        m_radio_in{0} // stick units usec
        ,m_control_in{0} // angle or range units
        ,m_servo_out{0}  // degrees * 100 or 0 to 100
        ,m_radio_out{0}  // raw pwm
        ,m_channel_type{type_in}
        ,m_rcin_idx{ch_in} 
        ,m_rcout_idx{ch_out}
    {
        AP_Param::setup_object_defaults(this, var_info);
    }

    // startup
    void        load_eeprom(void);
    void        save_eeprom(void);
    void        save_trim(void);

    // This should apply to the input surely?
    bool        get_reverse(void) const;
  
    void        set_pwm(int16_t pwm);

    void        set_pwm_no_deadzone(int16_t pwm);

    // generate PWM from servo_out value
    void        calc_pwm(void);

    /*
      return a normalised input for a channel, in range -1 to 1,
      centered around the channel trim. Ignore deadzone.
     */
    float       norm_input()const;

    /*
      return a normalised input for a channel, in range -1 to 1,
      centered around the channel trim. Take into account the deadzone
    */
    float      norm_output()const;

    //send values to the PWM timers for output
    void       output() const;
    void       output_trim() const;
    uint16_t   read() const;
    void       enable_out()const;
    void       disable_out()const;
    static const struct AP_Param::GroupInfo         var_info[];

    // radio_out is in same units as rcin e.g raw pwm units approx 1000 to 2000 us
    void set_radio_out(int16_t v) { m_radio_out = v;}
    int16_t get_radio_out()const{ return m_radio_out;}

    // in the units either angle or throttle range
    void set_servo_out(int16_t v) { m_servo_out = v;}
    
    // 0 to 100 for throttle
    int16_t get_servo_out() const { return m_servo_out;}

    int16_t get_control_in()const { return m_control_in;}

    int16_t get_radio_in()const {return m_radio_in;}


    int16_t get_radio_min()const {return m_radio_min.get();}
    void set_radio_min(int16_t v) {m_radio_min.set(v);}

    int16_t get_radio_max()const {return m_radio_max.get();}
    void set_radio_max(int16_t v) {m_radio_max.set(v);}

    int16_t get_radio_trim()const {return m_radio_trim.get();}
    void set_radio_trim(int16_t v) {m_radio_trim.set(v);}
    
// used externally by stick_mix_channel atm
    int16_t     pwm_to_angle()const;

    void        set_default_dead_zone();
    // only used for throttle in failsafe
    void failsafe_set_control_in(int16_t v) { set_control_in(v);}
private:
    void set_radio_in(int16_t v) {m_radio_in = v;}
    void set_control_in( int16_t v) { m_control_in = v;}
    int16_t     pwm_to_angle_dz(uint16_t dead_zone)const;
    int16_t     pwm_to_range_dz(uint16_t dead_zone)const;
    int16_t     angle_to_pwm()const;
    int16_t     pwm_to_range()const;

    int16_t     range_to_pwm()const;

    static constexpr int16_t default_dead_zone = 30;
    static constexpr int16_t angle_min_max = 4500;
    static constexpr int16_t range_high = 100;
    static constexpr int16_t range_low = 0;

    //radio_min, radio_max , radio_trim in usec units
    AP_Int16        m_radio_min;
    AP_Int16        m_radio_trim;
    AP_Int16        m_radio_max;
    AP_Int8         _reverse;
    // eeprom value never appears to be used, just has default updated in init_rc_in
    AP_Int16        _dead_zone;

    //direct stick input I think e.g approx 1000 to 2000 us;
    // looks to be same units as Read()
    int16_t         m_radio_in;

    // The user stick input ?
    //in the angle or range units,?
    // This is usually a function of radio_in above
    // the exception is when Plane::control_failsafe is called
    int16_t         m_control_in;
    // servo_out looks to be same units as control_in
    int16_t         m_servo_out;

    int16_t         m_radio_out;

    channel_type const    m_channel_type;

    // where to get input from
    uint8_t const         m_rcin_idx;
    // where to send output to
    uint8_t const         m_rcout_idx;
};

#endif
