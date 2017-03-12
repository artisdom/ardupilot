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

    RC_Channel(uint8_t ch_in, uint8_t ch_out, channel_type type_in, int16_t init_value) :
        m_joystick_input_usec{init_value} // stick units usec
        ,m_control_in{0} // angle or range units
        ,m_temp_output{0}  // degrees * 100 or 0 to 100
        ,m_output_usec{init_value}  // raw pwm
        ,m_channel_type{type_in}
        ,m_rcin_idx{ch_in} 
        ,m_rcout_idx{ch_out}
    {
       // AP_Param::setup_object_defaults(this, var_info);
    }

    // startup
    void        load_eeprom(void);
    void        save_eeprom(void);
    void        save_trim(void);

    void        read_joystick_input() { this->set_joystick_input_usec(this->read_joystick_usec());}
    void        set_joystick_input_centre() { this->set_joystick_input_usec(this->get_joystick_in_trim_usec());}
    void        set_joystick_input_min() { this->set_joystick_input_usec(this->get_joystick_in_min_usec());}

    // generate output from servo_out value
    void        calc_output_from_temp_output(void);

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
    void       write_output_usec() const;
 //   void       output_trim() const;
    // read from rcin
    uint16_t   read_joystick_usec() const;
    void       enable_out()const;
    void       disable_out()const;
    static const struct AP_Param::GroupInfo         var_info[];

    // radio_out is in same units as rcin e.g raw pwm units approx 1000 to 2000 us
    void    set_output_usec(int16_t v) { m_output_usec = v;}
    int16_t get_output_usec()const{ return m_output_usec;}
    int16_t get_output_max_usec()const { return m_output_max_usec;}
    int16_t get_output_min_usec()const { return m_output_min_usec;} 
    int16_t get_output_trim_usec()const { return m_output_trim_usec;}    

    // in the units either angle or thrust range
    void   set_temp_out(int16_t v) { m_temp_output = v;}
    // 0 to 100 for thrust
    int16_t get_temp_out() const { return m_temp_output;}

    int16_t get_control_in()const { return m_control_in;}

    int16_t get_joystick_in_usec()const {return m_joystick_input_usec;}
    int16_t get_joystick_in_min_usec()const {return m_joystick_in_min_usec;}
    int16_t get_joystick_in_max_usec()const {return m_joystick_in_max_usec;}
    int16_t get_joystick_in_trim_usec()const {return m_joystick_in_trim_usec;}
    uint8_t    get_rcin_index() const { return m_rcin_idx;}
#if !defined QUAN_PUBLIC_PRIVATE_MEMBERS
private:
#else
    void set_reversed (bool b) { m_is_reversed = b;}
#endif
        // used externally by stick_mix_channel atm
    int16_t    pwm_to_angle()const;
        // This should apply to the input surely?
    bool       input_is_reversed(void) const;
    void       set_joystick_input_usec(int16_t pwm);
    void       set_joystick_pwm_usec(int16_t v);
    void       set_control_in( int16_t v) { m_control_in = v;}
    int16_t    angle_to_pwm()const;
    int16_t    pwm_to_range()const;

    int16_t    range_to_pwm()const;

   // static constexpr int16_t default_dead_zone = 30;
   static constexpr int16_t angle_min_max = 4500;
   static constexpr int16_t range_high = 100;
   static constexpr int16_t range_low = 0;

   static constexpr int16_t m_joystick_in_min_usec = 1000;
   static constexpr int16_t m_joystick_in_max_usec = 2000;
   static constexpr int16_t m_joystick_in_trim_usec = (m_joystick_in_max_usec + m_joystick_in_min_usec)/ 2; 

   static constexpr int16_t m_output_min_usec = 1000;
   static constexpr int16_t m_output_max_usec  = 2000;
   static constexpr int16_t m_output_trim_usec = (m_output_max_usec + m_output_min_usec) / 2;
    // eeprom value never appears to be used, just has default updated in init_rc_in
     // 1 for forawrd -1 for reverse
   bool m_is_reversed = false;
  //  AP_Int16        _dead_zone;

    //direct stick input I think e.g approx 1000 to 2000 us;
    // looks to be same units as read()
    //{
    int16_t         m_joystick_input_usec;

    // now just the m_joystick_input_usec value converted to the cdeg units
    // however see the failsafe_set_control_in
    // which is used in failsafe (TODO
    // change functionlaity so this is not necessary by using rc_in direct)
    int16_t         m_control_in;
    //}

    // servo_out looks to be same units as control_in
    int16_t         m_temp_output;

    int16_t         m_output_usec;

    channel_type const    m_channel_type;

    // where to get input from
    uint8_t const         m_rcin_idx;
    // where to send output to
    uint8_t const         m_rcout_idx;
};

#endif
