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
        m_radio_in{init_value} // stick units usec
        ,m_control_in{0} // angle or range units
        ,m_servo_out{0}  // degrees * 100 or 0 to 100
        ,m_output{init_value}  // raw pwm
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

    // This should apply to the input surely?
    bool        get_reverse(void) const;
  
    void        read_joystick() { this->set_pwm(this->read());}
    void        set_joystick_centre() { this->set_pwm(this->get_radio_trim());}
    void        set_joystick_min() { this->set_pwm(this->get_radio_min());}

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
 //   void       output_trim() const;
    uint16_t   read() const;
    void       enable_out()const;
    void       disable_out()const;
    static const struct AP_Param::GroupInfo         var_info[];

    // radio_out is in same units as rcin e.g raw pwm units approx 1000 to 2000 us
    void set_radio_out(int16_t v) { m_output = v;}
    int16_t get_radio_out()const{ return m_output;}

    // in the units either angle or thrust range
    void set_servo_out(int16_t v) { m_servo_out = v;}
    
    // 0 to 100 for thrust
    int16_t get_servo_out() const { return m_servo_out;}

    int16_t get_control_in()const { return m_control_in;}

    int16_t get_radio_in()const {return m_radio_in;}


    int16_t get_radio_min()const {return m_radio_min;}
  //  void set_radio_min(int16_t v) {m_radio_min = v;}

    int16_t get_radio_max()const {return m_radio_max;}
   // void set_radio_max(int16_t v) {m_radio_max = v;}

    int16_t get_radio_trim()const {return m_radio_trim;}
   // void set_radio_trim(int16_t v) {m_radio_trim = v;}
    
    // used externally by stick_mix_channel atm
    int16_t     pwm_to_angle()const;

    uint8_t get_rcin_index() const { return m_rcin_idx;}
private:
    void       set_pwm(int16_t pwm);
    void       set_radio_in(int16_t v) {m_radio_in = v;}
    void       set_control_in( int16_t v) { m_control_in = v;}
   // int16_t     pwm_to_angle_dz(uint16_t dead_zone)const;
   // int16_t     pwm_to_range_dz(uint16_t dead_zone)const;
    int16_t    angle_to_pwm()const;
    int16_t    pwm_to_range()const;

    int16_t    range_to_pwm()const;

    static constexpr int16_t default_dead_zone = 30;
    static constexpr int16_t angle_min_max = 4500;
    static constexpr int16_t range_high = 100;
    static constexpr int16_t range_low = 0;

    //radio_min, radio_max , radio_trim in usec units
//    AP_Int16        m_radio_min;
//    AP_Int16        m_radio_trim;
//    AP_Int16        m_radio_max;
//    AP_Int8         _reverse;
     static constexpr int16_t m_radio_min = 1000;
     static constexpr int16_t m_radio_max = 2000;
     static constexpr int16_t m_radio_trim = (m_radio_max + m_radio_min)/ 2; 
    // eeprom value never appears to be used, just has default updated in init_rc_in
     // 1 for forawrd -1 for reverse
     static constexpr int8_t m_reverse = 1;
  //  AP_Int16        _dead_zone;

    //direct stick input I think e.g approx 1000 to 2000 us;
    // looks to be same units as read()
    //{
    int16_t         m_radio_in;

    // now just the m_radio_in value converted to the cdeg units
    // however see the failsafe_set_control_in
    // which is used in failsafe (TODO
    // change functionlaity so this is not necessary by using rc_in direct)
    int16_t         m_control_in;
    //}

    // servo_out looks to be same units as control_in
    int16_t         m_servo_out;

    int16_t         m_output;

    channel_type const    m_channel_type;

    // where to get input from
    uint8_t const         m_rcin_idx;
    // where to send output to
    uint8_t const         m_rcout_idx;
};

#endif
