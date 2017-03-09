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

    RC_Channel(uint8_t ch_out) :
        _type{channel_type::angle},
        _ch_out{ch_out} 
    {
        AP_Param::setup_object_defaults(this, var_info);
//        if (ch_out < max_channels) {
//           rc_ch[ch_out] = this;
//        }
    }

    // startup
    void        load_eeprom(void);
    void        save_eeprom(void);
    void        save_trim(void);

    // setup the control preferences
    void        set_range();
    void        set_angle();
    bool        get_reverse(void) const;
  
    
    // read input from APM_RC - create a control_in value
    void        set_pwm(int16_t pwm);
 //   static void set_pwm_all(void);
    void        set_pwm_no_deadzone(int16_t pwm);

    // generate PWM from servo_out value
    void        calc_pwm(void);

    /*
      return a normalised input for a channel, in range -1 to 1,
      centered around the channel trim. Ignore deadzone.
     */
    float       norm_input();

    /*
      return a normalised input for a channel, in range -1 to 1,
      centered around the channel trim. Take into account the deadzone
    */
    float                                           norm_output()const;



    //send values to the PWM timers for output
    void                                            output() const;
    void                                            output_trim() const;
  //  static void                                     output_trim_all();
   // static void                                     setup_failsafe_trim_all();
    // reads rcin
    uint16_t                                        read() const;
    void                                            enable_out();
    void                                            disable_out();

  //  static RC_Channel *rc_channel(uint8_t i);
    static const struct AP_Param::GroupInfo         var_info[];

    // radio_out is in same units as rcin e.g raw pwm units approx 1000 to 2000 us
    void set_radio_out(int16_t v) { m_radio_out = v;}
    int16_t get_radio_out()const{ return m_radio_out;}

    void set_servo_out(int16_t v) { m_servo_out = v;}
    
    // 0 to 100 for throttle
    int16_t get_servo_out() const { return m_servo_out;}


    //{
    // looks like these 2 are onlu called in Plane::control_failsafe
    void set_radio_in(int16_t v) {m_radio_in = v;}
    void set_control_in( int16_t v) { m_control_in = v;}
    // }

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
private:
    
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

     // pwm is stored here direct stick input I think e.g approx 1000 to 2000 us;
    // looks to be same units as Read()
    int16_t         m_radio_in;

    // The user stick input ?
    //in the angle or range units,?
    int16_t         m_control_in;

    // logical actuator output
    // current values to the servos - degrees * 100 (approx assuming servo is -45 to 45 degrees except [3] is 0 to 100
    // This is written by the various pitch/roll yaw controllers
    // for pitch and roll one way this is set is is by assign from RC_Channel::pwm_to_angle;
    // same units as control_in
    int16_t         m_servo_out;
    // is in same units as rcin e.g raw pwm units
    int16_t         m_radio_out;

    int16_t         pwm_to_angle_dz(uint16_t dead_zone)const;

    // pwm out type depends on the _type member
    // see calc_pwm memfun
    int16_t         pwm_out;
    // 
    AP_Int8         _reverse;
    // eeprom value never appears to be used, just has default updated in init_rc_in
    AP_Int16        _dead_zone;
    enum class channel_type : bool { angle,range };
    channel_type    _type;

 //   static RC_Channel *rc_ch[max_channels];

protected:
    // channel index for rc input?
    // nope used by both :(
    // add a  _ch_in ?
    uint8_t const         _ch_out;
};

#endif
