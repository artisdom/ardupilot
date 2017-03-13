
#ifndef AERFLITE_JOYSTICK_INPUT_BASE_HPP_INCLUDED
#define AERFLITE_JOYSTICK_INPUT_BASE_HPP_INCLUDED

/// @file	JoystickInput.h
/// @brief	JoystickInput manager, with EEPROM-backed storage of constants.

#include <AP_Param/AP_Param.h>
#include <quan/time.hpp>

#include <quan/angle.hpp>

/// @class	JoystickInput
/// @brief	rcin input buffer and switch abstraction to map an rc channel
  //  to map a specific joystick input
  // e.g 
  // JoyStickInput joystick_yaw{0};  // map to ch0 

extern const AP_HAL::HAL& hal;

struct JoystickInput_base {
    typedef quan::time_<int16_t>::us usec;
    static constexpr uint8_t max_channels = 14;
    // note that the init_value is not reversed
    // even if the raw input sense is reversed
    // so the init_value doesnt need to change
    // irespective of which sense user wants for input
    JoystickInput_base(uint8_t ch_in, usec init_value) :
        m_value{init_value} 
        ,m_rcin_idx{ch_in} 
    {
       //potentially reversed from eeprom
       // AP_Param::setup_object_defaults(this, var_info);
    }
    // startup
    void        load_eeprom(void);
    void        save_eeprom(void);
    void        save_trim(void);

    /* 
      update from rcin
      reverse the sense of the raw rcin value if input_sense_reversed() is true
    */
    void        update() ;
//    {
//        usec const rcin{hal.rcin->read(m_rcin_idx)};
//        if ( input_sense_reversed()){
//            m_value = (get_min() + get_max()) - rcin ;
//        }else{
//            m_value = rcin;
//        }
//    }
   /* set the value to centre */
    void        set_centre() { this->m_value = get_trim();}
   /* set the value to min */
    void        set_min() { this->m_value = get_min();}

    /*
      return a normalised input for a channel, in range -1 to 1,
      centered around the channel trim. 
     */
    float       as_float()const ;
//    { 
//      return quan::constrain(
//      (this->m_value - get_trim()) / ((get_max() - get_min())/2.f)
//      ,-1.f, 1.f
//      );
//    }

    usec as_usec()const {return m_value;}

private:   
   static constexpr usec m_min{1000};
   static constexpr usec m_max{2000};
   static constexpr usec m_trim = (m_max + m_min)/ 2; 
public:
    static constexpr usec get_min() {return m_min;}
    static constexpr usec get_max() {return m_max;}
    static constexpr usec get_trim() {return m_trim;}
    uint8_t get_rcin_index() const { return m_rcin_idx;}
#if !defined QUAN_PUBLIC_PRIVATE_MEMBERS
private:
#else
    void set_reversed (bool b) { m_is_reversed = b;}
#endif
   bool       input_sense_reversed(void) const { return m_is_reversed;}
//   void       set_joystick_input_usec(int16_t pwm);
//   void       set_joystick_pwm_usec(int16_t v);
   usec           m_value;
   uint8_t const  m_rcin_idx;
   bool           m_is_reversed;
   JoystickInput_base(JoystickInput_base const & ) = delete;
   JoystickInput_base & operator =(JoystickInput_base const & ) = delete; 
};

struct JoystickInput_angle : JoystickInput_base {

   JoystickInput_angle(uint8_t ch_in) : JoystickInput_base{ch_in,get_trim()}{}
   typedef quan::angle_<int16_t>::cdeg cdeg;

   cdeg as_angle() const ;
   
};

#endif  //AERFLITE_JOYSTICK_INPUT_BASE_HPP_INCLUDED
