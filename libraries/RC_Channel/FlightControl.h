#ifndef AERFLITE_ARDUPILOT_FLIGHT_CONTROL_H_INCLUDED
#define AERFLITE_ARDUPILOT_FLIGHT_CONTROL_H_INCLUDED

#include "JoystickInput.h"
#include <quan/constrain.hpp>
#include <quan/time.hpp>

template <typename FlightAxisT>
struct FltCtrlInput{
   
   typedef quan::angle_<int16_t>::cdeg cdeg;
   void set(JoystickInput<FlightAxisT> const & in){ this->set(in.as_angle());}

   void set( cdeg in) {m_value = in;}

   cdeg get() const {return m_value;}

  private:
      cdeg m_value;
};

template <>
struct FltCtrlInput<FlightAxis::Thrust>{
   typedef quan::force_<int16_t>::N force_type;
   void set(JoystickInput<FlightAxis::Thrust> const & in){ this->set(in.as_force());}

   void set( force_type in) {m_value = in;}

   void constrain(force_type min_in,force_type max_in) { m_value = quan::constrain(m_value,min_in,max_in);}
   force_type get() const {return m_value;}

  private:
      force_type m_value;
};

struct FltCtrlOutput_base {
   typedef quan::time_<int16_t>::us usec;
   usec as_usec() const {return m_value;}
   void set (usec in) { m_value = static_cast<usec>(in);}
   float as_float() const {return 0.f;}
   private:
      usec m_value;
};

template <typename FlightAxisT>
struct FltCtrlOutput : FltCtrlOutput_base{
   using FltCtrlOutput_base::set;
   void set(JoystickInput<FlightAxisT> const & in){ }
   void set(FltCtrlInput<FlightAxisT> const & in){ }
};

template <>
struct FltCtrlOutput<FlightAxis::Thrust> : FltCtrlOutput_base{
   using FltCtrlOutput_base::set;
   void set(JoystickInput<FlightAxis::Thrust> const & in){ }
   void set(FltCtrlInput<FlightAxis::Thrust> const & in){ }
   void enable(){}
   void disable(){}
   bool is_enabled() const {return false;}
};

#endif // AERFLITE_ARDUPILOT_FLIGHT_CONTROL_H_INCLUDED
