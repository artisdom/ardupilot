#ifndef AERFLITE_ARDUPILOT_FLIGHT_CONTROL_H_INCLUDED
#define AERFLITE_ARDUPILOT_FLIGHT_CONTROL_H_INCLUDED

#include "JoystickInput.h"

template <typename FlightAxisT>
struct FltCtrlInput{
   
   typedef quan::angle_<int16_t>::cdeg cdeg;
   void set(JoystickInput<FlightAxisT> const & in){ this->set(in.eval());}

   void set( cdeg in) {m_value = in;}

   cdeg get() const {return m_value;}

  private:
      cdeg m_value;
};

template <>
struct FltCtrlInput<FlightAxis::Thrust>{
   typedef quan::force_<int16_t>::N force_type;
   void set(JoystickInput<FlightAxis::Thrust> const & in){ this->set(in.eval());}

   void set( force_type in) {m_value = in;}

   force_type get() const {return m_value;}

  private:
      force_type m_value;
};

template <typename FlightAxisT>
struct FltCtrlOutput{
   
   typedef quan::time_<int16_t>::us usec;

   void set(JoystickInput<FlightAxisT> const & in){ }

   void set(FltCtrlInput<FlightAxisT> const & in){ }

   void set( usec in) {}

   usec get() const {return m_value;}

  private:
      usec m_value;
};

template <>
struct FltCtrlOutput<FlightAxis::Thrust>{
   
   typedef quan::time_<int16_t>::us usec;

   void set(JoystickInput<FlightAxis::Thrust> const & in){ }

   void set(FltCtrlInput<FlightAxis::Thrust> const & in){ }

   void set( usec in) {}

   usec get() const {return m_value;}

  private:
      usec m_value;
};



#endif // AERFLITE_ARDUPILOT_FLIGHT_CONTROL_H_INCLUDED
