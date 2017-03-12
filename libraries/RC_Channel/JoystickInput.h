#ifndef AERFLITE_JOYSTICK_INPUT_HPP_INCLUDED
#define AERFLITE_JOYSTICK_INPUT_HPP_INCLUDED

#include "JoystickInput_base.h"
#include "FlightAxes.h"
#include <quan/angle.hpp>
#include <quan/force.hpp>


template <typename FlightAxisT>
struct JoystickInput : JoystickInput_base{
   
   JoystickInput(uint8_t ch_in) : JoystickInput_base{ch_in,get_trim()}{}
   typedef quan::angle_<int16_t>::cdeg cdeg;

   cdeg eval() const
   {
      return (( get() - get_trim() ) * quan::angle_<int32_t>::cdeg{4500} ) 
         / ((get_max() - get_min())/2);
   }
};

template <>
struct JoystickInput<FlightAxis::Thrust> : JoystickInput_base{

   JoystickInput(uint8_t ch_in) : JoystickInput_base{ch_in,get_min()}{}
   // use this for type safety reasons
   typedef quan::force_<int16_t>::N force_type;
   force_type eval() const
   {
      return (( get() - get_min() ) * force_type{100} ) / (get_max() - get_min());
   }
   
};

#endif // AERFLITE_JOYSTICK_INPUT_HPP_INCLUDED
