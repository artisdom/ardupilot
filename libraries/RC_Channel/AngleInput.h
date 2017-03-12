#ifndef AERFLITE_ANGLE_INPUT_H_INCLUDED
#define AERFLITE_ANGLE_INPUT_H_INCLUDED

#include "JoystickInput.h"
#include <quan/angle.hpp>

struct AngleInput : JoystickInput{
   
   AngleInput(uint8_t ch_in) : JoystickInput{ch_in,get_trim()}{}
   typedef quan::angle_<int16_t>::cdeg cdeg;

   cdeg get_angle() const
   {
      return (( get() - get_trim() ) * quan::angle_<int32_t>::cdeg{4500} ) 
         / ((get_max() - get_min())/2);
   }
   
};

#endif // AERFLITE_ANGLE_INPUT_H_INCLUDED
