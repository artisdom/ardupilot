#ifndef AERFLITE_RANGE_INPUT_H_INCLUDED
#define AERFLITE_RANGE_INPUT_H_INCLUDED

#include "JoystickInput.h"
#include <quan/acceleration.hpp>

struct RangeInput : JoystickInput{
   
   RangeInput(uint8_t ch_in) : JoystickInput{ch_in,get_min()}{}
   int32_t get_thrust() const
   {
      return (( get() - get_min() ) * 100 ) / (get_max() - get_min());
   }
   
};

#endif // AERFLITE_ANGLE_INPUT_H_INCLUDED
