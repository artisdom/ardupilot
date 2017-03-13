#include "JoystickInput.h"
#include <quan/constrain.hpp>

constexpr JoystickInput_base::usec JoystickInput_base::m_min;
constexpr JoystickInput_base::usec JoystickInput_base::m_max;
constexpr JoystickInput_base::usec JoystickInput_base::m_trim;

float JoystickInput_base::as_float()const 
{ 
   return quan::constrain(
      (this->m_value - get_trim()) / ((get_max() - get_min())/2.f)
      ,-1.f, 1.f
   );
}

void JoystickInput_base::update() 
{
  usec const rcin{hal.rcin->read(m_rcin_idx)};
  if ( input_sense_reversed()){
      m_value = (get_min() + get_max()) - rcin ;
  }else{
      m_value = rcin;
  }
}

JoystickInput_angle::cdeg JoystickInput_angle::as_angle() const
{
   return (( as_usec() - get_trim() ) * quan::angle_<int32_t>::cdeg{4500} ) 
      / ((get_max() - get_min())/2);
}

JoystickInput<FlightAxis::Thrust>::force_type 
JoystickInput<FlightAxis::Thrust>::as_force() const
{
   return (( as_usec() - get_min() ) * force_type{100} ) / (get_max() - get_min());
}
