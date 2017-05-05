#include "Plane.h"

extern const AP_HAL::HAL& hal;

namespace {
   void output_action(uint8_t channel);
}

#if (CONFIG_HAL_BOARD == HAL_BOARD_SITL)
#error wtf
   #include "mixers/sitl_mixer.cpp"
#else
   #if (1)
      #include "mixers/disco_mixer.cpp"
   #else
      #include "mixers/falcon_mixer.cpp"
   #endif
#endif

namespace {
   void output_action(uint8_t channel)
   {
      float const v1 = (output[channel] + 3.f) * 500.f;
      uint16_t const out = quan::constrain(
         static_cast<uint16_t>(v1),
         static_cast<uint16_t>(1000U),
         static_cast<uint16_t>(2000U)
      ); 
      hal.rcout->write(channel,out);
   }
}

bool Plane::create_mixer()
{
   return true;
}

void Plane::mix()
{
    mixer_eval();
}
