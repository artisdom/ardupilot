
#include <stdarg.h>
#include <malloc.h>
#include <cstring>
#include <cstdio>
#include <quan/stm32/millis.hpp>
#include "../system/serial_port.hpp"
#include "../system/led.hpp"

#include "eeprom.hpp"

char const data[] =

#if 0
"Hello 123456789 ABCDEFGHIJKLMNOPQRSTUVWXYZ";
#else
"Simple Mixer \n"
"# anything global must be a constant ...\n"
"# type is deduced from the initialiser expression\n"
"# There are 3 types integer, float bool\n"
"# floats are differentiated by including a decimal point and fractional part\n"
"# bools are either true or false\n"
"# There is NO conversion between the different types\n"
"#  x = 1 ; // x is an integer\n"
"#  y = 1.0 ; // y is a float\n"
"#  z1 = x + x ; // OK z1 is an integer\n"
"#  z2 = x + y ; // Error : x and y are different types\n"
"#  b = true;   // b is a bool\n"
"#  c = b && x ;   // Error :  b and x are different types\n"
"#  d = b && !b ; // ok ( c is false)\n"
"#  w = x != 0  ; // w is bool \n"
"\n"
"pitch_gain = 500.0; # pitch gain surprisingly ( Just here to test comments)\n"
"roll_gain = 1000.0 - pitch_gain;\n"
"throttle_gain = 10.0;\n"
"pulse_offset = 1500.0;\n"
"thrt_pls_offset = 1000.0;\n"
"\n"
"# Provide names for some of the output array indices\n"
"elevon_left = 1;\n"
"elevon_right = 2;\n"
"\n"
"# An output in the mixer function is evaluated periodically\n"
"# (Possibly when the inputs connected to it change)\n"
"mixer()\n"
"{\n"
"   # in the mixer function expression can be variables as wel as constants\n"
"\n"
"   F_deflection = (input(Airspeed) * input(Airspeed))/( input(ARSPD_FBWA_MIN) * input(ARSPD_FBWA_MIN)) ;\n"
"   pitch = input(Pitch) * pitch_gain;\n"
"   roll  = input(Roll) * roll_gain;\n"
"\n"
"   # The output expressions\n"
"   output[elevon_left] = ( pitch + roll) / F_deflection + pulse_offset;\n"
"   output[elevon_right] = (pitch - roll) / F_deflection + pulse_offset;\n"
"\n"
"   # just to show literals work also for outputs\n"
"   output[3] = input(Throttle) * throttle_gain + thrt_pls_offset;\n"
"}\n"
"\n";

#endif

using quan::stm32::millis;

namespace {

   typedef Quan::i2c_eeprom_driver<Quan::eeprom_info::eeprom_24lc128> eeprom;
}

bool eeprom_test()
{

   // Can happen if devices active on bus
   if ( Quan::i2c_periph::is_busy()) {
      serial_port::write("i2c bus busy at startup\n");
      while (1) {asm volatile("nop":::);}
   }

   uint32_t const address = 120;
   uint32_t const len = strlen(data) +1;

   serial_port::printf<30>("length is %d\n",len);
   if ( eeprom::write(address,(uint8_t const*) data,len)){

      serial_port::write("write succeeded\n");
      auto now = millis();
      // n.b must be done exactly this way if accessing eeprom
      while ( ! Quan::i2c_periph::bus_free() || eeprom::write_in_progress()){
         if ( (millis() - now) > quan::time::ms{1000}){
            serial_port::write("big fail\n");
            if( ! Quan::i2c_periph::bus_free()){
               serial_port::write("bus didnt free\n");
            }
            if(eeprom::write_in_progress()){
               serial_port::write("write in progress didnt finish\n");
            }
            return false;
         }
      }
      serial_port::write("starting read...\n\n");
      char * result = (char*)malloc(len +1);
      if (result){
         if (eeprom::read(address,(uint8_t*) result,len)){
            serial_port::write("read succeeded\n");
            // after the read function returns
            // we need to wait until the read has completed,
            // signified by the bus being released.
            // obviously only for single threaded!
            auto now = quan::stm32::millis();
            while(!Quan::i2c_periph::bus_released() && ((quan::stm32::millis() - now) < quan::time::ms{1000U})){;}

            if (Quan::i2c_periph::bus_released()){
               serial_port::write(result,len);
               serial_port::write("\n");
            }else{
               serial_port::write("read failed, bus not released\n");
               return false;
            }
            serial_port::write("done\n");
            free (result); result = nullptr;
         }else{
            serial_port::write("read failed\n");
            return false;
         }
      }else{
         serial_port::write("malloc failed\n");
         return false;
      }
   }
   return true;
}
