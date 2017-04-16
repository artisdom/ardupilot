

#if 0

#include <cstring>
#include <mixer_lang.hpp>
#include <mixer_lang_cstrstream.hpp>
#include "Plane.h"

extern const AP_HAL::HAL& hal;



//float get_thrust_demand();
// avoid c style functions that use malloc
// TODO allocate a large block once, store the strings there
// and then free the entire block once when done
// to avoid small bits and pieces
// maybe count allocs and free when all deallocated
// freeing more is then  an error
char * apm_mix::duplicate_string(const char *str)
{
    if (str == nullptr){
       hal.console->printf("duplicate_string() : arg is nullptr\n");
       for(;;){ asm volatile("nop":::);}
    }
    // could use strnlen_s but n/a on gnu-arm-none-eabi afaics
    uint32_t alloc_length = 0U;
    uint32_t constexpr maxlen = 100U; // TODO can get from mixer_lang lib
    for ( uint32_t i = 0U; i < maxlen; ++i){
      if (str[i] == '\0'){
         alloc_length = i + 1U;
         break;
      }
    }
    if ( alloc_length == 0U){
       hal.console->printf("duplicate_string() : string too long\n");
       for(;;){ asm volatile("nop":::);}
    }
    char* dupstr = new char[alloc_length];
    if ( dupstr == nullptr){
       hal.console->printf("duplicate_string() : out of memory\n");
       for(;;){ asm volatile("nop":::);}
    }
    return strcpy(dupstr,str);
}

void apm_mix::delete_string(const char* s)
{
   delete [] s;
}

bool apm_mix::yyerror(const char* str )
{
   hal.console->printf("line %i , error : %s\n", apm_lexer::get_line_number(),str);
   return false;
}



bool in_rtl_mode();
namespace {

   // true simulates a possible sensor failure 
   // which may mean (for example) airspeed reading is no good
   bool in_failsafe = false;  //
   
   // TODO
   bool failsafe_on() { return in_failsafe;}

   apm_mix::float_t dummy() { return 0.f;}

   apm_mix::input_pair 
   inputs[] = { 
       {"Pitch", static_cast<apm_mix::float_t(*)()>([]()->apm_mix::float_t{return plane.get_pitch_demand();})}
      ,{"Yaw",  static_cast<apm_mix::float_t(*)()>([]()->apm_mix::float_t{return plane.get_yaw_demand();})}
      ,{"Roll", static_cast<apm_mix::float_t(*)()>([]()->apm_mix::float_t{return plane.get_roll_demand();})}
      ,{"Throttle",static_cast<apm_mix::float_t(*)()>([]()->apm_mix::float_t{return plane.get_thrust_demand();})} 
      ,{"Flap", static_cast<apm_mix::float_t(*)()>([]()->apm_mix::float_t{return plane.get_flap_demand();})}

   };

// convert the -1 to +1 value to a pwm output
   template<unsigned N>
   void output_action(apm_mix::float_t const & v)
   {
       float const v1 = (((v + 1.f)/2.f) + 1.f) * 1000.f;
       uint16_t const out = quan::constrain(static_cast<uint16_t>(v1),static_cast<uint16_t>(1000U),static_cast<uint16_t>(2000U)); 
       hal.rcout->write(N,out);
//       if ( in_rtl_mode()){
//         hal.console->printf("v %d in = %f, out = %d\n",static_cast<int>(N),static_cast<double>(v), out);
//       }
   }

//   template<>
//   void output_action<2>(apm_mix::float_t const & v)
//   {
//       float const v1 = (((v + 1.f)/2.f) + 1.f) * 1000.f;
//       uint16_t const out = quan::constrain(static_cast<uint16_t>(v1),static_cast<uint16_t>(1000U),static_cast<uint16_t>(2000U)); 
//       hal.rcout->write(2,out);
//       if ( in_rtl_mode()){
//         hal.console->printf("throt in = %f, out = %d\n",static_cast<double>(v), out);
//       }
//   }

   // Outputs are passed as an array to the mixer constructor
   // only float are allowed here
   apm_mix::output<apm_mix::float_t> 
   outputs[] = {
      {output_action<0>}
     ,{output_action<1>}
     ,{output_action<2>}
     ,{output_action<3>}
     ,{output_action<4>}
     ,{output_action<5>}
     ,{output_action<6>}
     ,{output_action<7>}
   };

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
 const char mixer_string [] = 

      "mixer(){\n"
      "   output[0] = input{Roll} ;\n"
      "   output[1] = input{Pitch} * -1.0 ;\n"
      "   output[2] = input{Throttle};\n"
      "   output[3] = input{Yaw} ;\n"
      "}\n"; 
#else
   const char mixer_string [] = 
   #if 1
      #include "skyhook_disco.mixstr"
   #else
         "roll_gain = -0.5;\n"
         "pitch_gain = -0.5;\n"
         "\n"
         "mixer(){\n"
         "roll = input{Roll} * roll_gain;\n"
         "pitch = input{Pitch} * pitch_gain;\n"
         "output[0] = roll + pitch;\n"
         "output[1] = roll - pitch;\n"
         "output[2] = input{Throttle};\n"
         "}\n"; 
   #endif
#endif
}


bool Plane::create_mixer()
{
   return true;
//   apm_lexer::cstrstream_t stream{mixer_string,500}; 
//
//   return apm_mix::mixer_create(
//      &stream
//      ,inputs, sizeof(inputs)/sizeof(inputs[0])
//      ,outputs, sizeof(outputs)/sizeof(outputs[0])
//   ); 
}

void Plane::mix()
{
  //  apm_mix::mixer_eval();
    mixer_eval();
}

#endif

