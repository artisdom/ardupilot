
#include "Plane.h"
#include <mixer_lang.hpp>
#include <mixer_lang_cstrstream.hpp>

extern const AP_HAL::HAL& hal;

namespace {

   // This value simulates the airspeed sensor reading
   // TODO make a way to vary it when running for sim ... prob
   // more suitable for a GUI version
   apm_mix::float_t airspeed_m_per_s = 0.0; //

   // true simulates a possible sensor failure 
   // which may mean (for example) airspeed reading is no good
   bool in_failsafe = false;  //
   
  // apm_mix::float_t get_airspeed(){ return plane.get_airspeed();}
   bool failsafe_on() { return in_failsafe;}

   apm_mix::float_t dummy() { return 0.f;}

   apm_mix::input_pair inputs[] = { 
      apm_mix::input_pair{"Pitch", static_cast<apm_mix::float_t(*)()>([]()->apm_mix::float_t{return plane.get_pitch_demand();})},
      apm_mix::input_pair{"Yaw",  static_cast<apm_mix::float_t(*)()>([]()->apm_mix::float_t{return plane.get_yaw_demand();})},
      apm_mix::input_pair{"Roll", static_cast<apm_mix::float_t(*)()>([]()->apm_mix::float_t{return plane.get_roll_demand();})},
      apm_mix::input_pair{"Throttle", static_cast<apm_mix::float_t(*)()>([]()->apm_mix::float_t{return plane.get_thrust_demand();})},
      apm_mix::input_pair{"Flap", dummy},
      apm_mix::input_pair{"Airspeed", static_cast<apm_mix::float_t(*)()>([]()->apm_mix::float_t{return plane.get_airspeed();})},
      apm_mix::input_pair{"ControlMode",dummy},

      apm_mix::input_pair{"ARSPD_MIN", static_cast<apm_mix::float_t(*)()>([]()->apm_mix::float_t{return 10.0;})},
      apm_mix::input_pair{"ARSPD_CRUISE",static_cast<apm_mix::float_t(*)()>([]()->apm_mix::float_t{return 12.0;})},
      apm_mix::input_pair{"ARSPD_MAX", static_cast<apm_mix::float_t(*)()>([]()->apm_mix::float_t{return 20.0;})},
      apm_mix::input_pair{"FAILSAFE_ON", failsafe_on},
   };

// convert the -1 to +1 value to a pwm output
   template<unsigned N>
   void output_action(apm_mix::float_t const & v)
   {
       float const v1 = (((v + 1.f)/2.f) + 1.f) * 1000.f;
       uint16_t const out = quan::constrain(static_cast<uint16_t>(v1),static_cast<uint16_t>(1000U),static_cast<uint16_t>(2000U)); 
       hal.rcout->write(N,out);
   }

   // Outputs are passed as an array to the mixer constructor
   // only float are allowed here
   apm_mix::abc_expr* outputs[] = {
       new apm_mix::output<apm_mix::float_t>{output_action<0>}
     , new apm_mix::output<apm_mix::float_t>{output_action<1>}
     , new apm_mix::output<apm_mix::float_t>{output_action<2>}
     , new apm_mix::output<apm_mix::float_t>{output_action<3>}
     , new apm_mix::output<apm_mix::float_t>{output_action<4>}
     , new apm_mix::output<apm_mix::float_t>{output_action<5>}
     , new apm_mix::output<apm_mix::float_t>{output_action<6>}
     , new apm_mix::output<apm_mix::float_t>{output_action<7>}
   };

   const char mixer_string [] = 
      "throttle_sign = -1.0;\n"
      "roll_sign=-1.0;\n"
      "pitch_sign = -1.0;\n"
      "yaw_sign = -1.0;\n"
      "\n"
      "mixer(){\n"
      "  output[0] = input{Roll} * roll_sign;\n"
      "  output[1] = input{Pitch} * pitch_sign;\n"
      "  output[2] = input{Throttle} * throttle_sign;\n"
      "  output[3] = input{Yaw} * yaw_sign;\n"
      "}\n";
}

void delete_outputs()
{
   for ( auto* p : outputs){
      delete p;
   }
}

bool apm_mix::yyerror(const char* str )
{
   hal.console->printf( "line %i , error : %s\n", apm_lexer::get_line_number(),str);
   return false;
}

bool Plane::create_mixer()
{

   auto* pstream = new apm_lexer::cstrstream_t{mixer_string,500}; // new apm_lexer::filestream_t{argv[1]};

   return ( apm_mix::mixer_create(
          pstream
         ,inputs, sizeof(inputs)/sizeof(inputs[0])
         ,outputs, sizeof(outputs)/sizeof(outputs[0])
      )); //{

}



void Plane::mix()
{
    apm_mix::mixer_eval();
}