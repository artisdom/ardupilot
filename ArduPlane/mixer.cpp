
#include "Plane.h"
#include <mixer_lang.hpp>
#include <mixer_lang_cstrstream.hpp>

namespace {


   

   // This value simulates the airspeed sensor reading
   // TODO make a way to vary it when running for sim ... prob
   // more suitable for a GUI version
   apm_mix::float_t airspeed_m_per_s = 0.0; //

   // true simulates a possible sensor failure 
   // which may mean (for example) airspeed reading is no good
   bool in_failsafe = false;  //
   
   // The mixer uses functions pointers  to get its inputs
   // The inputs can be of type Bool, Integer or Float 
   apm_mix::float_t get_airspeed(){ return airspeed_m_per_s;}
   bool failsafe_on() { return in_failsafe;}

   apm_mix::float_t dummy() { return 0.f;}

   // The input array represents all the available inputs.
   // The inputs are passed to the mixer constructor
   // TODO make inputs optionally constant
   // TODO add integer index inputs option
   // if the function is are unambiguous, the type of the input is deduced from the signature of the function
   // the function signatures below are declared to return a float type in joystick.hpp
   // you can also use anexplicit cast as in the lambda functions
   // c++ lambda functions can also be used as shown in the array
   // you can use different input types as shown by the bool and int inputs at the end of the array
   apm_mix::input_pair inputs[] = { 

      apm_mix::input_pair{"Pitch", dummy},
      apm_mix::input_pair{"Yaw",  dummy},
      apm_mix::input_pair{"Roll", dummy},
      apm_mix::input_pair{"Throttle", dummy},
      apm_mix::input_pair{"Flap", dummy},
      apm_mix::input_pair{"Airspeed", get_airspeed},
      apm_mix::input_pair{"ControlMode",dummy},

      apm_mix::input_pair{"ARSPD_MIN", static_cast<apm_mix::float_t(*)()>([]()->apm_mix::float_t{return 10.0;})},
      apm_mix::input_pair{"ARSPD_CRUISE",static_cast<apm_mix::float_t(*)()>([]()->apm_mix::float_t{return 12.0;})},
      apm_mix::input_pair{"ARSPD_MAX", static_cast<apm_mix::float_t(*)()>([]()->apm_mix::float_t{return 20.0;})},
      apm_mix::input_pair{"FAILSAFE_ON", failsafe_on},
      apm_mix::input_pair{"DUMMY_INT", static_cast<apm_mix::int_t(*)()>([]()->apm_mix::int_t{return 1000;})}
   };

   // The mixers also uses function pointers to send its outputs.
   // outputs can also be of type Bool, Integer or Float
   // Here for simplicity, we just pass an output function 'output_action'
   // that prints the output to stdout
   // The non type template parameter is handy to provide the index
   template<unsigned N>
   void output_action(apm_mix::float_t const & v)
   {
      if ( v == static_cast<apm_mix::float_t>(0)){
        // printf("output[%u] = % 6.3f\n",N,static_cast<double>(v));
      }else{
        // printf("output[%u] = %+6.3f\n",N,static_cast<double>(v));
      }
   }

   // to show that int_t can be assigned as an output function also
   template<unsigned N>
   void output_action(apm_mix::int_t const & v)
   {
      //printf("output[%u] = % 6d\n",N, static_cast<int>(v));
   }

   // to show that bool can be assigned as an output function also
   template<unsigned N>
   void output_action(bool const & v)
   {
     // const char * const true_ = "true";
     // const char* const false_ = "false";
     // printf("output[%u] = %s\n",N, (v? true_:false_));
   }

   // Outputs are passed as an array to the mixer constructor
   // Outputs can output a type of Integer, Bool or Float
   // you can mix types in one mixer as shown here where 
   // output 9 is an int and output 10 is a bool
   apm_mix::abc_expr* outputs[] = {
       new apm_mix::output<apm_mix::float_t>{output_action<0>}
     , new apm_mix::output<apm_mix::float_t>{output_action<1>}
     , new apm_mix::output<apm_mix::float_t>{output_action<2>}
     , new apm_mix::output<apm_mix::float_t>{output_action<3>}
     , new apm_mix::output<apm_mix::float_t>{output_action<4>}
     , new apm_mix::output<apm_mix::float_t>{output_action<5>}
     , new apm_mix::output<apm_mix::float_t>{output_action<6>}
     , new apm_mix::output<apm_mix::float_t>{output_action<7>}
     , new apm_mix::output<apm_mix::float_t>{output_action<8>}
     , new apm_mix::output<apm_mix::int_t>{output_action<9>}  
     , new apm_mix::output<bool>{output_action<10>}
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



bool Plane::create_mixer()
{
//int main(int argc , char* argv[])
//{
//   if ( argc < 2){
//      printf("Useage : %s <mixer_filename>\n",argv[0]);
//      return EXIT_SUCCESS;
//   }

   // create the mixer from a filestream
   auto* pstream = new apm_lexer::cstrstream_t{mixer_string,500}; // new apm_lexer::filestream_t{argv[1]};

   return ( apm_mix::mixer_create(
          pstream
         ,inputs, sizeof(inputs)/sizeof(inputs[0])
         ,outputs, sizeof(outputs)/sizeof(outputs[0])
      )); //{
//      printf ("\n\n#########################################################\n");
//      printf     ("#                                                       #\n");
//      printf     ("#              Welcome to mixer_lang                    #\n");
//      printf     ("#                                                       #\n");
//      printf     ("#########################################################\n\n");
//      printf("mixer \"%s\" was created ...  OK!\n\n",argv[1]);
 //  }else{
   //   printf("create mixer from %s failed ... quitting\n",argv[1]);
  //    delete pstream;
  //    delete_outputs();
    //  return EXIT_FAILURE;
 //  }

  // printf("Switch on your Taranis and once on, then connect it to your PC via USB\n\n");
  // printf("Then when ready,press any key to start and once running press any key to quit\n\n");
  // fflush(stdin);
  // while (! key_was_pressed()){;}
  // getchar(); // clear key pressed
  //auto result = EXIT_FAILURE; // main return value
  // if ( open_joystick("/dev/input/js0")){
   //   while (get_joystick()->is_running() && ! key_was_pressed()){
    ///     sleep_ms(20);
         
   //   }
   //   close_joystick();
    //  result = EXIT_SUCCESS;
   //}
  // apm_mix::close_mixer();
  // delete pstream;
  // delete_outputs(); 
  // return result 
}



void Plane::mix()
{
    apm_mix::mixer_eval();
}