/*
 *       Example of APM_Compass library (HMC5843 sensor).
 *       Code by Jordi MuÒoz and Jose Julio. DIYDrones.com
 */

#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_QUAN
#include <AP_Compass/AP_Compass.h>
#include <AP_HAL_Quan/AP_HAL_Quan_Test_Main.h>
#include <AP_HAL_Quan/Storage.h>
#include <quantracker/osd/osd.hpp>
#include <quan/min.hpp>
#include <quan/max.hpp>
#include <quan/conversion/parse_number.hpp>
#include <AP_OSD/fonts.hpp>
#include <cstdio>
#include <cstring>
#include <cctype>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();
using AP_HAL::millis;

namespace {

   Compass compass;

   AP_HAL::UARTDriver * uart = nullptr;

   quan::uav::osd::pxp_type ar[4];

   // wher the structure is stored
   uint32_t compass_params_eeprom_address = 0x10100;

   const char magic_value[5] = "quan";
   uint32_t magic_eeprom_address= 0x10000;

   struct compass_params_t{
      compass_params_t():offset{0.f,0.f,0.f},gain{1.f,1.f,1.f}{}
      Vector3f offset;
      Vector3f gain;
   };

   compass_params_t compass_params;

   // 0 on not inited,1 on success, -1 on ee read fail
   int read_magic()
   {
      const char magic[5]= {0,0,0,0,0};
      if (! Quan::storage_read((uint8_t*)magic,magic_eeprom_address,5)){
         uart->printf("read eeprom magic failed\n");
          return -1;
      }
      bool result = strncmp(magic,"quan",5)==0;
      if (result){
         uart->printf("magic found\n");
         return 1;
      }else{
         uart->printf("no magic found\n");
         return 0;
      }
   }

   void write_magic()
   {
      if (! Quan::storage_write(magic_eeprom_address,(uint8_t const*)"quan",5)){
         AP_HAL::panic("write eeprom magic failed\n");
      }
   }

   void read_compass_params(compass_params_t& c)
   {
      if (!Quan::storage_read((uint8_t*)&c,compass_params_eeprom_address,sizeof(compass_params_t))){
         AP_HAL::panic("read compass params failed\n");
      }
   }

   void write_compass_params(compass_params_t const & c)
   {
      if (!Quan::storage_write(compass_params_eeprom_address,(uint8_t const*)&c,sizeof(compass_params_t))){
         AP_HAL::panic("write compass params failed\n");
      }
   }

   void print_float (Vector3f & v)
   {
      uart->printf("[% 8.2f,% 8.2f,% 8.2f]",
        static_cast<double>(v.x)
        ,static_cast<double>(v.y)
        ,static_cast<double>(v.z)
      );
   }

   void view_params()
   {
      uart->printf("offsets = ");
      print_float(compass_params.offset);
      uart->printf("\ngains   = ");
      print_float(compass_params.gain); 
      uart->printf("\n"); 
   }

}

void setup() {
    // we output data on telemetry uart
    // which is connected to the RF modem.
    uart = hal.console;
   // uart->begin(57600);

    hal.console->println("Compass library test 1\n");
 //hal.scheduler->delay(1000);

    int magic_exists = read_magic();
    if (magic_exists == -1){
      AP_HAL::panic("read magic failed\n");
    }
    if ( magic_exists == 0){
        uart->printf("initialising compass params to default\n");
        write_magic();
        write_compass_params(compass_params);
    }
    if (magic_exists ==1){
        uart->printf("reading params\n");
        read_compass_params(compass_params);
    }
  
    uart->printf("compass params\n");
    view_params();
      
    if (!compass.init()) {
        AP_HAL::panic("compass initialisation failed!");
    }
  //  uart->printf("init done - %u compasses detected\n", compass.get_count());

    // mod this to your own offsets

    compass.set_offsets(compass.get_primary(),compass_params.offset);
    compass.set_declination(ToRad(0.0f)); // set local difference between magnetic north and true north

    hal.scheduler->delay(1000);

}

namespace {

   Vector3f raw_field;
}

void quan::uav::osd::on_draw() 
{ 
   pxp_type pos{-140,100};
   char buf[100];
   vTaskSuspendAll();
   sprintf(buf,"[% 6.2f,% 6.2f, %6.2f]"
      ,static_cast<double>(raw_field.x)
      ,static_cast<double>(raw_field.y)
      ,static_cast<double>(raw_field.z)
   ); 
   xTaskResumeAll();    
   draw_text(buf,pos,Quan::FontID::MWOSD);

}

namespace {

   int parse_channel()
   {
      for(;;){
         uart->printf("enter x or y or z \r\n");
         while( !uart->available() ) {
            hal.scheduler->delay(20);
         }
         char user_input = uart->read();
         switch(user_input){
         case 'x':
             uart->printf("x entered\n");
            return 0;
         case 'y':
            uart->printf("y entered\n");
            return 1;
         case 'z':
            uart->printf("z entered\n");
            return 2;
         default:
            uart->printf("Error : invalid input\\nn");
            break;
         }
      }
   }

   float parse_number()
   {    
      for (;;){
         uart->printf("enter number\n");
         constexpr uint32_t buflen = 100;
         char buffer[buflen] = {'\0'};
         uint32_t idx = 0U;
         for (;;){
             while( !uart->available() ) {
                hal.scheduler->delay(20);
             }
             char ch = uart->read();
             if ( ch == '\r'){
               if ( idx > 0){
                  buffer[idx] = '\0';
                  uart->printf("\r\n");
                  break;
               }else{
                  uart->printf("no number, restarting parse\n\n");
                  idx = 0U;
               }
             }
             if ( idx < (buflen-1)){
               buffer[idx++] = ch;
               uart->printf("%c",ch);
             }else{
                uart->printf("number too long, restarting parse\n\n");
                idx = 0U;
             }
         }
         typedef quan::detail::number_parser parse_t;
         parse_t parse;
         double d_res;
         int64_t i_res;
         parse_t::num_type num = parse(buffer,&d_res,&i_res,buflen);
         switch( num){
            case parse_t::num_type::FLOAT:
               uart->printf("%f enterd\n",d_res);
               return d_res;
            case parse_t::num_type::INT:
               uart->printf("%d enterd\n",static_cast<int>(i_res));
               return i_res;
            default:
               uart->printf("failed to recognise number, restarting parse\n\n");
               break;
         }
      }
   }
  
   void do_offset()
   {
      int idx= parse_channel();
      float number = parse_number();
      compass_params.offset[idx] = number;
      uart->printf("offsets = ");
      print_float(compass_params.offset);
      
   }

   void do_gain()
   {
      int idx= parse_channel();
      float number = parse_number();
      compass_params.gain[idx] = number;
      quan::three_d::vect<float> g{0.f,0.f,0.f};
      g[idx] = number;
      Quan::set_gains(g);
      uart->printf("gains = ");
      print_float(compass_params.gain);
   }

   void save_to_eeprom()
   {
      write_compass_params(compass_params);
      Quan::wait_for_eeprom_write_queue_flushed();
      uart->printf("params written to eeprom");
   }

   void print_menu()
   {
         while( uart->available() ) {
          uart->read();
         } 
          uart->printf(
         "Menu:\r\n"
         "    V) view gains and offsets\r\n"
         "    O) set offset\r\n"
         "    G) set gain \r\n"
         "    S) save to eeprom\r\n");

         while( !uart->available() ) {
          hal.scheduler->delay(20);
         }

   }
   
   void display_options()
   {
      print_menu();
      char user_input = uart->read();

      switch (toupper(user_input)){
         case 'V':
         view_params();
         break;
         case 'O':
         do_offset();
         break;
         case 'G':
         do_gain();
         break;
         case 'S':
         save_to_eeprom();
         break;
         default:
         uart->printf("unknown input\n");
         break;
      }

      while( uart->available() ) {
         uart->read();
      }
  }
}

void loop()
{
   hal.scheduler->delay(100);

   compass.read();

   raw_field = compass.get_raw_field();

   if (uart->available()){
      display_options();
   }

}

namespace {
   uint32_t get_flags()
   {
      HAL_Quan::start_flags flags{0};
      flags.init_gpio = true;
      flags.init_scheduler = true;
      flags.init_uartA = true;
      flags.init_uartB = true;
      flags.init_i2c = true;
      return flags.value;
   }
}

AP_HAL_TEST_MAIN( get_flags() )

#endif // CONFIG_HAL_BOARD == HAL_BOARD_QUAN

