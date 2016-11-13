
#include <quan/stm32/millis.hpp>
#include "../i2c_driver/i2c_periph.hpp"
#include "../system/serial_port.hpp"
#include "../system/led.hpp"
#include "../system/delay.hpp"
#include "lis3_mdl.hpp"
#include <quan/three_d/vect.hpp>

using AP_HAL::millis;

namespace {
   constexpr quan::time_<uint32_t>::ms max_time{500U};
}

bool lis3_mdl_write_reg(uint8_t reg, uint8_t val)
{
   if (!Quan::wait_for_i2c_bus_free(quan::time::ms{100})) { return false;}

   if ( Quan::lis3mdl_onboard::write(reg, val)){
      return true;
   }else{
      serial_port::write("lis3mdl write reg failed\n");
      if(Quan::i2c_periph::has_errored()){
         Quan::i2c_periph::init();
      }
      return false;
   }
}

namespace {
   uint8_t * result_values = nullptr;
}

bool lis3_mdl_setup()
{

    /*
         set full scale +-4 Gauss             CTRL_REG2  &= ~ ( bit(FS1) |  bit(FS0) )
         set high perf ( lowest noise )       CTRL_REG1  |=   ( bit(OM1) |  bit(OM0) )
                                              CTRL_REG4  |=   ( bit(OMZ1) | bit(OMZ0) )
         set single measurement mode
             set FODR to 1                    CTRL_REG1  |=    bit(FODR)
             set single measurement           CTRL_REG3   = (CTRL_REG3 & ~(bit(MD1)) | bit(MD0))
         temperature sensor on
     */

  return
          lis3_mdl_write_reg( Quan::lis3mdl_onboard::reg::ctrlreg2, 0U )  &&
          lis3_mdl_write_reg( Quan::lis3mdl_onboard::reg::ctrlreg1, (0b1 << 7U) | (0b11 << 5U) | (0b1 << 1U))  &&
          lis3_mdl_write_reg( Quan::lis3mdl_onboard::reg::ctrlreg4, (0b11 << 2U) ) &&
          lis3_mdl_write_reg( Quan::lis3mdl_onboard::reg::ctrlreg3, (0b1 << 1U) ); // this puts it at idle
        
}

bool lis3mdl_start_single_measurement()
{
    return lis3_mdl_write_reg(Quan::lis3mdl_onboard::reg::ctrlreg3, 1U);
}

#if defined QUAN_I2C_DEBUG
void show_i2c_registers();
#endif


namespace {

  quan::three_d::vect<int> minvals{0,0,0};
  quan::three_d::vect<int> maxvals{0,0,0};
  quan::three_d::vect<float> offsets{0,0,0};
};

bool lis3mdl_read()
{
   if (!Quan::wait_for_i2c_bus_free(quan::time::ms{100})) { return false;}
   // or reg to read with 0x80 for multiple byte read
   bool result = Quan::lis3mdl_onboard::read(Quan::lis3mdl_onboard::reg::out_X_reg_L | 0x80 ,result_values,8);
   if ( !result ){
      serial_port::write("lis3mdl read failed\n");
      return false;
   }
   if (!Quan::wait_for_i2c_bus_free(quan::time::ms{100})) { 
#if defined QUAN_I2C_DEBUG
         show_i2c_registers();
#endif
      if(Quan::i2c_periph::has_errored()){
         Quan::i2c_periph::init();
      }
      return false;
   }
   quan::three_d::vect<int> vect;
   for ( uint32_t i = 0; i < 3; ++i){
      union {
         uint8_t ar[2];
         int16_t v;
      } u;
      u.ar[0] = result_values[ 2U * i];
      u.ar[1] = result_values[ 2U * i + 1];
      vect[i] = u.v;
      if ( vect[i] < minvals[i]){
         minvals[i] = vect[i];
      }
      if( vect[i] > maxvals[i]){
         maxvals[i] = vect[i];
      }
      offsets[i] =  (maxvals[i] + minvals[i])/2;
   }
   vect -= offsets;
   serial_port::printf<100>("result = [%6d,%6d,%6d]\r",vect.x,vect.y,vect.z);
   return true;
}

bool lis3_mdl_run()
{
   result_values = (uint8_t*)malloc(8);
   if ( lis3_mdl_setup() ){
      serial_port::write("lis3mdl setup ok\n");
      for (;;) {
         if (!lis3mdl_start_single_measurement()){
            return false;
         }
         Quan::delay(quan::time::ms{5});
         if ( ! lis3mdl_read()){
            return false;
         }
         Quan::delay(quan::time::ms{90});
      }
      return true;
   }else{
      return false;
   }
}

bool lis3_mdl_test()
{
   serial_port::write("lis3_mdl whoami test \n");
   if (!Quan::wait_for_i2c_bus_free(quan::time::ms{100})) { return false;}
   auto  start_time = millis();
   uint8_t whoami = 0;
   unsigned count = 0;
   for ( uint8_t i = 0; i < 5; ++i){
      if ( Quan::lis3mdl_onboard::read(Quan::lis3mdl_onboard::reg::whoami, &whoami, 1U)){
         // read function is non blocking, result is only available once bus has been released
         if (!Quan::wait_for_i2c_bus_free(quan::time::ms{100})) { 
            if(Quan::i2c_periph::has_errored()){
               Quan::i2c_periph::init();
            }
            return false;
         }
         if ( whoami == Quan::lis3mdl_onboard::val::whoami){
            serial_port::printf<100>("whoami test succesful: got %u\n",static_cast<unsigned>(whoami));
            ++count;
         }else{
            serial_port::printf<100>("whoami test failed: got %u\n",static_cast<unsigned>(whoami));
         }
      }else{
         serial_port::write("whoami compass test i2c read function failed: read failed\n");
         if(Quan::i2c_periph::has_errored()){
            Quan::i2c_periph::init();
         }
         return false;
      }
   }
   auto time_taken = (millis() - start_time).numeric_value();
   serial_port::printf<100>("count = %u, time taken = %u ms\n",count,static_cast<uint32_t>(time_taken));
   return (count > 0);
}
