
#include "FreeRTOS.h"
#include <task.h>
#include <AP_HAL/AP_HAL.h>
#include "i2c_task.hpp"
#include "bmp_280.hpp"

Quan::bmp280::calib_param_t Quan::bmp280::calib_param;

extern const AP_HAL::HAL& hal;

namespace {

   bool bmp_280_write_reg_blocking(uint8_t reg, uint8_t val)
   {
      if (!Quan::wait_for_i2c_bus_free(quan::time::ms{1000})) { return false;}
      if ( Quan::bmp280::write(reg, val)){
         if (!Quan::wait_for_i2c_bus_free(quan::time::ms{1000})) { return false;}
         return true;
      }else{
         hal.console->printf("bmp 280 write reg failed\n");
         if(Quan::i2c_periph::has_errored()){
            Quan::i2c_periph::init();
         }
         return false;
      }
   }

   bool bmp_280_read_regs_blocking(uint8_t reg,uint8_t * result, uint32_t len)
   {
      if (!Quan::wait_for_i2c_bus_free(quan::time::ms{1000})) { return false;}
      if ( Quan::bmp280::read(reg, result, len)){
         if (!Quan::wait_for_i2c_bus_free(quan::time::ms{1000})) { return false;}
         return true;
      }else{
         hal.console->printf("bmp 280 read reg failed\n");
         if(Quan::i2c_periph::has_errored()){
            Quan::i2c_periph::init();
         }
         return false;
      }
   }

   // blocking
   bool bmp_280_read_cal_params()
   {  
      bool result = bmp_280_read_regs_blocking(Quan::bmp280::reg::dig_T1,Quan::bmp280::calib_param.arr,24);
      if (result){
   #if 0
         hal.console->printf("--------- call_ params ----------\n");
         
         serial_port::printf<100>("dig_T1 = %u\n",Quan::bmp280::calib_param.dig_T1);
         serial_port::printf<100>("dig_T2 = %d\n",Quan::bmp280::calib_param.dig_T2);
         serial_port::printf<100>("dig_T3 = %d\n",Quan::bmp280::calib_param.dig_T3);
         serial_port::printf<100>("dig_P1 = %u\n",Quan::bmp280::calib_param.dig_P1);
         serial_port::printf<100>("dig_P2 = %d\n",Quan::bmp280::calib_param.dig_P2);
         serial_port::printf<100>("dig_P3 = %d\n",Quan::bmp280::calib_param.dig_P3);
         serial_port::printf<100>("dig_P4 = %d\n",Quan::bmp280::calib_param.dig_P4);
         serial_port::printf<100>("dig_P5 = %d\n",Quan::bmp280::calib_param.dig_P5);
         serial_port::printf<100>("dig_P6 = %d\n",Quan::bmp280::calib_param.dig_P6);
         serial_port::printf<100>("dig_P7 = %d\n",Quan::bmp280::calib_param.dig_P7);
         serial_port::printf<100>("dig_P8 = %d\n",Quan::bmp280::calib_param.dig_P8);
         serial_port::printf<100>("dig_P9 = %d\n",Quan::bmp280::calib_param.dig_P9);

         hal.console->printf("-------- ~call_ params ----------\n");
   #endif
      }else{
         hal.console->printf("failed to read cal params\n");
         if(Quan::i2c_periph::has_errored()){
            Quan::i2c_periph::init();
         }
      }
      return result;
   }

    // blocking
   bool bmp_280_setup()
   {
      Quan::bmp280::config_bits config;
      config.spi3w_en = false ; // not spi mode
      config.filter   = 0b100;  // filter coefficient x16
      config.t_sb     = 0b000;  // 0.5 ms standby

      Quan::bmp280::ctrl_meas_bits ctrl_meas;
      ctrl_meas.mode   = 0b000;     // forced
      ctrl_meas.osrs_p = 0b101;   // pressure oversampling  x16
      ctrl_meas.osrs_t = 0b010;   // temperature oversampling x2

      // N.B with these settings max update == 20 Hz
      return bmp_280_write_reg_blocking(Quan::bmp280::reg::config,config.value) &&
            bmp_280_write_reg_blocking(Quan::bmp280::reg::ctrl_meas,ctrl_meas.value) &&
            bmp_280_read_cal_params();
   }

   int32_t t_fine;

   // from the BMP280 datasheet
   int32_t bmp280_compensate_T_int32(int32_t adc_T)
   {
      int32_t var1 = ((((adc_T >> 3) - ((int32_t)Quan::bmp280::calib_param.dig_T1 << 1))) * ((int32_t) Quan::bmp280::calib_param.dig_T2)) >> 11; 
      int32_t var2 = (((((adc_T >> 4) - ((int32_t)Quan::bmp280::calib_param.dig_T1)) * ((adc_T >> 4) - ((int32_t)Quan::bmp280::calib_param.dig_T1))) >> 12) *
      ((int32_t)Quan::bmp280::calib_param.dig_T3)) >> 14;
      t_fine = var1 + var2;
      int32_t temperature = (t_fine * 5 +128) >> 8;
      return temperature;
   }

   // from the BMP280 datasheet
   // Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits).
   // Output value of “24674867” represents 24674867/256 = 96386.2 Pa = 963.862 hPa
   // T1 unsigned and P1 unsigned
   uint32_t bmp280_compensate_P_int64(int32_t adc_P)
   {
      int64_t var1, var2, p;
      var1 = ((int64_t)t_fine) - 128000;
      var2 = var1 * var1 * (int64_t)Quan::bmp280::calib_param.dig_P6;
      var2 = var2 + ((var1*(int64_t)Quan::bmp280::calib_param.dig_P5) << 17);
      var2 = var2 + (((int64_t)Quan::bmp280::calib_param.dig_P4) << 35);
      var1 = ((var1 * var1 * (int64_t)Quan::bmp280::calib_param.dig_P3) >> 8) + ((var1 * (int64_t)Quan::bmp280::calib_param.dig_P2) << 12);
      var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)Quan::bmp280::calib_param.dig_P1) >> 33;
      if (var1 == 0)
      {
          return 0; // avoid exception caused by division by zero
      }
      p = 1048576 - adc_P;
      p = (((p << 31) - var2) * 3125) / var1;
      var1 = (((int64_t) Quan::bmp280::calib_param.dig_P9 ) * (p >> 13) * (p >> 13)) >> 25;
      var2 = (((int64_t) Quan::bmp280::calib_param.dig_P8 ) * p) >> 19;
      p = ((p + var1 + var2) >> 8) + (((int64_t)Quan::bmp280::calib_param.dig_P7) << 4);
      return (uint32_t)p;
   }

   // --------------------------------------------------------

   void bmp280_calculate(Quan::detail::baro_args & result)
   {
      uint32_t const adc_T = (((uint32_t)result_values[3]) << 12U )    // msb
            |               (((uint32_t)result_values[4]) << 4U  )    // lsb
            |               (((uint32_t)result_values[5]) >> 4U  )    //xlsb
            ;   
      // temperature
      int32_t const temperature = bmp280_compensate_T_int32(adc_T);


      uint32_t const adc_P =  (((uint32_t)result_values[0]) << 12U )    // msb
            |   (((uint32_t)result_values[1]) << 4U  )    // lsb
            |   (((uint32_t)result_values[2]) >> 4U  )   //xlsb
            ;
     
      uint32_t const pressure = bmp280_compensate_P_int64(adc_P);

      result.pressure = quan::pressure_<float>::Pa{pressure/255.f};
      result.temperature = quan::temperature_<float>::K{temperature / 100.f + 273.15f};
      //serial_port::printf<100>("T = %f, P = %f\n",temperature/ 100.0, pressure/ 255.0);
   }


   bool bmp_280_start_conv()
   {
      Quan::bmp280::ctrl_meas_bits ctrl_meas;
      ctrl_meas.mode   = 0b001;   // forced
      ctrl_meas.osrs_p = 0b001;   // pressure oversampling  x1
      ctrl_meas.osrs_t = 0b001;   // temperature oversampling x1

      if ( bmp_280_write_reg(Quan::bmp280::reg::ctrl_meas,ctrl_meas.value)){
         // 3 regs, 330 usec
         vTaskDelay(1);
         return true;
      }else{
         hal.console->printf("bmp_280 start conv failed trying reset\n");
         if(Quan::i2c_periph::has_errored()){
            Quan::i2c_periph::init();
         }
         return false;
      }
   }

   bool bmp_280_start_read()
   {
      if ( bmp_280_read_regs(Quan::bmp280::reg::press_msb,result_values,6)){
         // 930 usec
         vTaskDelay(2);
         return true;
      }else{
         hal.console->printf("bmp_280 read failed trying reset\n");
         if(Quan::i2c_periph::has_errored()){
            Quan::i2c_periph::init();
         }
         return false;
      }
   }

}

namespace Quan{

   bool setup_baro()
   {
      return bmp_280_setup();     
   }
   
   bool baro_request_conversion()
   {
      if (! i2c_periph::bus_released() ){
         return false;
      }
      return bmp280_request_conversion();
   }

   // must be > 5 ms after request_conversion finishes
   bool baro_start_read()
   {
      if (! i2c_periph::bus_released() ){
         return false;
      }
      return bmp280_start_read();
   }

   void baro_calculate(Quan::detail::baro_args & result)
   {
      bmp280_calculate(result);
   }
   
}