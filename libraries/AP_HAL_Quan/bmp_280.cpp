
#include "FreeRTOS.h"
#include <task.h>
#include <AP_HAL/AP_HAL.h>
#include "i2c_task.hpp"
#include "bmp_280.hpp"

Quan::bmp280::calib_param_t __attribute__ ((section (".dma_memory"))) Quan::bmp280::calib_param;

extern const AP_HAL::HAL& hal;

namespace {

   bool bmp_280_write_reg(uint8_t reg, uint8_t val)
   {
      if ( Quan::bmp280::write(reg, val)){
        return true;
      }else{
         hal.console->printf("bmp 280 write reg failed\n");
      }
      if(Quan::i2c_periph::has_errored()){
         hal.console->printf("re init i2c\n");
         Quan::i2c_periph::init();
      }
      return false;
   }

   bool bmp_280_read_regs(uint8_t reg,uint8_t * result, uint32_t len)
   {
      if ( Quan::bmp280::read(reg, result, len)){
        return true;
      }else{
         hal.console->printf("bmp 280 read reg failed\n");
      }

      if(Quan::i2c_periph::has_errored()){
         hal.console->printf("re init i2c\n");
         Quan::i2c_periph::init();
      }
      return false;
   }

   // blocking
   bool bmp_280_read_cal_params()
   {  
      bool result = bmp_280_read_regs(Quan::bmp280::reg::dig_T1,Quan::bmp280::calib_param.arr,24);
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
            hal.console->printf("re init i2c\n");
            Quan::i2c_periph::init();
         }
      }
      return result;
   }

   bool bmp_280_setup()
   {
      Quan::bmp280::config_bits config;
      config.spi3w_en = false ; // not spi mode
      config.filter   = 0b100;  // filter coefficient x16
      config.t_sb     = 0b000;  // 0.5 ms standby

      // N.B with these settings max update == 20 Hz
      if (! bmp_280_write_reg(Quan::bmp280::reg::config,config.value)){
          hal.console->printf("bmp_280 write config failed\n");
          return false;
      }

      Quan::bmp280::ctrl_meas_bits ctrl_meas;
      ctrl_meas.mode   = 0b000;    // forced
      ctrl_meas.osrs_p = 0b101;   // pressure oversampling  x16
      ctrl_meas.osrs_t = 0b010;   // temperature oversampling x2

      if (! bmp_280_write_reg(Quan::bmp280::reg::ctrl_meas,ctrl_meas.value) ){
          hal.console->printf("bmp_280 write ctrl meas failed\n");
          return false;
      }

      if (! bmp_280_read_cal_params()){
         hal.console->printf("bmp_280 read cal params failed\n");
         return false;
      }
      return true;
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

   uint8_t __attribute__ ((section (".dma_memory"))) result_values[6] ;

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

  //See BMP280 ref_man 3.8.1 Measurement time
   /*
      pressure over sample    Temp over sample     Measurement time ms (rounded up)   Nearest useful         F Hz
               1                     1                      7                       
               2                     1                      9                               10                100
               4                     1                     14                               20                50
               8                     1                     23                               25                40
              16                     2                     44                               50                20
   
   */
   bool bmp280_request_conversion()
   {

/*
   ctrl_meas settings ref man 4.3.4
   mode 

   osrs_t[2:0]    oversampling
      000           skipped
      001            x1
      010            x2
      011            x4
      100            x8
      101           x16
      110           x16
      111           x16
      
   osrs_p[2:0]
      000            skipped
      001            x1
      010            x2
      011            x4
      100            x8
      101           x16
      110           x16
      111           x16
*/

      Quan::bmp280::ctrl_meas_bits ctrl_meas;
      ctrl_meas.mode   = 0b001;   // forced
      ctrl_meas.osrs_p = 0b101;   // pressure oversampling  x16
      ctrl_meas.osrs_t = 0b010;   // temperature oversampling x2
      // todo incorporate into write
  
      if( Quan::bmp280::write(Quan::bmp280::reg::ctrl_meas,ctrl_meas.value)){
        return true;
      }else{
         hal.console->printf("bmp_280 start conv failed\n");
      }
      if(Quan::i2c_periph::has_errored()){
         hal.console->printf("i2c error : trying reset\n");
         Quan::i2c_periph::init();
      }
      return false;
   }

   bool bmp280_start_read()
   {
      if ( Quan::bmp280::read(Quan::bmp280::reg::press_msb,result_values,6)){
         return true;
      }else{
         hal.console->printf("bmp_280 read failed\n");
      }
      if(Quan::i2c_periph::has_errored()){
         hal.console->printf("i2c error : trying reset\n");
         Quan::i2c_periph::init();
      }
      return false;
   }

} // ~namespace

namespace Quan{

   bool setup_baro()
   {
      return bmp_280_setup();     
   }
   
   // takes 330 usec
   bool baro_request_conversion()
   {
      return bmp280_request_conversion();
   }

   // must be some time after request_conversion finishes
   // to allow for conversion to take place
   // exact delay dependent on the filter settings
   // takes  930 usec
   bool baro_start_read()
   {
      return bmp280_start_read();
   }
   // must be > 930 usec after  baro_start_read
   bool baro_calculate()
   {
      auto baro_queue = get_baro_queue_handle();
      detail::baro_args args;
      bmp280_calculate(args);
      if ( uxQueueSpacesAvailable(baro_queue) != 0){
       //  hal.console->printf("Sending baro data to queue\n");
         xQueueSendToBack(baro_queue,&args,0);
      }
      return true;
   }
   
}