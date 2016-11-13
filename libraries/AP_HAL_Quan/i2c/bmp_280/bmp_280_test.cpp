

//#include <quan/temperature.hpp>

#include "bmp_280.hpp"

Quan::bmp280::calib_param_t Quan::bmp280::calib_param;

bool bmp_280_write_reg(uint8_t reg, uint8_t val)
{
   if (!Quan::wait_for_i2c_bus_free(quan::time::ms{100})) { return false;}
   if ( Quan::bmp280::write(reg, val)){
      return true;
   }else{
      serial_port::write("bmp 280 write reg failed\n");
      if(Quan::i2c_periph::has_errored()){
         Quan::i2c_periph::init();
      }
      return false;
   }
}

bool bmp_280_read_regs(uint8_t reg,uint8_t * result, uint32_t len)
{
   if (!Quan::wait_for_i2c_bus_free(quan::time::ms{100})) { return false;}
   if ( Quan::bmp280::read(reg, result, len)){
      return true;
   }else{
      serial_port::write("bmp 280 read reg failed\n");
      if(Quan::i2c_periph::has_errored()){
         Quan::i2c_periph::init();
      }
      return false;
   }
}

bool bmp_280_read_cal_params()
{  
   bool result = bmp_280_read_regs(Quan::bmp280::reg::dig_T1,Quan::bmp280::calib_param.arr,24);
   if (result){
#if 0
      if (!Quan::wait_for_i2c_bus_free(quan::time::ms{100})) { return false;}
      serial_port::write("--------- call_ params ----------\n");
      
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

      serial_port::write("-------- ~call_ params ----------\n");
#endif
   }else{
      serial_port::write("failed to read cal params\n");
      if(Quan::i2c_periph::has_errored()){
         Quan::i2c_periph::init();
      }
   }
   return result;
}

// see https://github.com/BoschSensortec/BMP280_driver/blob/master/bmp280.h
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
   return bmp_280_write_reg(Quan::bmp280::reg::config,config.value) &&
         bmp_280_write_reg(Quan::bmp280::reg::ctrl_meas,ctrl_meas.value) &&
         bmp_280_read_cal_params();
}

namespace {
   uint8_t * result_values = nullptr;
}

// ----------------------------------------------------

// Returns temperature in DegC, resolution is 0.01 DegC. Output value of “5123” equals 51.23 DegC.
// t_fine carries fine temperature as global value
namespace {
   int32_t t_fine;
}

double bmp280_compensate_temperature_double(int32_t v_uncomp_temperature_s32)
{
	//double v_x1_u32r = BMP280_INIT_VALUE;
	//double v_x2_u32r = BMP280_INIT_VALUE;
	//double temperature = BMP280_INIT_VALUE;
	/* calculate x1*/
	double v_x1_u32r = (((double)v_uncomp_temperature_s32) / 16384.0 -
			((double)Quan::bmp280::calib_param.dig_T1) / 1024.0) *
	((double)Quan::bmp280::calib_param.dig_T2);
	/* calculate x2*/
	double v_x2_u32r = ((((double)v_uncomp_temperature_s32) / 131072.0 -
			((double)Quan::bmp280::calib_param.dig_T1) / 8192.0) *
			(((double)v_uncomp_temperature_s32) / 131072.0 -
			((double)Quan::bmp280::calib_param.dig_T1) / 8192.0)) *
	((double)Quan::bmp280::calib_param.dig_T3);
	/* calculate t_fine*/
	t_fine = (int32_t)(v_x1_u32r + v_x2_u32r);
	/* calculate true pressure*/
	double temperature = (v_x1_u32r + v_x2_u32r) / 5120.0;

	return temperature;
}

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

void result_calc()
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

   serial_port::printf<100>("T = %f, P = %f\n",temperature/ 100.0, pressure/ 255.0);
}

bool bmp_280_read()
{
   bool result = bmp_280_read_regs(Quan::bmp280::reg::press_msb,result_values,6);
   if (result){
      if (!Quan::wait_for_i2c_bus_free(quan::time::ms{100})) { return false;}
      result_calc();
      return true;;
   }else{
      serial_port::write("bmp_280 read failed\n");
      if(Quan::i2c_periph::has_errored()){
         Quan::i2c_periph::init();
      }
      return false;
   }

}
bool bmp_280_start_conv()
{
   Quan::bmp280::ctrl_meas_bits ctrl_meas;
   ctrl_meas.mode   = 0b001;   // forced
   ctrl_meas.osrs_p = 0b001;   // pressure oversampling  x1
   ctrl_meas.osrs_t = 0b001;   // temperature oversampling x1
   // N.B with these settings max update == 20 Hz
   if ( ctrl_meas.value !=  0b00100101){
      serial_port::write("unexpected ctrl_meas value\n");
   }
   return bmp_280_write_reg(Quan::bmp280::reg::ctrl_meas,ctrl_meas.value);
}

bool bmp_280_run()
{
   result_values = (uint8_t*)malloc(6);
   if (bmp_280_setup()){
      serial_port::write("bmp280 setup ok\n");
      for (;;){
         if (!bmp_280_start_conv()){
            serial_port::write("start conv failed\n");
            return false;
         }
         Quan::delay(quan::time::ms{50});
         if ( ! bmp_280_read()){
            return false;
         }
     }
     return true;
  }else{
     serial_port::write("bmp280 run failed\n");
     if(Quan::i2c_periph::has_errored()){
        Quan::i2c_periph::init();
     }
     return false;
  }
}


bool bmp280_math_test()
{
   Quan::bmp280::calib_param.dig_T1  = 27504;
   Quan::bmp280::calib_param.dig_T2  = 26435;
   Quan::bmp280::calib_param.dig_T3  = -1000;
   Quan::bmp280::calib_param.dig_P1  = 36477;
   Quan::bmp280::calib_param.dig_P2  = -10685;
   Quan::bmp280::calib_param.dig_P3  = 3024;
   Quan::bmp280::calib_param.dig_P4  = 2855;
   Quan::bmp280::calib_param.dig_P5  = 140;
   Quan::bmp280::calib_param.dig_P6  = -7;
   Quan::bmp280::calib_param.dig_P7  = 15500;
   Quan::bmp280::calib_param.dig_P8  = -14600;
   Quan::bmp280::calib_param.dig_P9  = 6000;

   int32_t adc_T = 519888;
   int32_t adc_P = 415148;

   int32_t const temperature = bmp280_compensate_T_int32(adc_T);

   uint32_t const pressure = bmp280_compensate_P_int64(adc_P);
      
   serial_port::printf<100>("temperature = %f DegC, pressure = %f Pa\n",temperature/100.0, pressure/256.0);

   return true;
}

bool bmp_280_test()
{
   serial_port::write("bmp 280 whoami test \n");
   auto  start_time = millis();
   uint8_t whoami = 0;
   unsigned count = 0;
   if (!Quan::wait_for_i2c_bus_free(quan::time::ms{100})) { return false;}

   for ( uint8_t i = 0; i < 5; ++i){
      if ( Quan::bmp280::read(Quan::bmp280::reg::whoami, &whoami, 1U)){
         // Though read function returned, it is non blocking
         // result is only available once bus has been released
         if (!Quan::wait_for_i2c_bus_free(quan::time::ms{100})) { 
            if(Quan::i2c_periph::has_errored()){
               Quan::i2c_periph::init();
            }
            return false;
         }
         if ( whoami == Quan::bmp280::val::whoami){
            serial_port::printf<100>("whoami test succesful: got %u\n",static_cast<unsigned>(whoami));
            ++count;
         }else{
            serial_port::printf<100>("whoami test failed: got %u\n",static_cast<unsigned>(whoami));
         }
      }else{
         serial_port::write("whoami baro test i2c read function failed: read failed\n");
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

