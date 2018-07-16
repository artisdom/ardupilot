
#include "FreeRTOS.h"
#include <task.h>
#include "i2c_task.hpp"
#include "sdp3x_i2c.hpp"

#if defined QUAN_I2C_DEBUG
#include <AP_HAL_Quan/i2c/i2c_driver/i2c_debug.hpp>
#endif

extern const AP_HAL::HAL& hal;

#if 1
namespace {

   // input: data must be a non null pointer to an array of data to be crced of length num_elements
   // output: the calculated crc
   uint8_t sdp3x_crc(const uint8_t * data, uint32_t num_elements)
   {

      uint8_t constexpr init_crc_value = 0xFF;
      uint8_t constexpr polynomial = 0x31;

      uint8_t crc_value = init_crc_value;

      for (uint32_t i = 0U; i < num_elements; ++i) {
         crc_value ^= data[i];
         for (uint8_t j = 0U; j < 8U; ++j) {
            crc_value = ((crc_value & 0x80) == 0U) 
            ? (crc_value << 1U)
            : (crc_value << 1U) ^ polynomial;
         }
      }

      return crc_value;
   }

}
#endif

namespace Quan{

  bool sdp3x_exist_test()
  {

    uint8_t ar[18] = {(sdp3x_i2c_0::cmd::read_product_id[0] & 0xFF00) >> 8U,sdp3x_i2c_0::cmd::read_product_id[0] & 0xFF};
    if (! sdp3x_i2c_0::write(ar,2)){
      hal.console->printf("sdp3x write cmd 0 failed\n");
      return false;
    }
    if (! Quan::wait_for_bus_free_ms(1)){
      hal.console->printf("sdp3x couldnt get bus after cmd 0 write\n");
      show_i2c_sr1_flags();
      return false;
    }
    ar[0] = (sdp3x_i2c_0::cmd::read_product_id[1] & 0xFF00) >> 8U;
    ar[1] = sdp3x_i2c_0::cmd::read_product_id[1] & 0xFF;

   if (! sdp3x_i2c_0::write(ar,2)){
      hal.console->printf("sdp3x write cmd 1 failed\n");
      show_i2c_sr1_flags();
      return false;
    }
    for (auto & p : ar){
         p = 0U;
    }
    if (! Quan::wait_for_bus_free_ms(1)){
      hal.console->printf("sdp3x couldnt get bus after cmd 1 write\n");
      return false;
    }
    if (!sdp3x_i2c_0::read(ar,18)){
      hal.console->printf("sdp3_x read failed");
      show_i2c_sr1_flags();
      return false;
    }
   
    bool crc_good = sdp3x_crc(&ar[0],2) == ar[2];
    if (!crc_good){
       hal.console->printf("crc failed\n");
    }
    uint32_t const sdp3x_id = (ar[0] << 24U) | (ar[1] << 16U) | (ar[3] << 8U) | ( ar[4]);

    hal.console->printf("sdp3x id = %X\n", static_cast<unsigned int>(sdp3x_id));
    
    return true;
    
  }

}



