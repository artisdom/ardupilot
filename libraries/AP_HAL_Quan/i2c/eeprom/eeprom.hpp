#ifndef STM32F4_TEST_M24M01_HPP_INCLUDED
#define STM32F4_TEST_M24M01_HPP_INCLUDED

#include "../i2c_driver/i2c_eeprom_driver.hpp"

namespace Quan{

   struct eeprom_info {

      struct eeprom_m24m01{ 
         static constexpr uint8_t      get_device_address()   { return 0b10100000;}
         static constexpr const char * get_device_name()      { return "M24M01 eeprom";}
         static constexpr uint32_t     get_memory_size_bytes(){ return 128U * 1024U;}
         static constexpr uint32_t     get_page_size_bytes()   { return 256U;}
         static constexpr Quan::i2c_driver::millis_type get_write_cycle_time() { return Quan::i2c_driver::millis_type{5U};}
      };

      struct eeprom_24lc128{
         static constexpr uint8_t      get_device_address()   { return 0b10100000;}
         static constexpr const char * get_device_name()      { return "24LC128 eeprom";}
         static constexpr uint32_t     get_memory_size_bytes(){ return 16U * 1024U;}
         static constexpr uint32_t     get_page_size_bytes()   { return 64U;}
         static constexpr Quan::i2c_driver::millis_type get_write_cycle_time() { return Quan::i2c_driver::millis_type{5U};}
      };

   };

}// Quan

#endif // STM32F4_TEST_M24M01_HPP_INCLUDED
