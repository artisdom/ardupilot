


#include "Storage.h"

extern const AP_HAL::HAL& hal;

namespace Quan{

    bool eeprom_read(void * buffer,uint16_t eeprom_address,size_t n);
    bool eeprom_write(uint16_t eeprom_address, void const * buffer,size_t n);

}

using namespace Quan;

QuanStorage::QuanStorage()
{}

void QuanStorage::init(void*)
{}

void QuanStorage::read_block(void* dst, uint16_t eeprom_address, size_t n) 
{
   if ( !Quan::eeprom_read(dst,eeprom_address,n) ){
      hal.scheduler->panic("eeprom read failed");
   }
}

void QuanStorage::write_block(uint16_t eeprom_address, const void* src, size_t n)
{
   if ( !Quan::eeprom_write(eeprom_address,src,n) ){
      hal.scheduler->panic("eeprom write failed");
   }  
}

