
#include "i2c_eeprom_driver.hpp"
#include "../eeprom/eeprom.hpp"
#include <AP_HAL_Quan/AP_HAL_Quan.h>

extern const AP_HAL::HAL& hal;

using AP_HAL::millis;

Quan::i2c_eeprom_driver_base::millis_type       Quan::i2c_eeprom_driver_base::m_write_cycle_time_ms;
volatile uint32_t                         Quan::i2c_eeprom_driver_base::m_write_end_time_ms;
uint32_t                                  Quan::i2c_eeprom_driver_base::m_memory_size_bytes ;
uint32_t                                  Quan::i2c_eeprom_driver_base::m_page_size_bytes;

#if !defined QUAN_I2C_TX_DMA
uint32_t        Quan::i2c_eeprom_driver_base::m_data_length = 0;
//uint8_t const*  i2c_eeprom_driver_base::m_data_ptr = nullptr;
Quan::i2c_eeprom_driver_base::data_ptr_type Quan::i2c_eeprom_driver_base::m_data;
uint32_t        Quan::i2c_eeprom_driver_base::m_index =0;
#endif

uint8_t         Quan::i2c_eeprom_driver_base::m_data_address[2] ={0U,0U};
uint8_t         Quan::i2c_eeprom_driver_base::m_data_address_hi = 0U;

bool Quan::i2c_eeprom_driver_base::write_in_progress() 
{ 
  return millis() < m_write_end_time_ms;
}

void Quan::i2c_eeprom_driver_base::set_new_write_end_time()
{
    m_write_end_time_ms = millis() + m_write_cycle_time_ms + 1U;
}

bool Quan::i2c_eeprom_driver_base::get_bus()
{
   if (!Quan::i2c_periph::bus_free()) {
      return false;
   }

   if ( write_in_progress()){
       return false;
   }
   
   if(!Quan::i2c_periph::get_bus()){
      return false;
   }

   return true;
}

bool Quan::i2c_eeprom_driver_base::install_device(
      const char* name, 
      uint8_t address,
      uint32_t memory_size,
      uint32_t page_size,
      i2c_driver::millis_type write_cycle_time
){
   if ( ! get_bus() ){
      return false; // has reported
   }
   set_device_name(name);
   set_device_address(address);
   set_memory_size_bytes(memory_size);
   set_page_size_bytes(page_size);
   set_write_cycle_time(write_cycle_time);
   return true;  
}

namespace  {

   // n.b must do
   // ! Quan::i2c_periph::bus_free() || eeprom::write_in_progress()
   // since write in progress is taken from bus_free
   // any other way wont work
   // N.B specific to eeprom
   bool wait_for_bus_free()
   {
      auto const wait_ms = millis() + 1000U;
      bool bus_free = false;
      while (millis() < wait_ms){
         if ( Quan::i2c_periph::bus_free()){
            bus_free = true;
            break;
         }
      }
      if(! bus_free){
        // serial_port::write("failed to get free bus\n");
         return false;
      }
      while ( millis() < wait_ms){
         if ( ! Quan::i2c_eeprom_driver_base::write_in_progress()){
            return true;
         }
      }
     // serial_port::write("write to eeprom didnt end\n");
      return false;
   }

}

/*
   For freertos, dealing with passing memory handles
   If memory is all same size then malloc might be ok
*/

template <typename ID>
bool Quan::i2c_eeprom_driver<ID>::read(uint32_t start_address_in,uint8_t* data_out, uint32_t len)
{
   if ( len == 0){
      AP_HAL::panic("eeprom_orig write zero length");
      return false;
   }
   if ( data_out == nullptr){
      AP_HAL::panic("eeprom_orig read data ptr is null");
      return false;
   }
   // one past the last address to write
   uint32_t const end_address = start_address_in + len;

   if ( end_address >= ID::get_memory_size_bytes()){
      AP_HAL::panic("eeprom_orig read address out of range");
      return false;
   }
   uint32_t const start_page = start_address_in / ID::get_page_size_bytes();
   uint32_t const end_page =  (end_address -1) / ID::get_page_size_bytes();
   if ( start_page == end_page){
      if (! wait_for_bus_free()){
         return false;
      }
      return read_page(start_address_in,data_out, len);
   }else{

      uint32_t  start_address = start_address_in;
      uint8_t * data = data_out;
      uint8_t * const data_end = data_out + len;
      uint32_t  cur_page = start_page;
      uint32_t  bytes_to_read  = ID::get_page_size_bytes() - (start_address_in % ID::get_page_size_bytes());

      while (data < data_end){

         if (! wait_for_bus_free()){
            return false;
         }
         if (!read_page(start_address,data,bytes_to_read)){
            return false;
         }
         start_address += bytes_to_read; // dest eeprom address
         data += bytes_to_read; //advance
          if ( ++cur_page == end_page){
            bytes_to_read = data_end - data;
         }else{
            bytes_to_read = ID::get_page_size_bytes();
         }
      }
      return true;
   }
}

bool Quan::i2c_eeprom_driver_base::ll_read(uint32_t start_address_in, uint8_t * data, uint32_t len)
{
   m_data_address[0] = static_cast<uint8_t>((start_address_in & 0xFF00) >> 8U);
   m_data_address[1] = static_cast<uint8_t>(start_address_in & 0xFF);
   m_data_address_hi = static_cast<uint8_t>((start_address_in >> 15U) & 0b10);

#if !defined QUAN_I2C_TX_DMA
   m_data.read_ptr = data;
   m_data_length = len;
   m_index       = 0;
#endif   
   
   Quan::i2c_periph::set_error_handler(on_read_error);
   Quan::i2c_periph::set_event_handler(on_read_start_sent);

   Quan::i2c_periph::enable_error_interrupts(true);
   Quan::i2c_periph::enable_event_interrupts(true);
   Quan::i2c_periph::enable_buffer_interrupts(false);
   Quan::i2c_periph::enable_ack_bit(true);

   if (len > 1 ){
      Quan::i2c_periph::enable_dma_bit(true);
      Quan::i2c_periph::enable_dma_last_bit(true);
      Quan::i2c_periph::set_dma_rx_buffer(data,len);
      Quan::i2c_periph::clear_dma_rx_stream_flags();
      Quan::i2c_periph::set_dma_rx_handler(on_read_dma_transfer_complete);
      Quan::i2c_periph::enable_dma_rx_stream(true);
   }

   Quan::i2c_periph::request_start_condition();

   return true;
}

void Quan::i2c_eeprom_driver_base::on_read_start_sent()
{
   Quan::i2c_periph::get_sr1();
   Quan::i2c_periph::send_data(get_device_address() | m_data_address_hi);
   Quan::i2c_periph::set_event_handler(on_read_device_address_sent);
}

void Quan::i2c_eeprom_driver_base::on_read_device_address_sent()
{
   Quan::i2c_periph::get_sr1();
   Quan::i2c_periph::get_sr2();
   Quan::i2c_periph::send_data(m_data_address[0]);
   Quan::i2c_periph::set_event_handler(on_read_data_address_hi_sent);
}

void Quan::i2c_eeprom_driver_base::on_read_data_address_hi_sent()
{
   Quan::i2c_periph::send_data(m_data_address[1]);
   Quan::i2c_periph::request_start_condition();
   Quan::i2c_periph::set_event_handler(on_read_data_address_lo_sent);
}

void Quan::i2c_eeprom_driver_base::on_read_data_address_lo_sent()
{
   Quan::i2c_periph::receive_data(); //clear the txe and btf flags
   Quan::i2c_periph::set_event_handler(on_read_start2_sent);
}

void Quan::i2c_eeprom_driver_base::on_read_start2_sent()
{
   // flags sr1.sb
   Quan::i2c_periph::get_sr1();
   Quan::i2c_periph::send_data(get_device_address() | m_data_address_hi | 1U); // send eeprom read address
   Quan::i2c_periph::set_event_handler(on_read_device_read_address_sent);
}

void Quan::i2c_eeprom_driver_base::on_read_device_read_address_sent()
{
   // flags sr2.[busy:msl] sr1.addr
   Quan::i2c_periph::get_sr1();
   Quan::i2c_periph::get_sr2();
   if ( m_data_length > 1){ // into dma
      Quan::i2c_periph::enable_event_interrupts(false);
      Quan::i2c_periph::set_event_handler(Quan::i2c_periph::default_event_handler);
   }else{ // dma doesnt work for single byte read
      Quan::i2c_periph::enable_ack_bit(false);
      Quan::i2c_periph::request_stop_condition();
      Quan::i2c_periph::enable_buffer_interrupts(true); // enable rxne
      Quan::i2c_periph::set_event_handler(single_byte_receive_handler);
   }
}

void Quan::i2c_eeprom_driver_base::single_byte_receive_handler()
{
   *m_data.read_ptr = Quan::i2c_periph::receive_data();
   Quan::i2c_periph::enable_buffer_interrupts(false);
   Quan::i2c_periph::enable_event_interrupts(false);
   Quan::i2c_periph::set_default_handlers();
   Quan::i2c_periph::release_bus();
}

void Quan::i2c_eeprom_driver_base::on_read_dma_transfer_complete()
{
         //led::on();
   Quan::i2c_periph::enable_dma_rx_stream(false);
   Quan::i2c_periph::enable_dma_bit(false);
   Quan::i2c_periph::enable_dma_last_bit(false);
   Quan::i2c_periph::clear_dma_rx_stream_tcif();
   Quan::i2c_periph::request_stop_condition();
   Quan::i2c_periph::set_default_handlers();
   Quan::i2c_periph::release_bus();
}

void Quan::i2c_eeprom_driver_base::on_read_error()
{
   hal.console->printf("%s : i2c read error : ",get_device_name());
   Quan::i2c_periph::default_error_handler();
}

//--------------------------

/*
write any length of data
 This will block until the last write has started
*/

template <typename ID>
bool Quan::i2c_eeprom_driver<ID>::write(uint32_t start_address_in, uint8_t const * data_in, uint32_t len)
{

   if ( len == 0){
      AP_HAL::panic("eeprom_orig write zero length");
      return false;
   }
   if ( data_in == nullptr){
      AP_HAL::panic("eeprom_orig write data is null");
      return false;
   }
   // end address is one past the last address to write
   uint32_t const end_address = start_address_in + len;

   if ( end_address >= ID::get_memory_size_bytes()){
      hal.console->printf("memory size = %lu\n",ID::get_memory_size_bytes() );
      AP_HAL::panic("eeprom_write address out of range");
      return false;
   }
   uint32_t constexpr page_size = ID::get_page_size_bytes();
   uint32_t const start_page = start_address_in / page_size;
   uint32_t const end_page =  (end_address -1) / page_size;
  
   if ( start_page == end_page){

      if (! wait_for_bus_free()){
         return false;
      }
      return write_page(start_address_in,data_in, len);

   }else{

      uint32_t             start_address  = start_address_in;
      uint8_t const*       data           = data_in;
      uint8_t const* const data_end       = data_in + len;
      uint32_t             cur_page       = start_page;
      uint32_t             bytes_to_write = page_size - (start_address_in % page_size);

      while (data < data_end){

         if (!wait_for_bus_free() ){
            return false;
         }
         if (! write_page(start_address,data, bytes_to_write)){
           AP_HAL::panic("eewr failed2\n");
           return false;
         }
         start_address += bytes_to_write; // dest eeprom_orig address
         data += bytes_to_write; //advance
         if ( ++cur_page == end_page){
            bytes_to_write = data_end - data;
         }else{
            bytes_to_write = page_size;
         }
      }
   }
   return true;
};

template struct Quan::i2c_eeprom_driver<Quan::eeprom_info::eeprom_24lc128>;

bool Quan::i2c_eeprom_driver_base::ll_write(uint32_t data_address_in, uint8_t const * data_in, uint32_t len)
{
   m_data_address[0] = static_cast<uint8_t>((data_address_in & 0xFF00) >> 8U);
   m_data_address[1] = static_cast<uint8_t>(data_address_in & 0xFF);
// TODO check that address in range
   m_data_address_hi = static_cast<uint8_t>((data_address_in >> 15U) & 0b10);

#if !defined QUAN_I2C_TX_DMA
   m_data.write_ptr    = data_in;
   m_data_length       = len;
   m_index             = 0;
#endif   
   
   Quan::i2c_periph::set_error_handler(on_write_error);
   Quan::i2c_periph::set_event_handler(on_write_start_sent);
#if defined QUAN_I2C_TX_DMA
   Quan::i2c_periph::set_dma_tx_handler(on_dma_transfer_complete);
   Quan::i2c_periph::set_dma_tx_buffer(data,len);
   Quan::i2c_periph::enable_dma_tx_stream(false);
   Quan::i2c_periph::clear_dma_tx_stream_flags();
   //Quan::i2c_periph::enable_dma_bit(true);
#endif
   
   Quan::i2c_periph::enable_error_interrupts(true);
   Quan::i2c_periph::enable_event_interrupts(true);
   Quan::i2c_periph::enable_buffer_interrupts(false);

   Quan::i2c_periph::request_start_condition();

   return true;
}

void Quan::i2c_eeprom_driver_base::on_write_start_sent()
{
   Quan::i2c_periph::get_sr1();
   // here add the extra address bit to device address 
   Quan::i2c_periph::send_data(get_device_address() | m_data_address_hi);
   Quan::i2c_periph::set_event_handler(on_write_device_address_sent);
}

void Quan::i2c_eeprom_driver_base::on_write_device_address_sent()
{
   Quan::i2c_periph::get_sr1();
   Quan::i2c_periph::get_sr2();
   Quan::i2c_periph::send_data(m_data_address[0]);
   Quan::i2c_periph::set_event_handler(on_write_data_address_hi_sent);
}

void Quan::i2c_eeprom_driver_base::on_write_data_address_hi_sent()
{
    Quan::i2c_periph::send_data(m_data_address[1]);
#if defined QUAN_I2C_TX_DMA
    Quan::i2c_periph::set_event_handler(on_write_data_address_lo_sent);
#else
    Quan::i2c_periph::set_event_handler(on_writing_data);
#endif
}

#if defined QUAN_I2C_TX_DMA
// txe on 2nd data address
// disable i2c events
// do final dma setup
// and point dma handler to end of dma handler
// start sending the data using dma
void Quan::i2c_eeprom_driver_base::on_write_data_address_lo_sent()
{
    Quan::i2c_periph::enable_event_interrupts(false);
    Quan::i2c_periph::enable_dma_tx_stream(true);
}

void Quan::i2c_eeprom_driver_base::on_write_dma_transfer_complete()
{
   Quan::i2c_periph::enable_dma_tx_stream(false);
   Quan::i2c_periph::enable_dma_bit(false);
   Quan::i2c_periph::clear_dma_tx_stream_tcif();
   Quan::i2c_periph::enable_event_interrupts(true);
   Quan::i2c_periph::set_event_handler(on_write_last_byte_transfer_complete);
}

#else
// non dma version
void Quan::i2c_eeprom_driver_base::on_writing_data()
{
   if ( m_index == 0){ //    // txe on 2nd data adddress
      Quan::i2c_periph::enable_buffer_interrupts(true);
   }
   Quan::i2c_periph::send_data(m_data.write_ptr[m_index]);
   if ( ++m_index == m_data_length){
      Quan::i2c_periph::set_event_handler(on_write_last_byte_transfer_complete);
      Quan::i2c_periph::enable_buffer_interrupts(false);
   }
}
#endif

// btf on last
void Quan::i2c_eeprom_driver_base::on_write_last_byte_transfer_complete()
{
   Quan::i2c_periph::enable_event_interrupts(false);
   Quan::i2c_periph::request_stop_condition();
   Quan::i2c_periph::set_default_handlers();
   set_new_write_end_time();
   Quan::i2c_periph::release_bus();
}

void Quan::i2c_eeprom_driver_base::on_write_error()
{
   hal.console->printf("%s : i2c write error : ",get_device_name());
   Quan::i2c_periph::default_error_handler();
}
