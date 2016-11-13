

#include "i2c_register_based_driver.hpp"
#include <AP_HAL_Quan/AP_HAL_Quan.h>


Quan::i2c_register_based_driver_base::data_ptr_type Quan::i2c_register_based_driver_base::m_data{}; // return data
uint32_t Quan::i2c_register_based_driver_base::m_data_length = 0U;
uint8_t  Quan::i2c_register_based_driver_base::m_register_index = 0U; //

using AP_HAL::panic;

// todo remove the time arg.
// It shouldnt need to wait
// We should probably be calling bus_free only here
bool Quan::i2c_register_based_driver_base::get_bus()
{

   if(!Quan::i2c_periph::bus_released()) {
     // serial_port::write("i2c reg based driver : bus not released\n");
      return false;
   }
   // i2c bus may still be busy after bus was released
   if(!Quan::i2c_periph::get_bus()){
     // serial_port::write("i2c reg based driver : failed to acquire bus\n");
      return false;
   }
   return true;
}

/*
This function starts the read and then  returns
 The read data ptr must remain valid
 After the function returns
 wait for bus to be released
 which signals that the read has been successful
 see the lis3_mdl_test example.
*/
bool Quan::i2c_register_based_driver_base::ll_read(uint8_t register_index, uint8_t * data, uint32_t len)
{

   if ( ( len > 1 ) && ( !is_valid_heap_memory(data) ) ){
      panic("invalid mem in i2c_reg based driver read"); 
      return false;
   }
   m_data.read_ptr = data;
   m_register_index = register_index;
   m_data_length = len;

   Quan::i2c_periph::set_error_handler(on_read_error);
   Quan::i2c_periph::set_event_handler(on_read_start_sent); // first handler

   Quan::i2c_periph::enable_error_interrupts(true);
   Quan::i2c_periph::enable_event_interrupts(true);
   Quan::i2c_periph::enable_buffer_interrupts(false);
   Quan::i2c_periph::enable_ack_bit(true);

   if (m_data_length > 1 ){
      Quan::i2c_periph::enable_dma_bit(true);
      Quan::i2c_periph::enable_dma_last_bit(true);
      Quan::i2c_periph::set_dma_rx_buffer(data,m_data_length);
      Quan::i2c_periph::clear_dma_rx_stream_flags();
      Quan::i2c_periph::set_dma_rx_handler(on_read_dma_transfer_complete);
      Quan::i2c_periph::enable_dma_rx_stream(true);
   }

   Quan::i2c_periph::request_start_condition();

   return true;
}

// read handlers
void Quan::i2c_register_based_driver_base::on_read_start_sent()
{   // sb set
   Quan::i2c_periph::get_sr1();
   Quan::i2c_periph::send_data(get_device_address()); //
   Quan::i2c_periph::set_event_handler(on_read_device_address_sent);
}

void Quan::i2c_register_based_driver_base::on_read_device_address_sent()
{   // txe , addr
   Quan::i2c_periph::get_sr1();
   Quan::i2c_periph::get_sr2();
   Quan::i2c_periph::send_data(m_register_index);
   Quan::i2c_periph::request_start_condition();
   Quan::i2c_periph::set_event_handler(on_read_reg_index_sent);
}

void Quan::i2c_register_based_driver_base::on_read_reg_index_sent()
{  // txe btf 
   Quan::i2c_periph::receive_data(); //clear the txe and btf flags
   Quan::i2c_periph::set_event_handler(on_read_repeated_start_sent);
}

void Quan::i2c_register_based_driver_base::on_read_repeated_start_sent()
{   // sb
   Quan::i2c_periph::get_sr1();
   Quan::i2c_periph::send_data(get_device_address() | 1);
   Quan::i2c_periph::set_event_handler(on_read_device_read_address_sent);
}

void Quan::i2c_register_based_driver_base::on_read_device_read_address_sent()
{   // addr
   Quan::i2c_periph::get_sr1();
   Quan::i2c_periph::get_sr2();
   if ( m_data_length > 1){ // into dma
     //  led::on();
      Quan::i2c_periph::enable_event_interrupts(false);
      // since we transferred control to dma
      // then only the dma handler is  now required
   }else{ // dma doesnt work for single byte read
      Quan::i2c_periph::enable_ack_bit(false);
      Quan::i2c_periph::request_stop_condition();
      Quan::i2c_periph::enable_buffer_interrupts(true); // enable rxne
      Quan::i2c_periph::set_event_handler(on_read_single_byte_handler);
   }
}

void Quan::i2c_register_based_driver_base::on_read_single_byte_handler()
{  
   *m_data.read_ptr = Quan::i2c_periph::receive_data();
   Quan::i2c_periph::enable_buffer_interrupts(false);
   Quan::i2c_periph::enable_event_interrupts(false);
   Quan::i2c_periph::set_default_handlers();
   Quan::i2c_periph::release_bus();
}

// dma handler
void Quan::i2c_register_based_driver_base::on_read_dma_transfer_complete()
{ 
   Quan::i2c_periph::enable_dma_rx_stream(false);
   Quan::i2c_periph::enable_dma_bit(false);
   Quan::i2c_periph::enable_dma_last_bit(false);
   Quan::i2c_periph::clear_dma_rx_stream_tcif();
   Quan::i2c_periph::request_stop_condition();
   Quan::i2c_periph::set_default_handlers();
   Quan::i2c_periph::release_bus();
}

void Quan::i2c_register_based_driver_base::on_read_error()
{
   if (get_device_name() != nullptr){
      panic ("%s",get_device_name());
   }else{
      panic("unknown device");
   }
   hal.console->printf(" : read error");
   Quan::i2c_periph::default_error_handler();
}

void Quan::i2c_register_based_driver_base::on_write_error()
{
   if (get_device_name() != nullptr){
      panic ("%s",get_device_name());
   }else{
      panic("unknown device");
   }
   hal.console->printf(" : write error");
   Quan::i2c_periph::default_error_handler();
}

bool Quan::i2c_register_based_driver_base::ll_write(uint8_t register_index, uint8_t value)
{

   m_data.value_to_write = value;
   m_register_index = register_index;
   m_data_length = 1U;

   Quan::i2c_periph::set_error_handler(on_write_error);
   Quan::i2c_periph::set_event_handler(on_write_start_sent);

   Quan::i2c_periph::enable_error_interrupts(true);
   Quan::i2c_periph::enable_event_interrupts(true);
   Quan::i2c_periph::enable_buffer_interrupts(false);

   Quan::i2c_periph::request_start_condition();
   return true;
}

// write handlers
// only one reg is written at a time for setting up
void Quan::i2c_register_based_driver_base::on_write_start_sent()
{  
   Quan::i2c_periph::get_sr1();
   Quan::i2c_periph::send_data(get_device_address());
   Quan::i2c_periph::set_event_handler(on_write_device_address_sent);
}

void Quan::i2c_register_based_driver_base::on_write_device_address_sent()
{   
   Quan::i2c_periph::get_sr1();
   Quan::i2c_periph::get_sr2();
   Quan::i2c_periph::send_data(m_register_index);
   Quan::i2c_periph::set_event_handler(on_write_reg_index_sent); // on_write_reg_index_sent
}

void Quan::i2c_register_based_driver_base::on_write_reg_index_sent()
{  
   Quan::i2c_periph::send_data(m_data.value_to_write);
   Quan::i2c_periph::set_event_handler(on_write_value_sent);
}

void Quan::i2c_register_based_driver_base::on_write_value_sent()
{   
   Quan::i2c_periph::enable_event_interrupts(false);
   Quan::i2c_periph::request_stop_condition();
   Quan::i2c_periph::set_default_handlers();
   Quan::i2c_periph::release_bus();
}

