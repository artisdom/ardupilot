
#include <stm32f4xx.h>
#include "FreeRTOS.h"
#include <quan/stm32/gpio.hpp>
#include <quan/stm32/spi.hpp>

#include "SPIDriver.h"

namespace {

   // SPI1 used for MPU6000 IMU
   typedef quan::stm32::spi1 spi1;
   //pins
   // MOSI
   typedef quan::mcu::pin<quan::stm32::gpiob,5> spi1_mosi;
   typedef quan::mcu::pin<quan::stm32::gpiob,4> spi1_miso;
   typedef quan::mcu::pin<quan::stm32::gpiob,3> spi1_sck;
   typedef quan::mcu::pin<quan::stm32::gpioa,12> spi1_soft_nss;

   void setup_pins()
   {
      //spi1
      quan::stm32::module_enable<quan::stm32::gpioa>();
      quan::stm32::module_enable<quan::stm32::gpiob>();

      quan::stm32::apply<
         spi1_mosi
         ,quan::stm32::gpio::mode::af5  
         ,quan::stm32::gpio::pupd::none
      >();

      quan::stm32::apply<
         spi1_miso
         ,quan::stm32::gpio::mode::af5  
         ,quan::stm32::gpio::pupd::none
      >();

      quan::stm32::apply<
         spi1_sck
         ,quan::stm32::gpio::mode::af5  
         ,quan::stm32::gpio::pupd::none
      >();

      quan::stm32::apply<
         spi1_soft_nss
         ,quan::stm32::gpio::mode::output
         ,quan::stm32::gpio::pupd::none
         ,quan::stm32::gpio::ospeed::slow
         ,quan::stm32::gpio::ostate::high
      >();

   }
/*
   
    // clock for MPU6000 want a 1 MHz and a 20 MHz clock
      // apb2 clock (fpclk) is at 84 MHz
      // for slow speed
      // use fpclk div 128 == 650 kHz
      // cr1.br = 0b110;
      // for high speed
      // 21 MHz clock ( 20 MHz +- 10%) fpclk div 4 for high speed mode
      // cr1.br = 0b001;

// DMA
      RX on DMA2_Stream0.ch3 or DMA2_Stream2.ch3
      TX on DMA2_Stream3.ch3 or DMA2_Stream5.ch3
*/
   constexpr uint16_t spi_slow_brr = (0b110 << 3);
   constexpr uint16_t spi_fast_brr = (0b01 << 3);
   constexpr uint16_t spi_brr_mask = (0b111 << 3);

   void spi_setup()
   {
      setup_pins();

      // TODO 
     // quan::stm32::module_enable<spi1>();
      quan::stm32::rcc::get()->apb2enr.bb_setbit<12>(); // |= (1 << 12);
      quan::stm32::rcc::get()->apb2rstr.bb_setbit<12>();
      quan::stm32::rcc::get()->apb2rstr.bb_clearbit<12>();
      // spi setup
     
      // always full transfer
      // MSB first
      // data clock timing 
      // cr1

/*
      // SPE = false // periphal disabled for now enable only for a transfer?
      // LSBFIRST =  false, 
      // RXONLY = false
      // DFF = false 8  bit
      // crcnext = false
      // crcen = false
      // BIDIMODE = false; // 2 line mode
      // BIDIOE = false ;//dont care
      // SSI =  0 
*/
      spi1::get()->cr1 = 
         ( 1 << 0)      // (CPHA)
       | ( 1 << 1)      // (CPOL)
       | ( 1 << 2)      // (MSTR)
       |  spi_slow_brr  // (BRR)
       |  (1 << 8)      // (SSI)
       |  (1 << 9)      // (SSM)
      ;
        
     // spi1::get()->cr2 = 
      //  TXDMAEN
      //  RXDMAEN
   }

   void start_spi()
   {
       spi1::get()->cr1.bb_setbit<6>(); //( SPE)
   }

   bool transfer_flag = false;
   struct Quan::spi_device_driver : public AP_HAL::SPIDeviceDriver {
       spi_device_driver(){}
       void init()
       {
         m_semaphore.init();
         spi_setup();
         start_spi();
       }

       AP_HAL::Semaphore* get_semaphore(){ return & m_semaphore;}
       bool transaction(const uint8_t *tx, uint8_t *rx, uint16_t len)
       {
         if ( tx == nullptr){
            return false;
         }
         cs_assert();
         if (rx == nullptr){
            transfer(tx,len);
         }else{
            for ( uint16_t i = 0 ; i < len; ++i){
               rx[i] = transfer(tx[i]);
            }
         }
         cs_release();
       }

       void cs_assert();
       {
          quan::stm32::clear<spi1_soft_nss>();
       }

       void cs_release()
       {
          quan::stm32::set<spi1_soft_nss>();
       }

       // send and receive 1 byte no nss so not really interface
       uint8_t transfer(uint8_t data)
       {
            ll_write(tx[i]);
            while ( (!txe()) || ( busy()) || (!rxne()) ){;}
            return ll_read();
       }
       //tx only... not interface
       void transfer (const uint8_t *data, uint16_t len)
       {
          for ( uint16_t i = 0; i < len; ++i){
            transfer(data[i]);
          }
       }

       void set_bus_speed(enum bus_speed speed) 
       {

       }
       void set_state(State state) { };
       State get_state() { return State::UNKNOWN; }
   private:
       static bool txe(){ return spi1::get()->sr.bb_getbit<1>();}
       static bool rxne(){ return spi1::get()->sr.bb_getbit<0>();}
       static bool busy() { return spi1::get()->sr.bb_getbit<7>();}
       static uint8_t ll_read { return spi1::get()->dr;}
       static void ll_write ( uint8_t val){ spi1::get()->dr = val;}
       QuanSemaphore m_semaphore;
   } mpu6000device ;

}  // namespace


Quan::QuanSPIDeviceManager::QuanSPIDeviceManager()
{}

void Quan::QuanSPIDeviceManager::init(void *)
{
 // init each spi device?
   mpu6000device::init();
}

/*
 devices are listed in AP_HAL/AP_HAL_Namespace.hpp
  including AP_HAL:: SPIDevice_MPU6000 
*/
AP_HAL::SPIDeviceDriver* Quan::QuanSPIDeviceManager::device(enum AP_HAL::SPIDevice e)
{
   return ( e == AP_HAL::SPIDevice_MPU6000 )
      ? &mpu6000device 
      : nullptr
  ;
}

