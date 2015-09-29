
#include <stm32f4xx.h>
#include <quan/stm32/spi.hpp>
#include <quan/stm32/gpio.hpp>
#include <quan/stm32/rcc.hpp>
#include <quan/three_d/vect.hpp>
#include <quan/stm32/millis.hpp>

#include <quan/stm32/rcc.hpp>
#include <quan/stm32/f4/exti/set_exti.hpp>
#include <quan/stm32/f4/syscfg/module_enable_disable.hpp>

#include "serial_port.hpp"
#include <cstdio>
#include <cstring>

#include <quan/time.hpp>

namespace {
   typedef quan::time_<int64_t>::ms ms;
}
void delay ( ms const &  t);

extern "C" void DMA2_Stream0_IRQHandler() __attribute__ ((interrupt ("IRQ")));
extern "C" void EXTI15_10_IRQHandler() __attribute__ ((interrupt ("IRQ")));

namespace {

   typedef console::serial_port sp;

   struct val{
      static constexpr uint8_t device_wakeup = 0U;
      static constexpr uint8_t device_reset = (1 << 7);
      static constexpr uint8_t i2c_if_dis = (1U << 4U);
     // for MPU9250 
      // static constexpr uint8_t whoami = 0x71;
      // for MPU6000
      static constexpr uint8_t whoami =   0x68  ;
   };

   struct reg{
      static constexpr uint8_t sample_rate_div     = 25U;
      static constexpr uint8_t config              = 26U;
      static constexpr uint8_t gyro_config         = 27U;
      static constexpr uint8_t accel_config        = 27U;
      static constexpr uint8_t accel_config2       = 28U;
      static constexpr uint8_t fifo_enable         = 35U;
      static constexpr uint8_t intr_bypass_en_cfg  = 55U;
      static constexpr uint8_t intr_enable         = 56U;
      static constexpr uint8_t intr_status         = 58U;
      static constexpr uint8_t accel_measurements  = 59U; // 59 to 64
      static constexpr uint8_t temp_measurements   = 65U; // 65 to 66
      static constexpr uint8_t gyro_measurements   = 67U; // 67 to 72
      static constexpr uint8_t signal_path_reset   = 104U; 
      static constexpr uint8_t accel_intr_ctrl     = 105U; 
      // user ctrl bit 7 enable DMP
      // user ctrl bit 3 reset DMP
      static constexpr uint8_t user_ctrl           = 106U;
      static constexpr uint8_t pwr_mgmt1           = 107U;
      static constexpr uint8_t pwr_mgmt2           = 108U;

      // --- DMP specific regs here ---

      static constexpr uint8_t fifo_count          = 114U; // 114 to 115
      static constexpr uint8_t fifo_read_write     = 116U; // 59 to 64
      static constexpr uint8_t whoami              = 117U; 
      static constexpr uint8_t accel_offsets       = 119U; // 119 to 126
   };



   // SPI1 used for MPU6000/ MPU9250 IMU
  struct spi_device_driver  {

      typedef quan::stm32::spi1 spi1;

      static void init() 
      {
         quan::stm32::rcc::get()->apb2enr.bb_setbit<12>(); // |= (1 << 12);
         quan::stm32::rcc::get()->apb2rstr.bb_setbit<12>();
         quan::stm32::rcc::get()->apb2rstr.bb_clearbit<12>();
         setup_spi_pins();
         setup_spi_regs(); 
         setup_exti();
         setup_dma();
         start_spi();
      }

       static bool transaction(const uint8_t *tx, uint8_t *rx, uint16_t len) 
       {
         cs_assert();
         transfer(tx,rx,len);
         cs_release();
         return true;
       }
private:
       static uint8_t transfer(uint8_t data)
       {
         while (!txe()){;}
         ll_write(data);
         while ( !rxne() ){;}
         return ll_read();
       }

       static void transfer (const uint8_t *tx, uint8_t* rx, uint16_t len)
       {
          for ( uint16_t i = 0; i < len; ++i){
            rx[i]= transfer(tx[i]);
          }
       }
  public:
       static void reg_write( uint8_t r, uint8_t v)
       {
          // critical section
          uint8_t arr[2] = {r, v};
          transaction(arr,arr, 2U);
          // ~critical section
       }

      static uint8_t reg_read( uint8_t r)
      {
         // critical section
         uint8_t arr[2] = {static_cast<uint8_t>(r | 0x80),0U};
         transaction(arr,arr, 2);
         // ~critical section
         return arr[1];
      }

   private:
      static constexpr uint16_t spi_slow_brr = (0b110 << 3);
      static constexpr uint16_t spi_fast_brr = (0b001 << 3);
      static constexpr uint16_t spi_brr_and_clear_mask = ~(0b111 << 3);
   public:
      static void set_bus_speed(int speed) 
      {
         switch (speed){
      
            case 0: 
               if ( m_fast_speed){
                  spi1::get()->cr1 = (spi1::get()->cr1 & spi_brr_and_clear_mask) | spi_slow_brr;
                  m_fast_speed = false;
               }
               break;

            case 1:
               if ( m_fast_speed == false){
                  spi1::get()->cr1 = (spi1::get()->cr1 & spi_brr_and_clear_mask) | spi_fast_brr;
                  m_fast_speed = true;
               }
               break;
            default:
               break;
         }
      }

      static void stop_spi()
      {
         spi1::get()->cr1.bb_clearbit<6>(); //( SPE)
      }

      static void start_spi()
      {
         spi1::get()->cr1.bb_setbit<6>(); //( SPE)
      }

public:
      static void cs_assert()
      {
         m_transfer_in_progress = true;
         quan::stm32::clear<spi1_soft_nss>();
         (void)ll_read();
      }

      // worst case 1 cycle at 168 Mhz == 6 ns
      static void cs_release()
      {
         while (busy()) { asm volatile ("nop":::);}
         if ( !m_fast_speed){
            // 500 ns hold
            for (uint8_t i = 0; i < 50; ++i){
               asm volatile ("nop":::);
            }
         }
         quan::stm32::set<spi1_soft_nss>();
         m_transfer_in_progress = false;
      }

      static bool txe(){ return spi1::get()->sr.bb_getbit<1>();}
      static bool rxne(){ return spi1::get()->sr.bb_getbit<0>();}
      static bool busy(){ return spi1::get()->sr.bb_getbit<7>();}
private:

      static uint8_t ll_read() { return spi1::get()->dr;}
      static void ll_write( uint8_t val){ spi1::get()->dr = val;}
      // Quan::QuanSemaphore m_semaphore;
      static bool m_fast_speed;
      static volatile bool m_transfer_in_progress;

      typedef quan::mcu::pin<quan::stm32::gpiob,5>  spi1_mosi;
      typedef quan::mcu::pin<quan::stm32::gpiob,4>  spi1_miso;
      typedef quan::mcu::pin<quan::stm32::gpiob,3>  spi1_sck;
      typedef quan::mcu::pin<quan::stm32::gpioa,12> spi1_soft_nss;
public:
      typedef quan::mcu::pin<quan::stm32::gpioc,14> mpu6000_irq;
private:
      static void setup_spi_pins()
      {
         //spi1
         quan::stm32::module_enable<quan::stm32::gpioa>();
         quan::stm32::module_enable<quan::stm32::gpiob>();
         quan::stm32::module_enable<quan::stm32::gpioc>();

         quan::stm32::apply<
            spi1_mosi
            ,quan::stm32::gpio::mode::af5  
            ,quan::stm32::gpio::pupd::none
            ,quan::stm32::gpio::ospeed::medium_fast
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
            ,quan::stm32::gpio::ospeed::medium_fast
         >();

         quan::stm32::apply<
            spi1_soft_nss
            ,quan::stm32::gpio::mode::output
            ,quan::stm32::gpio::pupd::none
            ,quan::stm32::gpio::ospeed::medium_fast
            ,quan::stm32::gpio::ostate::high
         >();
      }

      static void setup_spi_regs()
      {
         
         // SPE = false // periphal disabled for now?
         // LSBFIRST =  false, 
         // RXONLY = false
         // DFF = false 8  bit
         // crcnext = false
         // crcen = false
         // BIDIMODE = false; // 2 line mode
         // BIDIOE = false ;//dont care
         spi1::get()->cr1 = 
         ( 1 << 0)      // (CPHA)
         | ( 1 << 1)      // (CPOL)
         | ( 1 << 2)      // (MSTR)
         |  spi_slow_brr  // (BRR)
         |  (1 << 8)      // (SSI)
         |  (1 << 9)      // (SSM)
         ;
      }
public:
      static void enable_dma()
      { 
         spi1::get()->cr2 |= (( 1 << 1 ) | ( 1 << 0)) ;// (TXDMAEN ) | ( RXDMAEN)
      }

      static void disable_dma()
      { 
         spi1::get()->cr2 &= ~(( 1 << 1 ) | ( 1 << 0)) ;// (TXDMAEN ) | ( RXDMAEN)
      }
      
private:
      static void setup_exti()
      {
          // PC14
         quan::stm32::apply<
            mpu6000_irq
            , quan::stm32::gpio::mode::input
            , quan::stm32::gpio::pupd::pull_up // make this pullup ok as mpu6000 is on 3v
         >();
         quan::stm32::module_enable<quan::stm32::syscfg>(); 
         quan::stm32::set_exti_syscfg<mpu6000_irq>();
         quan::stm32::set_exti_falling_edge<mpu6000_irq>();
         quan::stm32::nvic_enable_exti_irq<mpu6000_irq>();
         NVIC_SetPriority(
            quan::stm32::detail::get_exti_irq_num<mpu6000_irq::pin_value>::value
            ,14
         );
         quan::stm32::enable_exti_interrupt<mpu6000_irq>(); 
      }
public:
      static uint8_t dma_tx_buffer[16] ;
      static volatile uint8_t  dma_rx_buffer[16] ;
private:
      static void setup_dma()
      {
         // DMA2
         quan::stm32::rcc::get()->ahb1enr |= (1 << 22);
         for ( uint8_t i = 0; i < 20; ++i){
            asm volatile ("nop" : : :);
         }

         // for now we use both tx and rx dma
         // RX
         DMA_Stream_TypeDef * dma_stream = DMA2_Stream0;
         constexpr uint32_t  dma_channel = 3;
         constexpr uint32_t  dma_priority = 0b01; // medium
         dma_stream->CR = (dma_stream->CR & ~(0b111 << 25)) | ( dma_channel << 25); //(CHSEL) select channel
         dma_stream->CR = (dma_stream->CR & ~(0b11 << 16)) | (dma_priority << 16U); // (PL) priority
         dma_stream->CR = (dma_stream->CR & ~(0b11 << 13)) ; // (MSIZE) 8 bit memory transfer
         dma_stream->CR = (dma_stream->CR & ~(0b11 << 11)) ; // (PSIZE) 8 bit transfer
         dma_stream->CR |= (1 << 10);// (MINC)
         dma_stream->CR &= ~(0b11 << 6) ; // (DIR ) peripheral to memory
         dma_stream->CR |= ( 1 << 4) ; // (TCIE)

         dma_stream->PAR = (uint32_t)&SPI1->DR;  // periph addr
         dma_stream->M0AR = (uint32_t)dma_rx_buffer; 
         dma_stream->NDTR = 16;

         NVIC_SetPriority(DMA2_Stream0_IRQn,15);  // low prio
         NVIC_EnableIRQ(DMA2_Stream0_IRQn);

         // TX
         dma_stream = DMA2_Stream5;
         constexpr uint32_t  dma_channel1 = 3;
         dma_stream->CR = (dma_stream->CR & ~(0b111 << 25)) | ( dma_channel1 << 25); //(CHSEL) select channel
         dma_stream->CR = (dma_stream->CR & ~(0b11 << 16)) | (dma_priority << 16U); // (PL) priority
         dma_stream->CR = (dma_stream->CR & ~(0b11 << 13)) ; // (MSIZE) 8 bit memory transfer
         dma_stream->CR = (dma_stream->CR & ~(0b11 << 11)) ; // (PSIZE) 8 bit transfer
         dma_stream->CR |= (1 << 10);// (MINC)
         dma_stream->CR = (dma_stream->CR & ~(0b11 << 6)) | (0b01 << 6) ; // (DIR )  memory to peripheral
        // dma_stream->CR |= ( 1 << 4) ; // (TCIE)
         dma_stream->PAR = (uint32_t)&SPI1->DR;  // periph addr
         dma_stream->M0AR = (uint32_t)dma_tx_buffer; 
         dma_stream->NDTR = 16;
         
         DMA2->HIFCR |= ( 0b111101 << 6) ; // Stream 5 clear flags
         DMA2->LIFCR |= ( 0b111101 << 0) ; // Stream 0 clear flags
      }
            
   };

   bool spi_device_driver::m_fast_speed = false;
   volatile bool spi_device_driver::m_transfer_in_progress = false;
   volatile uint8_t  spi_device_driver::dma_rx_buffer[16] __attribute__((section(".telem_buffer"))) = {0};
   uint8_t spi_device_driver::dma_tx_buffer[16] __attribute__((section(".telem_buffer"))) = {
      (reg::intr_status | 0x80),0,0,0,
      0,0,0,0,
      0,0,0,0,
      0,0,0,0
    };
}

bool whoami_test()
{
   uint8_t val = spi_device_driver::reg_read(reg::whoami);

   if ( val == val::whoami){
      sp::write("whoami succeeded\n");
   }else{
      sp::write("whoami failed\n");
      char buf[20];
      sprintf(buf,"got %u\n",static_cast<unsigned int>(val));
      sp::write(buf);
   }
}

void spi_setup()
{
   sp::write("spi setup\n");
   spi_device_driver::init();
   delay(ms{100});

   // reset
   spi_device_driver::reg_write(reg::pwr_mgmt1, 1U << 7U);
   delay(ms{100});
  
   // wakeup
 //  sp::write("spi wake up\n");
   spi_device_driver::reg_write(reg::pwr_mgmt1, 3U);
   delay(ms{100});

   // disable I2C
   spi_device_driver::reg_write(reg::user_ctrl, 1U << 4U);
   delay(ms{100});

   while (! whoami_test() )
   {
      delay(ms{100});
   }
  
   spi_device_driver::reg_write(reg::fifo_enable, 0U);
   delay(ms{1});

   spi_device_driver::reg_write(reg::sample_rate_div, 17);
   delay(ms{1});

   spi_device_driver::reg_write(reg::config, 0x04);
   delay(ms{1});

   spi_device_driver::reg_write(reg::gyro_config, 0x8);
   delay(ms{1});

   spi_device_driver::reg_write(reg::accel_config, 0x8);
   delay(ms{1});

   // want active low     bit 7 = true
   // hold until cleared   bit 5 = true
   spi_device_driver::reg_write(reg::intr_bypass_en_cfg, 0b10100000);
   delay(ms{1});

   // Should be initialised at startup
   // so check init linker flags etc
   spi_device_driver::dma_tx_buffer[0] = (reg::intr_status | 0x80);
   memset(spi_device_driver::dma_tx_buffer+1,0,15);

   spi_device_driver::enable_dma();
   spi_device_driver::set_bus_speed(1) ;
   // int on data ready
   spi_device_driver::reg_write(reg::intr_enable, 0b00000001);
}

// no irq version
extern "C" void read_imu()
{

   uint8_t arr[16];
   // start with interrupt status reg
   // so read clears irq
   arr[0] = reg::intr_status | 0x80;
   memcpy(arr+1,0,15);
   spi_device_driver::transaction(arr,arr,16);

   quan::three_d::vect<double> accel;
   quan::three_d::vect<double> gyro;

   asm volatile ("nop" :::);

   union{
      uint8_t arr[2];
      int16_t val;
   }u;  

   u.arr[1] = arr[2];
   u.arr[0] = arr[3];
   accel.x = u.val;

   u.arr[1] = arr[4];
   u.arr[0] = arr[5];
   accel.y = u.val;

   u.arr[1] = arr[6];
   u.arr[0] = arr[7];
   accel.z = u.val;

   char buf [120];
   sprintf(buf, "accel = [%.3f,%.3f,%.3f] \n",accel.x,accel.y,accel.z);
   sp::write(buf);

   u.arr[1] = arr[10];
   u.arr[0] = arr[11];
   gyro.x = u.val;

   u.arr[1] = arr[12];
   u.arr[0] = arr[13];
   gyro.y = u.val;
   u.arr[1] = arr[14];
   u.arr[0] = arr[15];
   gyro.z = u.val;

   sprintf(buf, "gyro = [%.3f,%.3f,%.3f]\n",gyro.x,gyro.y,gyro.z);
   sp::write(buf);
   
}

// Interrupt from MPU6000
extern "C" void EXTI15_10_IRQHandler() __attribute__ ((interrupt ("IRQ")));
extern "C" void EXTI15_10_IRQHandler()
{      
   // if data has been read by app then switch buffers
   // structure for accumulating samples
   // v filter
   if (quan::stm32::is_event_pending<spi_device_driver::mpu6000_irq>()){

      spi_device_driver::cs_assert(); // start transaction
      spi_device_driver::stop_spi();
      
      quan::stm32::clear_event_pending<spi_device_driver::mpu6000_irq>();
       
      DMA2_Stream5->NDTR = 16; // TX
      DMA2_Stream0->NDTR = 16; // RX

      DMA2_Stream0->CR |= (1 << 0); // (EN) enable DMA rx
      DMA2_Stream5->CR |= (1 << 0); // (EN) enable DMA  tx
      spi_device_driver::start_spi();
      
   }else{
      // other event?
   }
}

// RX DMA complete
extern "C" void DMA2_Stream0_IRQHandler() __attribute__ ((interrupt ("IRQ")));
extern "C" void DMA2_Stream0_IRQHandler()
{

   quan::three_d::vect<double> accel;
   quan::three_d::vect<double> gyro;

   DMA2_Stream5->CR &= ~(1 << 0); // (EN) disable DMA
   DMA2_Stream0->CR &= ~(1 << 0); // (EN) disable DMA



   union{
      uint8_t arr[2];
      int16_t val;
   }u;  

   volatile uint8_t * arr = spi_device_driver::dma_rx_buffer;

   u.arr[1] = arr[2];
   u.arr[0] = arr[3];
   accel.x = u.val;

   u.arr[1] = arr[4];
   u.arr[0] = arr[5];
   accel.y = u.val;

   u.arr[1] = arr[6];
   u.arr[0] = arr[7];
   accel.z = u.val;

   char buf [120];

   u.arr[1] = arr[10];
   u.arr[0] = arr[11];
   gyro.x = u.val;

   u.arr[1] = arr[12];
   u.arr[0] = arr[13];
   gyro.y = u.val;

   u.arr[1] = arr[14];
   u.arr[0] = arr[15];
   gyro.z = u.val;

   while(DMA2_Stream5->CR & (1 << 0)){;}
   while(DMA2_Stream0->CR & (1 << 0)){;}

   DMA2->HIFCR |= ( 0b111101 << 6) ; // Stream 5 clear flags
   DMA2->LIFCR |= ( 0b111101 << 0) ; // Stream 0 clear flags
   spi_device_driver::cs_release();

   sprintf(buf, "accel = [%.3f,%.3f,%.3f] \n",accel.x,accel.y,accel.z);
   sp::write(buf);
   sprintf(buf, "gyro = [%.3f,%.3f,%.3f]\n",gyro.x,gyro.y,gyro.z);
   sp::write(buf);
}













