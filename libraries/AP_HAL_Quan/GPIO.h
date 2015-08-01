
#ifndef __AP_HAL_QUAN_GPIO_H__
#define __AP_HAL_QUAN_GPIO_H__

#include <AP_HAL_Quan.h>


   class Quan::QuanGPIO : public AP_HAL::GPIO {
   public:
       QuanGPIO();
       void    init();
       void    pinMode(uint8_t pin, uint8_t output);
       int8_t  analogPinToDigitalPin(uint8_t pin);
       uint8_t read(uint8_t pin);
       void    write(uint8_t pin, uint8_t value);
       void    toggle(uint8_t pin);

       /* Alternative interface: */
       AP_HAL::DigitalSource* channel(uint16_t n);

       /* Interrupt interface: */
       bool    attach_interrupt(uint8_t interrupt_num, AP_HAL::Proc p,
               uint8_t mode);

       /* return true if USB cable is connected */
       bool    usb_connected(void);
   };

   struct Quan::QuanDigitalSource : public AP_HAL::DigitalSource {
     virtual void init() = 0;
//       QuanDigitalSource(uint8_t v);
//       void    mode(uint8_t output);
//       uint8_t read();
//       void    write(uint8_t value); 
//       void    toggle();
//   private:
      // uint8_t _v;
   };



#endif // __AP_HAL_QUAN_GPIO_H__
