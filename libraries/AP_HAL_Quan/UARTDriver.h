
#ifndef __AP_HAL_QUAN_UARTDRIVER_H__
#define __AP_HAL_QUAN_UARTDRIVER_H__

#include <AP_HAL_Quan.h>

//template <typename Usart>
//class Quan::QuanUARTDriver : public AP_HAL::UARTDriver {
//public:
//    QuanUARTDriver();
//    /* Quan implementations of UARTDriver virtual methods */
//    void begin(uint32_t baudrate);
//    void begin(uint32_t baudrate, uint16_t rxBufferSize, uint16_t TxBufferSize);
//    void end();
//    void flush();
//    bool is_initialized();
//    void set_blocking_writes(bool blocking);
//    bool tx_pending();
//
//    /* Quan implementations of Stream virtual methods */
//    int16_t available();
//    int16_t txspace();
//    int16_t read();
//
//    /* Quan implementations of Print virtual methods */
//    size_t write(uint8_t c);
//    size_t write(const uint8_t *buffer, size_t size);
//};
//
//#endif // __AP_HAL_QUAN_UARTDRIVER_H__

namespace Quan{
   AP_HAL::UARTDriver * get_serial_port(uint32_t i);
}

#endif
