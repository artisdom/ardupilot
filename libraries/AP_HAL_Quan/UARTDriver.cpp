
#include "UARTDriver.h"

using namespace Quan;

QuanUARTDriver::QuanUARTDriver() {}

void QuanUARTDriver::begin(uint32_t b) {}
void QuanUARTDriver::begin(uint32_t b, uint16_t rxS, uint16_t txS) {}
void QuanUARTDriver::end() {}
void QuanUARTDriver::flush() {}
bool QuanUARTDriver::is_initialized() { return false; }
void QuanUARTDriver::set_blocking_writes(bool blocking) {}
bool QuanUARTDriver::tx_pending() { return false; }

/* Quan implementations of Stream virtual methods */
int16_t QuanUARTDriver::available() { return 0; }
int16_t QuanUARTDriver::txspace() { return 1; }
int16_t QuanUARTDriver::read() { return -1; }

/* Quan implementations of Print virtual methods */
size_t QuanUARTDriver::write(uint8_t c) { return 0; }

size_t QuanUARTDriver::write(const uint8_t *buffer, size_t size)
{
    size_t n = 0;
    while (size--) {
        n += write(*buffer++);
    }
    return n;
}
