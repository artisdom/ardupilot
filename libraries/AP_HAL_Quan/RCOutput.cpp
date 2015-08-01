
#include "RCOutput.h"

using namespace Quan;

void QuanRCOutput::init(void* machtnichts) {}

void QuanRCOutput::set_freq(uint32_t chmask, uint16_t freq_hz) {}

uint16_t QuanRCOutput::get_freq(uint8_t ch) {
    return 50;
}

void QuanRCOutput::enable_ch(uint8_t ch)
{}

void QuanRCOutput::disable_ch(uint8_t ch)
{}

void QuanRCOutput::write(uint8_t ch, uint16_t period_us)
{}

void QuanRCOutput::write(uint8_t ch, uint16_t* period_us, uint8_t len)
{}

uint16_t QuanRCOutput::read(uint8_t ch) {
    return 900;
}

void QuanRCOutput::read(uint16_t* period_us, uint8_t len)
{}

