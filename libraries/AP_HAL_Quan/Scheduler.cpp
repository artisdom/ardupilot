
#include "Scheduler.h"

using namespace Quan;

extern const AP_HAL::HAL& hal;

QuanScheduler::QuanScheduler()
{}

void QuanScheduler::init(void* machtnichts)
{}

void QuanScheduler::delay(uint16_t ms)
{}

uint64_t QuanScheduler::millis64() {
    return 10000;
}

uint64_t QuanScheduler::micros64() {
    return 200000;
}

uint32_t QuanScheduler::millis() {
    return millis64();
}

uint32_t QuanScheduler::micros() {
    return micros64();
}

void QuanScheduler::delay_microseconds(uint16_t us)
{}

void QuanScheduler::register_delay_callback(AP_HAL::Proc k,
            uint16_t min_time_ms)
{}

void QuanScheduler::register_timer_process(AP_HAL::MemberProc k)
{}

void QuanScheduler::register_io_process(AP_HAL::MemberProc k)
{}

void QuanScheduler::register_timer_failsafe(AP_HAL::Proc, uint32_t period_us)
{}

void QuanScheduler::suspend_timer_procs()
{}

void QuanScheduler::resume_timer_procs()
{}

bool QuanScheduler::in_timerprocess() {
    return false;
}

void QuanScheduler::begin_atomic()
{}

void QuanScheduler::end_atomic()
{}

bool QuanScheduler::system_initializing() {
    return false;
}

void QuanScheduler::system_initialized()
{}

void QuanScheduler::panic(const prog_char_t *errormsg) {
    hal.console->println_P(errormsg);
    for(;;);
}

void QuanScheduler::reboot(bool hold_in_bootloader) {
    for(;;);
}
