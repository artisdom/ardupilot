/*
 *       Example of AP_BattMonitor library
 *       Code by DIYDrones.com
 */


#include <AP_BattMonitor/AP_BattMonitor.h>
#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_QUAN
#include <quantracker/osd/osd.hpp>
#endif

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

AP_BattMonitor battery_mon;

#if CONFIG_HAL_BOARD == HAL_BOARD_QUAN
// do something on osd to check its running ok
void quan::uav::osd::on_draw() 
{ 
    pxp_type pos{-140,50};
    draw_text("Quan APM Battery monitor test",pos);
}
#endif

void setup() {
    hal.console->println("Battery monitor library test");

// 
    // set battery monitor to smbus
   // battery_mon.set_monitoring(0, AP_BattMonitor::BattMonitor_TYPE_SMBUS);
    // initialise the battery monitor
    battery_mon.init();

    hal.scheduler->delay(1000);
}

namespace {
   uint32_t counter =0;
}

void loop()
{
    hal.scheduler->delay(100);

    battery_mon.read();

    // display output at 1hz
    if (++counter >= 10) {
        counter = 0;
        hal.console->printf("\nVoltage: %.2f V \tCurrent: %.2f A \tTotCurr:%.2f mAh",
			    battery_mon.voltage(),
			    battery_mon.current_amps(),
                battery_mon.current_total_mah());
    }
}

AP_HAL_MAIN();
