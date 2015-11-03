/*
 *       Example of AP_BattMonitor library
 *       Code by DIYDrones.com
 */

#include <AP_Common/AP_Common.h>
#include <AP_Progmem/AP_Progmem.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>                // ArduPilot Mega Vector/Matrix math Library
#include <AP_ADC/AP_ADC.h>                 // ArduPilot Mega Analog to Digital Converter Library
#include <AP_ADC_AnalogSource/AP_ADC_AnalogSource.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL_AVR/AP_HAL_AVR.h>
#include <AP_HAL_PX4/AP_HAL_PX4.h>
#include <AP_HAL_Linux/AP_HAL_Linux.h>
#include <AP_HAL_Empty/AP_HAL_Empty.h>
#include <AP_HAL_Quan/AP_HAL_Quan.h>
#include <AP_BattMonitor/AP_BattMonitor.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <DataFlash/DataFlash.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_Baro/AP_Baro.h>
#include <Filter/Filter.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Compass/AP_Compass.h>
#include <AP_Declination/AP_Declination.h>
#include <AP_Airspeed/AP_Airspeed.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <AP_NavEKF/AP_NavEKF.h>
#include <AP_Notify/AP_Notify.h>
#include <AP_Mission/AP_Mission.h>
#include <StorageManager/StorageManager.h>
#include <AP_Terrain/AP_Terrain.h>
#include <AP_RangeFinder/AP_RangeFinder.h>

const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;
namespace {
   

   AP_BattMonitor battery_mon;
}

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
