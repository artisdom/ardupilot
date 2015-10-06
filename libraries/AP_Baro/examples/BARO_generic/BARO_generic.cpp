/*
  generic Baro driver test
 */
#include <AP_Common/AP_Common.h>
#include <AP_Progmem/AP_Progmem.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Buffer/AP_Buffer.h>
#include <Filter/Filter.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_Notify/AP_Notify.h>
#include <AP_GPS/AP_GPS.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <DataFlash/DataFlash.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_Mission/AP_Mission.h>
#include <StorageManager/StorageManager.h>
#include <AP_Terrain/AP_Terrain.h>
#include <AP_ADC/AP_ADC.h>
#include <AP_ADC_AnalogSource/AP_ADC_AnalogSource.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Compass/AP_Compass.h>
#include <AP_Declination/AP_Declination.h>
#include <AP_Airspeed/AP_Airspeed.h>
#include <AP_HAL_AVR/AP_HAL_AVR.h>
#include <AP_HAL_SITL/AP_HAL_SITL.h>
#include <AP_HAL_Linux/AP_HAL_Linux.h>
#include <AP_HAL_FLYMAPLE/AP_HAL_FLYMAPLE.h>
#include <AP_HAL_PX4/AP_HAL_PX4.h>
#include <AP_HAL_Quan/AP_HAL_Quan.h>
#include <AP_HAL_Empty/AP_HAL_Empty.h>
#include <AP_Rally/AP_Rally.h>
#include <AP_NavEKF/AP_NavEKF.h>
#include <AP_Scheduler/AP_Scheduler.h>
#include <AP_BattMonitor/AP_BattMonitor.h>
#include <AP_RangeFinder/AP_RangeFinder.h>

const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

static AP_Baro barometer;

#if CONFIG_HAL_BOARD != HAL_BOARD_QUAN
static uint32_t timer;
static uint8_t counter;
#endif

void setup()
{
    hal.console->printf("Barometer library test\n");

    hal.scheduler->delay(1000);

#if CONFIG_HAL_BOARD == HAL_BOARD_APM2
    // disable CS on MPU6000
    hal.gpio->pinMode(63, HAL_GPIO_OUTPUT);
    hal.gpio->write(63, 1);
#endif
    hal.console->printf("Initing baro\n");
    barometer.init();
    hal.console->printf("Calibrating baro\n");
    barometer.calibrate();
    hal.console->printf("Done setup\n");


#if CONFIG_HAL_BOARD != HAL_BOARD_QUAN
// for Quan we use the hal.scheduler->delay fun rather than busy waiting
// on counters to avoid the apm_task saturating
    timer = hal.scheduler->micros();
 #endif
}

#if CONFIG_HAL_BOARD == HAL_BOARD_QUAN
// do something on osd to check its running ok
void quan::uav::osd::on_draw() 
{ 
    pxp_type pos{-140,50};
    draw_text("Quan APM Baro Test",pos);
}
#endif

void loop()
{
      
#if CONFIG_HAL_BOARD == HAL_BOARD_QUAN
   hal.scheduler->delay(100);  // just run at 10 Hz. for Quan baro.accumulate is a nop
   {
      uint32_t timer = hal.scheduler->micros();
      hal.console->printf("in apm baro task\n");
#else
    // run accumulate() at 50Hz and update() at 10Hz
    if((hal.scheduler->micros() - timer) > 20*1000UL) {
        timer = hal.scheduler->micros();

        barometer.accumulate();
        if (counter++ < 5) {
            return;
        }
        counter = 0;
#endif
        barometer.update();
        uint32_t read_time = hal.scheduler->micros() - timer;
        float alt = barometer.get_altitude();
        if (!barometer.healthy()) {
            hal.console->println("baro not healthy");
            return;
        }
        hal.console->print("Pressure:");
        hal.console->print(barometer.get_pressure());
        hal.console->print(" Temperature:");
        hal.console->print(barometer.get_temperature());
        hal.console->print(" Altitude:");
        hal.console->print(alt);
        hal.console->printf(" climb=%.2f t=%u",
                            static_cast<double>(barometer.get_climb_rate()),
                            (unsigned)read_time);
        hal.console->println();
    }
}

AP_HAL_MAIN();
