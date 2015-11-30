/*
  generic Baro driver test
 */

#include <AP_HAL/AP_HAL.h>
#include <AP_Baro/AP_Baro.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_QUAN
#include <quantracker/osd/osd.hpp>
#endif

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

static AP_Baro barometer;

#if CONFIG_HAL_BOARD != HAL_BOARD_QUAN
static uint32_t timer;
static uint8_t counter;
#endif

void setup()
{
    hal.console->printf("Barometer library test\n");

    hal.scheduler->delay(1000);

    barometer.init();
    hal.console->printf("Calibrating baro\n");
    barometer.calibrate();
    hal.console->printf("Done setup\n");


#if CONFIG_HAL_BOARD != HAL_BOARD_QUAN
// for Quan we use the hal.scheduler->delay fun rather than busy waiting
// on counters to avoid the apm_task saturating
    timer = AP_HAL::micros();
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
  // {
      uint32_t timer = AP_HAL::micros();
     // hal.console->printf("in apm baro task\n");
#else
    // run accumulate() at 50Hz and update() at 10Hz
    if((AP_HAL::micros() - timer) > 20*1000UL) {
        timer = AP_HAL::micros();

        barometer.accumulate();
        if (counter++ < 5) {
            return;
        }
        counter = 0;
#endif
        barometer.update();
        uint32_t read_time = AP_HAL::micros() - timer;
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
#if CONFIG_HAL_BOARD != HAL_BOARD_QUAN
    } else {
        hal.scheduler->delay(1);
    }
#endif
}

AP_HAL_MAIN();
