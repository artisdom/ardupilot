/*
  generic Baro driver test
 */

#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_QUAN
#include <AP_HAL_Quan/AP_HAL_Quan_Test_Main.h>
#include <AP_Baro/AP_Baro.h>
#include <quantracker/osd/osd.hpp>
#include <stdio.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

static AP_Baro barometer;

void setup()
{
    for ( uint8_t i = 1 ; i < 4; ++i){
       hal.gpio->pinMode(i,HAL_GPIO_OUTPUT);
       hal.gpio->write(i,0);
    }
    hal.console->printf("Barometer library test %lu\n", AP_HAL::millis());
//
//    hal.scheduler->delay(1000);
//
//    barometer.init();
//    hal.console->printf("Calibrating baro\n");
//    barometer.calibrate();
//    hal.console->printf("Done setup\n");
}

// do something on osd to check its running ok
void quan::uav::osd::on_draw() 
{ 
    pxp_type pos{-140,50};
    char buf[100];
    sprintf(buf,"Quan APM Baro Test : %lu", AP_HAL::millis());
    draw_text(buf,pos);
}

namespace {
   TickType_t prev_wake_time= 0; 
}

void loop()
{
   vTaskDelayUntil(&prev_wake_time,100); 

//   uint32_t timer = AP_HAL::micros();
//
//   barometer.update();
//   uint32_t read_time = AP_HAL::micros() - timer;
//   float alt = barometer.get_altitude();
//   if (!barometer.healthy()) {
//      hal.console->println("baro not healthy");
//      return;
//   }
//   hal.console->print("Pressure:");
//   hal.console->print(barometer.get_pressure());
//   hal.console->print(" Temperature:");
//   hal.console->print(barometer.get_temperature());
//   hal.console->print(" Altitude:");
//   hal.console->print(alt);
//   hal.console->printf(" climb=%.2f t=%u\n",
//                   static_cast<double>(barometer.get_climb_rate()),
//                   (unsigned)read_time);

}

namespace {
   uint32_t get_flags()
   {
      HAL_Quan::start_flags flags{0};
      flags.init_gpio = true;
      flags.init_scheduler = true;
      flags.init_uartA = true;
      flags.init_i2c = true;
      return flags.value;
   }
}

AP_HAL_TEST_MAIN( get_flags() )

#endif
