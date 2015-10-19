/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 *   Airspeed.pde - airspeed example sketch
 *
 */
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL_Quan/AP_HAL_Quan.h>
#include <AP_Airspeed/AP_Airspeed.h>
#include <AP_Vehicle/AP_Vehicle.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_APM1
AP_ADC_ADS7844 apm1_adc;
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_QUAN

// do something on osd to check its running ok
void quan::uav::osd::on_draw() 
{ 
    pxp_type pos{-140,50};
    draw_text("Quan APM Airspeed Test",pos);
}

#endif

const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

static AP_Vehicle::FixedWing aparm;

AP_Airspeed airspeed(aparm);

void setup()
{
    hal.console->println("ArduPilot Airspeed library test");
    airspeed.init();
    airspeed.calibrate(false);
}

void loop(void)
{
#if CONFIG_HAL_BOARD == HAL_BOARD_QUAN
     hal.scheduler->delay(100);
#else
    static uint32_t timer;
    if((hal.scheduler->millis() - timer) > 100) {
        timer = hal.scheduler->millis();
#endif
        airspeed.read();
        hal.console->printf("airspeed %8.3f\n", static_cast<double>(airspeed.get_airspeed()));
#if CONFIG_HAL_BOARD != HAL_BOARD_QUAN
    }
    hal.scheduler->delay(1);
#endif
}

AP_HAL_MAIN();
