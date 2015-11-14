// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

//
// Simple test for the AP_AHRS interface
//

#include <AP_ADC/AP_ADC.h>
#include <AP_ADC_AnalogSource/AP_ADC_AnalogSource.h>
#include <AP_AHRS/AP_AHRS.h>

#include <AP_HAL/AP_HAL.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();


// INS and Baro declaration
AP_InertialSensor ins;

Compass compass;

AP_GPS gps;
AP_Baro baro;
AP_SerialManager serial_manager;
AP_BattMonitor  battery_monitor;

static AP_Vehicle::FixedWing aparm;

AP_Airspeed airspeed(aparm);

// choose which AHRS system to use
AP_AHRS_DCM  ahrs(ins, baro, gps);

#define HIGH 1
#define LOW 0

void setup(void)
{

    ins.init(AP_InertialSensor::RATE_100HZ);

    ahrs.init();
    serial_manager.init();
    AP_OSD::enqueue::initialise();
    baro.init();
    baro.calibrate();

    airspeed.init();
    airspeed.calibrate(false);
    
    if( compass.init() ) {
        hal.console->printf("Enabling compass\n");
        compass.set_offsets(0, {10.6004,-35.4656,-48.9482});
        ahrs.set_compass(&compass);
    } else {
        hal.console->printf("No compass detected\n");
    }
    gps.init(NULL, serial_manager);
 
}

namespace {

  uint16_t print_counter =0;
  uint16_t counter10_Hz;
 
}

void loop(void)
{
    
    ins.wait_for_sample();
    float heading = 0;
    if (++counter10_Hz == 5){
        counter10_Hz = 0;
        compass.read();
        heading = compass.calculate_heading(ahrs.get_dcm_matrix());
        AP_OSD::enqueue::heading(ToDeg(heading));

        baro.update();
        AP_OSD::enqueue::baro_altitude(baro.get_altitude());

        gps.update();
        AP_OSD::enqueue::gps_status(gps.status());
        AP_OSD::enqueue::gps_location(
            {gps.location().lat,gps.location().lng,gps.location().alt}
        );

        airspeed.read();
        AP_OSD::enqueue::airspeed(airspeed.get_airspeed());

        battery_monitor.read();
        AP_OSD::enqueue::battery(
            {battery_monitor.voltage(),
               battery_monitor.current_amps(),
                  battery_monitor.current_total_mah()}
        );

    }

    ahrs.update();
#if CONFIG_HAL_BOARD != HAL_BOARD_QUAN
    counter++;

    if (now - last_print >= 100000 /* 100ms : 10hz */) {
        Vector3f drift  = ahrs.get_gyro_drift();
        hal.console->printf(
                "r:%4.1f  p:%4.1f y:%4.1f "
                    "drift=(%5.1f %5.1f %5.1f) hdg=%.1f rate=%.1f\n",
                        ToDeg(ahrs.roll),
                        ToDeg(ahrs.pitch),
                        ToDeg(ahrs.yaw),
                        ToDeg(drift.x),
                        ToDeg(drift.y),
                        ToDeg(drift.z),
                        compass.use_for_yaw() ? ToDeg(heading) : 0.0f,
                        (1.0e6f*counter)/(now-last_print));
        last_print = now;
        counter = 0;
    }
#else
    AP_OSD::enqueue::attitude({ToDeg(ahrs.pitch),ToDeg(ahrs.roll),ToDeg(ahrs.yaw)});

    if (++print_counter == 20) {
      print_counter = 0;
      Vector3f drift  = ahrs.get_gyro_drift();
      hal.console->printf_P(
         PSTR("r:%4.1f  p:%4.1f y:%4.1f "
         "drift=(%5.1f %5.1f %5.1f) hdg=%.1f \n"),
            static_cast<double>(ToDeg(ahrs.roll)),
            static_cast<double>(ToDeg(ahrs.pitch)),
            static_cast<double>(ToDeg(ahrs.yaw)),
            static_cast<double>(ToDeg(drift.x)),
            static_cast<double>(ToDeg(drift.y)),
            static_cast<double>(ToDeg(drift.z)),
            compass.use_for_yaw() ? static_cast<double>(ToDeg(heading)) : 0.0
      );
   }
#endif
}

AP_HAL_MAIN();
