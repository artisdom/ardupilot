

#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_QUAN

#include <AP_HAL_Quan/imu_task.hpp>
#include "AP_InertialSensor_Quan.h"

extern const AP_HAL::HAL& hal;

AP_InertialSensor_Quan::AP_InertialSensor_Quan(AP_InertialSensor &imu)
: AP_InertialSensor_Backend(imu)
 ,m_accel_id{imu.register_accel(0)} 
 ,m_gyro_id{imu.register_gyro(0)}
{
  _product_id = AP_PRODUCT_ID_QUAN;
}

AP_InertialSensor_Backend * AP_InertialSensor_Quan::detect(AP_InertialSensor &imu)
{
// get some warning this has started !
   hal.console->printf("starting quan imu\n");
   Quan::detail::inertial_sensor_setup(
      imu.get_sample_rate(), 
      imu.get_accel_filter_hz(), 
      imu.get_gyro_filter_hz()   
   );
   hal.scheduler->delay(50);
   
   return new AP_InertialSensor_Quan(imu);
}

bool AP_InertialSensor_Quan::gyro_sample_available(void)
{
   return Quan::wait_for_imu_sample(0U);
}

bool AP_InertialSensor_Quan::accel_sample_available(void)
{
   return Quan::wait_for_imu_sample(0U);
}

bool AP_InertialSensor_Quan::update()
{
   Vector3f accel;
   Vector3f gyro;
   if ( Quan::update_ins(accel,gyro)){
      _rotate_and_correct_accel(m_accel_id,accel);
      _notify_new_accel_raw_sample(m_accel_id, accel);
      _publish_accel(m_accel_id,accel);
      _rotate_and_correct_gyro(m_gyro_id,gyro);
      _publish_gyro(m_gyro_id,gyro);
      return true;
   }else{
      return false;
   }
}

#endif // CONFIG_HAL_BOARD == HAL_BOARD_QUAN

