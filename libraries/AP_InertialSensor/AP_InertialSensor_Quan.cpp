

#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_QUAN

#include <AP_HAL_Quan/imu_task.hpp>
#include "AP_InertialSensor_Quan.h"

AP_InertialSensor_Quan::AP_InertialSensor_Quan(AP_InertialSensor &imu)
: AP_InertialSensor_Backend(imu)
 ,m_accel_id{imu.register_accel()} 
 ,m_gyro_id{imu.register_gyro()}
{
  _product_id = AP_PRODUCT_ID_QUAN;
}

AP_InertialSensor_Backend * AP_InertialSensor_Quan::detect(AP_InertialSensor &imu)
{
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

void AP_InertialSensor_Quan::start()
{
   Quan::detail::spi_setup(
      get_sample_rate_hz(), 
      _accel_filter_cutoff(), 
      _gyro_filter_cutoff()
   );
   // running!
}

#endif // CONFIG_HAL_BOARD == HAL_BOARD_QUAN

