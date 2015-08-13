
#if CONFIG_HAL_BOARD == HAL_BOARD_QUAN
#include "Plane.h"

#include <quan/where.hpp>
#include <task.h>
#include <quantracker/osd/osd.hpp>

/*
  Since The osd is in a different thread
  Need to provide mutual exclusion.
  Most things are only read by the osd thread, not written
  OK unless interrupts involved!
*/

namespace quan{ namespace stm32{ namespace freertos{

  template <typename T>
  inline
  typename quan::where_c< (sizeof(T) > 4)>,T >::type 
  atomic_read( T const & value)
  {
      taskENTER_CRITICAL();
      T result = value;
      taskEXIT_CRITICAL();
      return result;
  }

  template <typename T>
  inline
  typename quan::where_c<(sizeof(T) <= 4)>,T const & >::type 
  atomic_read( T const & value)
  {
     return value;
  }

}}} // quan::stm32::freertos

void quan::uav::osd::on_draw() 
{ 
   quan::uav::osd::draw_text("Hello APM World again",{-100,0}); 
}

#endif  // #if CONFIG_HAL_BOARD == HAL_BOARD_QUAN