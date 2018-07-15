
#include "FreeRTOS.h"
#include <task.h>
#include "i2c_task.hpp"
#include "sdp3x_i2c.hpp"

extern const AP_HAL::HAL& hal;

namespace Quan{

  bool sdp3x_exist_test()
  {

    uint8_t ar[2] = {(sdp3x_i2c_0::cmd::read_product_id[0] & 0xFF00) >> 8U,sdp3x_i2c_0::cmd::read_product_id[0] & 0xFF};
    if (! sdp3x_i2c_0::write(ar[0],ar[1])){
      hal.console->printf("sdp3x write cmd 0 failed\n");
      return false;
    }
    if (! Quan::wait_for_bus_free_ms(1)){
      hal.console->printf("sdp3x couldnt get bus after cmd 0 write\n");
      return false;
    }
    ar[0] = (sdp3x_i2c_0::cmd::read_product_id[1] & 0xFF00) >> 8U;
    ar[1] = sdp3x_i2c_0::cmd::read_product_id[1] & 0xFF;

    if (! sdp3x_i2c_0::write(ar[0],ar[1])){
      hal.console->printf("sdp3x write cmd 1 failed\n");
      return false;
    }
    if (! Quan::wait_for_bus_free_ms(1)){
      hal.console->printf("sdp3x couldnt get bus after cmd 1 write\n");
      return false;
    }
    // wait for command to complete
    return true;
    
  }

}



