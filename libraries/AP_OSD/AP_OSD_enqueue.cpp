#include "AP_OSD_enqueue.h"

namespace AP_OSD{namespace dequeue{namespace detail{
   QueueHandle_t initialise();
}}}

// called by the sender when ready to start sending
namespace {
   // note this is a duplicate handle
   QueueHandle_t osd_queue = nullptr;
}

void AP_OSD::enqueue::initialise()
{
   osd_queue = AP_OSD::dequeue::detail::initialise();
}

namespace {

   bool queue_ready_for_msg()
   {
      return ( (osd_queue != nullptr) && (uxQueueSpacesAvailable(osd_queue) != 0) );
   }

   bool put_message( AP_OSD::msgID id, quan::three_d::vect<float> const & v)
   {
      if ( queue_ready_for_msg()){
          AP_OSD::osd_message_t msg;
          msg.id = id;
          msg.value.vect3df = v; 
          xQueueSendToBack(osd_queue,&msg,0);
         return true;
      }else{
         return false;
      }
   }

   bool put_message( AP_OSD::msgID id, float const & v)
   {
      if ( queue_ready_for_msg()){
          AP_OSD::osd_message_t msg;
          msg.id = id;
          msg.value.f = v;  
          xQueueSendToBack(osd_queue,&msg,0);
          return true;
      }else{
         return false;
      }
   }
}
   
bool AP_OSD::enqueue::attitude(quan::three_d::vect<float> const & in)
{
   return put_message(AP_OSD::msgID::attitude,in);
}

bool AP_OSD::enqueue::raw_compass(quan::three_d::vect<float> const & in)
{
  return put_message(AP_OSD::msgID::raw_compass,in);
}

bool AP_OSD::enqueue::drift(quan::three_d::vect<float> const & in)
{
   return put_message(AP_OSD::msgID::drift,in);
}

bool AP_OSD::enqueue::heading(float in)
{
   return put_message(AP_OSD::msgID::heading,in);
}