
#include "Semaphores.h"
#include "FreeRTOS.h"
#include <task.h>
#include <queue.h>

using namespace Quan;

bool QuanSemaphore::give() {
    if (_taken) {
        _taken = false;
        return true;
    } else {
        return false;
    }
}

bool QuanSemaphore::take(uint32_t timeout_ms) {
    return take_nonblocking();
}

bool QuanSemaphore::take_nonblocking() {
    /* No syncronisation primitives to garuntee this is correct */
    if (!_taken) {
        _taken = true;
        return true;
    } else {
        return false;
    }
}
