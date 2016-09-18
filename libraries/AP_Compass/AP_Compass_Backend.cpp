/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_HAL/AP_HAL.h>

#include "AP_Compass.h"
#include "AP_Compass_Backend.h"

extern const AP_HAL::HAL& hal;

AP_Compass_Backend::AP_Compass_Backend(Compass &compass, const char* name, uint8_t index, bool external) :
    _compass{compass}, _name{name},_index{index},_is_external{external},_last_update_usec{0U}
{
  if (index >= compass.max_backends){
     AP_HAL::panic("array index out of range in AP_Compass_Backend");
  }
}


bool AP_Compass_Backend::install(){ return _compass._add_backend(*this);}

bool AP_Compass_Backend::is_installed() const { return _compass._backends[_index] == this;}

void AP_Compass_Backend::rotate_field(Vector3f &mag)
{
    Compass::mag_state &state = _compass._state[_index];
    mag.rotate(get_compass_orientation<AP_HAL::Tag_BoardType>());

    if (! _is_external) {
        // and add in AHRS_ORIENTATION setting if not an external compass
        mag.rotate(_compass._board_orientation);
    } else {
        // add user selectable orientation
        mag.rotate((enum Rotation)state.orientation.get());
    }
}

void AP_Compass_Backend::publish_raw_field(const Vector3f &mag, uint32_t time_us)
{
    Compass::mag_state &state = _compass._state[_index];

    // note that we do not set last_update_usec here as otherwise the
    // EKF and DCM would end up consuming compass data at the full
    // sensor rate. We want them to consume only the filtered fields
    state.last_update_ms = AP_HAL::millis();

    _compass._calibrator[_index].new_sample(mag);
}

void AP_Compass_Backend::correct_field(Vector3f &mag)
{
    Compass::mag_state &state = _compass._state[_index];

    if (state.diagonals.get().is_zero()) {
        state.diagonals.set(Vector3f(1.0f,1.0f,1.0f));
    }

    const Vector3f &offsets = state.offset.get();
    const Vector3f &diagonals = state.diagonals.get();
    const Vector3f &offdiagonals = state.offdiagonals.get();
    const Vector3f &mot = state.motor_compensation.get();

    /*
     * note that _motor_offset[] is kept even if compensation is not
     * being applied so it can be logged correctly
     */
    mag += offsets;
    if(_compass.get_motor_compensation_type() != Compass::Motor_compensation_type::Disabled && !is_zero(_compass._thr_or_curr)) {
        state.motor_offset = mot * _compass._thr_or_curr;
        mag += state.motor_offset;
    } else {
        state.motor_offset.zero();
    }

    Matrix3f mat(
        diagonals.x, offdiagonals.x, offdiagonals.y,
        offdiagonals.x,    diagonals.y, offdiagonals.z,
        offdiagonals.y, offdiagonals.z,    diagonals.z
    );

    mag = mat * mag;
}

/*
  copy latest data to the frontend from a backend
 */
void AP_Compass_Backend::publish_filtered_field(const Vector3f &mag)
{
    Compass::mag_state &state = _compass._state[_index];

    state.field = mag;

    state.last_update_ms = AP_HAL::millis();
   // state.last_update_usec = AP_HAL::micros();
    set_last_update_usec(AP_HAL::micros());
}

//void AP_Compass_Backend::set_last_update_usec(uint32_t last_update)
//{
//   // Compass::mag_state &state = _compass._state[_index];
//   // state.last_update_usec = last_update;
//   
//}


/*
  set dev_id for an instance
*/
void AP_Compass_Backend::set_dev_id(uint32_t dev_id)
{
    _compass._state[_index].dev_id.set(dev_id);
}

