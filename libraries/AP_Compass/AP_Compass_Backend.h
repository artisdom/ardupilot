// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
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
  Compass driver backend class. Each supported compass sensor type
  needs to have an object derived from this class.
 */
#pragma once

#include "AP_Compass.h"

class Compass; 

class AP_Compass_Backend
{
protected:
    AP_Compass_Backend(Compass &compass, const char* name_in, uint8_t idx, bool is_external);
public:
    // we declare a virtual destructor so that drivers can
    // override with a custom destructor if need be.
    virtual ~AP_Compass_Backend(void) {}

    // initialize the magnetometers
    virtual bool init(void) = 0;

    // read sensor data
    virtual void read(void) = 0;

    // accumulate a reading from the magnetometer. Optional in
    // backends
    virtual void accumulate(void) {};
    uint8_t get_index() const {return _index;}
    // tell if instance is an external compass
    void set_external ( bool val){_is_external = val;}
    bool install();
    bool is_installed() const ;
    bool is_external()const { return _is_external;}
    const char* get_name() const { return _name;}
protected:

    /*
     * A compass measurement is expected to pass through the following functions:
     * 1. rotate_field - this rotates the measurement in-place from sensor frame
     *      to body frame
     * 2. publish_raw_field - this provides an uncorrected point-sample for
     *      calibration libraries
     * 3. correct_field - this corrects the measurement in-place for hard iron,
     *      soft iron, motor interference, and non-orthagonality errors
     * 4. publish_filtered_field - legacy filtered magnetic field
     *
     * All those functions expect the mag field to be in milligauss.
     */

    void rotate_field(Vector3f &mag);
    void publish_raw_field(const Vector3f &mag, uint32_t time_us);
    void correct_field(Vector3f &mag);
    void publish_filtered_field(const Vector3f &mag);
    void set_last_update_usec(uint32_t last_update);

   // register a new compass instance with the frontend
   // done in ctor
   // uint8_t register_compass(void) const;

    // set dev_id for an instance
    // only if it is installed in the front end
    void set_dev_id(uint32_t dev_id);

    // access to frontend
    Compass &_compass;
    
private:
    void apply_corrections(Vector3f &mag);
    const char* _name;
    uint8_t _index; 
    bool _is_external;

};
