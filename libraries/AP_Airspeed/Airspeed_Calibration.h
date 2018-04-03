#ifndef AERFPILOT_LIBRARIES_AP_AIRSPEEED_AIRSPEED_CALIBRATION_H_INCLUDED
#define AERFPILOT_LIBRARIES_AP_AIRSPEEED_AIRSPEED_CALIBRATION_H_INCLUDED

#include <AP_Math/matrix3.h>
#include <AP_Math/vector3.h>
#include <AP_Vehicle/AP_Vehicle.h>

class Airspeed_Calibration {
public:
    friend class AP_Airspeed;
    // constructor
    Airspeed_Calibration(const AP_Vehicle::FixedWing &parms);

    // initialise the calibration
    void init(float initial_ratio);

    // take current airspeed in m/s and ground speed vector and return
    // new scaling factor
    float update(float airspeed, const Vector3f &vg);

private:
    // state of kalman filter for airspeed ratio estimation
    Matrix3f P; // covarience matrix
    const float Q0; // process noise matrix top left and middle element
    const float Q1; // process noise matrix bottom right element
    Vector3f state; // state vector
    const float DT; // time delta
    const AP_Vehicle::FixedWing &aparm;
};

#endif // AERFPILOT_LIBRARIES_AP_AIRSPEEED_AIRSPEED_CALIBRATION_H_INCLUDED
