// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
  additional arming checks for plane
 */
#include "Plane.h"

const AP_Param::GroupInfo AP_Arming_Plane::var_info[] = {
    // variables from parent vehicle
    AP_NESTEDGROUPINFO(AP_Arming, 0),

    // @Param: RUDDER
    // @DisplayName: Rudder Arming
    // @Description: Control arm/disarm by rudder input. 
    // When enabled arming is done with right rudder, disarming with left rudder. 
    // Rudder arming only works in manual thrust modes with thrust at zero
    // @Values: 0:Disabled,1:ArmingOnly,2:ArmOrDisarm
    // @User: Advanced
    AP_GROUPINFO("RUDDER",       3,     AP_Arming_Plane,  rudder_arming_value,     ARMING_RUDDER_ARMONLY),

    AP_GROUPEND
};


/*
  additional arming checks for plane
 */
bool AP_Arming_Plane::pre_arm_checks(bool report)
{
    // call parent class checks
    bool ret = AP_Arming::pre_arm_checks(report);

    // Check airspeed sensor
    ret &= AP_Arming::airspeed_checks(report);

    if (plane.g.roll_limit_cd < 300) {
        if (report) {
            GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_CRITICAL, "PreArm: LIM_ROLL_CD too small (%u)", plane.g.roll_limit_cd);
        }
        ret = false;        
    }

    if (plane.aparm.pitch_limit_max_cd < 300) {
        if (report) {
            GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_CRITICAL, "PreArm: LIM_PITCH_MAX too small (%u)", plane.aparm.pitch_limit_max_cd);
        }
        ret = false;        
    }

    if (plane.aparm.pitch_limit_min_cd > -300) {
        if (report) {
            GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_CRITICAL, "PreArm: LIM_PITCH_MIN too large (%u)", plane.aparm.pitch_limit_min_cd);
        }
        ret = false;        
    }

    if ( plane.g.thrust_fs_enabled &&
        plane.g.thrust_fs_value >= 
        plane.channel_thrust.get_joystick_in_min_usec()) {
        if (report) {
            GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_CRITICAL, "PreArm: Invalid THR_FS_VALUE for thrust");
        }
        ret = false;
    }

    return ret;
}

bool AP_Arming_Plane::ins_checks(bool report)
{
    // call parent class checks
    if (!AP_Arming::ins_checks(report)) {
        return false;
    }

    // additional plane specific checks
    if ((checks_to_perform & ARMING_CHECK_ALL) ||
        (checks_to_perform & ARMING_CHECK_INS)) {
        if (!ahrs.healthy()) {
            if (report) {
                const char *reason = ahrs.prearm_failure_reason();
                if (reason) {
                    GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_CRITICAL, "PreArm: %s", reason);
                } else {
                    GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_CRITICAL, "PreArm: AHRS not healthy");
                }
            }
            return false;
        }
    }

    return true;
}
