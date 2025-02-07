#include "mission_item_impl.h"
#include "global_include.h"
#include "log.h"
#include <cmath>

namespace mavsdk {

MissionItemImpl::MissionItemImpl() {}

MissionItemImpl::~MissionItemImpl() {}

void MissionItemImpl::set_position(double latitude_deg, double longitude_deg)
{
    _latitude_deg = latitude_deg;
    _longitude_deg = longitude_deg;
}

void MissionItemImpl::set_relative_altitude(float relative_altitude_m)
{
    _relative_altitude_m = relative_altitude_m;
}

void MissionItemImpl::set_speed(float speed_m_s)
{
    _speed_m_s = speed_m_s;
}

void MissionItemImpl::set_fly_through(bool fly_through)
{
    _fly_through = fly_through;
}

void MissionItemImpl::set_acceptance_radius(float radius_m)
{
    _acceptance_radius_m = radius_m;
}

void MissionItemImpl::set_gimbal_pitch_and_yaw(float pitch_deg, float yaw_deg)
{
    _gimbal_pitch_deg = pitch_deg;
    _gimbal_yaw_deg = yaw_deg;
}

void MissionItemImpl::set_loiter_time(float loiter_time_s)
{
    _loiter_time_s = loiter_time_s;
}

void MissionItemImpl::set_camera_action(MissionItem::CameraAction action)
{
    _camera_action = action;
}

void MissionItemImpl::set_camera_photo_interval(double interval_s)
{
    if (interval_s > 0.0) {
        _camera_photo_interval_s = interval_s;
    } else {
        LogWarn() << "Invalid interval argument";
    }
}

void MissionItemImpl::set_mavlink_param3(float payloadWeight)
{
    _param3 = payloadWeight;
}
    
void MissionItemImpl::set_mavlink_param4(float deadline)
{
    _param4 = deadline;
}

MAV_FRAME MissionItemImpl::get_mavlink_frame() const
{
    return MAV_FRAME_GLOBAL_RELATIVE_ALT_INT;
}

MAV_CMD MissionItemImpl::get_mavlink_cmd() const
{
    return MAV_CMD_NAV_WAYPOINT;
}

uint8_t MissionItemImpl::get_mavlink_autocontinue() const
{
    return 1;
}

float MissionItemImpl::get_mavlink_param1() const
{
    float hold_time_s;
    if (_fly_through) {
        hold_time_s = 0.0f;
    } else {
        hold_time_s = 0.5f;
    }

    return hold_time_s;
}

float MissionItemImpl::get_mavlink_param2() const
{
    float acceptance_radius_m;
    if (std::isfinite(_acceptance_radius_m)) {
        acceptance_radius_m = _acceptance_radius_m;
    } else if (_fly_through) {
        // _acceptance_radius_m is 0, determine the radius using fly_through
        acceptance_radius_m = 3.0f;
    } else {
        // _acceptance_radius_m is 0, determine the radius using fly_through
        acceptance_radius_m = 1.0f;
    }

    return acceptance_radius_m;
}

float MissionItemImpl::get_mavlink_param3() 
{
    return _param3;
    //return 0.0f;
}

float MissionItemImpl::get_mavlink_param4() 
{
    // Just let the drone fly forward.
    return _param4;
    
    /* FIXME set relevant parameter as NAN in PX4 */
    //float yaw_angle_deg = NAN;
    //return yaw_angle_deg;
}

int32_t MissionItemImpl::get_mavlink_x() const
{
    return int32_t(std::round(_latitude_deg * 1e7));
}

int32_t MissionItemImpl::get_mavlink_y() const
{
    return int32_t(std::round(_longitude_deg * 1e7));
}

float MissionItemImpl::get_mavlink_z() const
{
    return _relative_altitude_m;
}

bool MissionItemImpl::is_position_finite() const
{
    return std::isfinite(_latitude_deg) && std::isfinite(_longitude_deg) &&
           std::isfinite(_relative_altitude_m);
}

} // namespace mavsdk
