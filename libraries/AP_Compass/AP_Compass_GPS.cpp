#include "AP_Compass_GPS.h"

#include <AP_HAL/AP_HAL.h>
#include "../../ArduCopter/Copter.h"

#include <unistd.h>

extern const AP_HAL::HAL &hal;

// constructor
AP_Compass_GPS::AP_Compass_GPS(Compass &compass):
    AP_Compass_Backend(compass),
    _instance(0),
    _ahrs(nullptr),
    _last_true_heading_ms(0)
{

}

// detect the sensor
AP_Compass_Backend *AP_Compass_GPS::detect(Compass &compass)
{
    AP_Compass_GPS *sensor = new AP_Compass_GPS(compass);
    if (sensor == nullptr) {
        return nullptr;
    }
    if (!sensor->init()) {
        delete sensor;
        return nullptr;
    }
    return sensor;
}

bool AP_Compass_GPS::init()
{
    _instance = register_compass();
    _compass._setup_earth_field();
    hal.console->printf("GPS compass initialized.\n");
    return true;
}

void AP_Compass_GPS::read()
{
    if(!_ahrs)
    {
        _ahrs = &copter.get_ahrs();
    }

    // Make sure that there's a new valid GPS true heading data available
    if(!_ahrs->get_gps().true_heading_ok() ||
            _ahrs->get_gps().true_heading_ms() <= _last_true_heading_ms)
    {
        // No valid data available
        return;
    }

    // Update earth field vector if declination has changed
    if (!is_equal(_compass._hil.last_declination, _compass.get_declination())) {
        _compass._setup_earth_field();
        _compass._hil.last_declination = _compass.get_declination();
    }

    float true_heading = radians(_ahrs->get_gps().true_heading());

    // Get the DCM of the actual attitude from the AHRS (body to NED)
    Matrix3f rbn = _ahrs->get_rotation_body_to_ned();

    // Correct the DCM with the actual true heading
    // TODO: Heading sensor offset is forced to zero for now
    Matrix3f rbn_corrected = fuse_true_heading(rbn, true_heading, 0.0f);

    // Convert the earth frame magnetic vector to body frame
    Vector3f field = rbn_corrected.mul_transpose(_compass._hil.Bearth);

    // Publish the new data
    publish_filtered_field(field, _instance);
}

Matrix3f AP_Compass_GPS::fuse_true_heading(const Matrix3f& body_to_ned,
            float true_heading, float sensor_offset)
{
    // Compute the vertical plane normal in NED which match the provided true
    // heading measurement from the sensor
    Vector3f dgps_vertical_plane_normal(-sinf(true_heading), cosf(true_heading), 0.0f);

    // Compute the normal vector of the x-y plane of the UAV in NED,
    // i.e. z-axis of the DCM.
    Vector3f uav_ne_plane_normal = body_to_ned.colz();

    // The heading sensor attitude must have the following axis:
    // - The z-axis is the same then the actual uav attitude, because we assume
    //      that the AHRS gives accurate roll/pitch with other sensors.
    // - The x-axis is the intersection of the x-y plane of the uav with the
    //      vertical plane corresponding to the provided true heading.
    // - The y-axis is orthogonal to x-z.
    Vector3f dgps_ned_x_axis = dgps_vertical_plane_normal % uav_ne_plane_normal;
    Vector3f dgps_ned_z_axis = uav_ne_plane_normal;
    Vector3f dgps_ned_y_axis = dgps_ned_z_axis % dgps_ned_x_axis;

    // Normalize axis
    dgps_ned_x_axis.normalize();
    dgps_ned_y_axis.normalize();
    dgps_ned_z_axis.normalize();

    // Compute the DCM of the heading sensor in NED
    Matrix3f r_dgps_ned(dgps_ned_x_axis.x, dgps_ned_y_axis.x, dgps_ned_z_axis.x,
                        dgps_ned_x_axis.y, dgps_ned_y_axis.y, dgps_ned_z_axis.y,
                        dgps_ned_x_axis.z, dgps_ned_y_axis.z, dgps_ned_z_axis.z);

    // Compute the DCM of the heading sensor in uav
    Matrix3f r_dgps_uav;
    r_dgps_uav.from_euler(0.0f, 0.0f, sensor_offset);

    // Compute the DCM of the uav in NED
    Matrix3f r_uav_ned = r_dgps_ned * r_dgps_uav.transposed();

    return r_uav_ned;
}
