#pragma once

#include "AP_Compass.h"
#include <AP_Math/AP_Math.h>

// Forward declarations
class AP_AHRS;

/**
 * GPS-based compass backend.
 *
 * This compass driver uses the true heading information coming from a GPS. This
 * is typically supported by dual antenna GPS.
 */
class AP_Compass_GPS : public AP_Compass_Backend
{
public:

    AP_Compass_GPS(Compass &compass);
    void read();
    bool init();

    // detect the sensor
    static AP_Compass_Backend *detect(Compass &compass);

    static Matrix3f fuse_true_heading(const Matrix3f& body_to_ned,
            float true_heading, float sensor_offset);

private:

    // Compass instance id
    uint8_t _instance;

    // Last GPS message timestamp
    uint32_t _last_true_heading_ms;
};
