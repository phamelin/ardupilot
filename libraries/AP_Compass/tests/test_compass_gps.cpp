#include <AP_gtest.h>

#include <AP_Compass/AP_Compass_GPS.h>

float rot_to_heading(const Matrix3f& rot)
{
    return atan2f(rot.colx().y, rot.colx().x);
}


void test_fuse_true_heading(float roll, float pitch, float yaw, float sensor_offset)
{
    // Compute the uav DCM in NED
    Matrix3f r_uav_ned;
    r_uav_ned.from_euler(roll, pitch, yaw);

    // Compute the heading of the uav
    float true_heading = rot_to_heading(r_uav_ned);

    // Compute the GPS DCM in UAV
    Matrix3f r_gps_uav;
    r_gps_uav.from_euler(0.0f, 0.0f, sensor_offset);

    // Compute the GPS DCM in NED
    Matrix3f r_gps_ned = r_uav_ned * r_gps_uav;

    // Compute the heading measurement from GPS according to the sensor offset
    float true_heading_meas = rot_to_heading(r_gps_ned);

    // To confirm that the heading measurement fusing only use the z-axis from
    // the actual attitude, we apply a small rotation around the z-axis, which
    // must not affect the result.
    r_uav_ned.rotate(Vector3f(0.0f, 0.0f, radians(5.0f)));

    // Compute the updated DCM with the heading measurement
    Matrix3f r_uav_ned_fused = AP_Compass_GPS::fuse_true_heading(r_uav_ned, true_heading_meas, sensor_offset);

    // Compare computed DCM with the provided roll/pitch/yaw
    float roll_corrected, pitch_corrected, yaw_corrected;
    r_uav_ned_fused.to_euler(&roll_corrected, &pitch_corrected, &yaw_corrected);

    EXPECT_NEAR(roll, roll_corrected, 1e-6);
    EXPECT_NEAR(pitch, pitch_corrected, 1e-6);
    EXPECT_NEAR(yaw, yaw_corrected, 1e-6);
}

TEST(AP_COMPASS_GPS, fuse_true_heading)
{
    test_fuse_true_heading(radians(0.0f), radians(0.0f), radians(45.0f), radians(0.0f));
    test_fuse_true_heading(radians(0.0f), radians(0.0f), radians(-45.0f), radians(0.0f));
    test_fuse_true_heading(radians(0.0f), radians(0.0f), radians(45.0f), radians(30.0f));
    test_fuse_true_heading(radians(0.0f), radians(0.0f), radians(-45.0f), radians(30.0f));
    test_fuse_true_heading(radians(0.0f), radians(0.0f), radians(45.0f), radians(-30.0f));
    test_fuse_true_heading(radians(0.0f), radians(0.0f), radians(-45.0f), radians(-30.0f));

    test_fuse_true_heading(radians(25.0f), radians(0.0f), radians(45.0f), radians(0.0f));
    test_fuse_true_heading(radians(25.0f), radians(0.0f), radians(-45.0f), radians(0.0f));
    test_fuse_true_heading(radians(25.0f), radians(0.0f), radians(45.0f), radians(30.0f));
    test_fuse_true_heading(radians(25.0f), radians(0.0f), radians(-45.0f), radians(30.0f));
    test_fuse_true_heading(radians(25.0f), radians(0.0f), radians(45.0f), radians(-30.0f));
    test_fuse_true_heading(radians(25.0f), radians(0.0f), radians(-45.0f), radians(-30.0f));

    test_fuse_true_heading(radians(0.0f), radians(25.0f), radians(45.0f), radians(0.0f));
    test_fuse_true_heading(radians(0.0f), radians(25.0f), radians(-45.0f), radians(0.0f));
    test_fuse_true_heading(radians(0.0f), radians(25.0f), radians(45.0f), radians(30.0f));
    test_fuse_true_heading(radians(0.0f), radians(25.0f), radians(-45.0f), radians(30.0f));
    test_fuse_true_heading(radians(0.0f), radians(25.0f), radians(45.0f), radians(-30.0f));
    test_fuse_true_heading(radians(0.0f), radians(25.0f), radians(-45.0f), radians(-30.0f));

    test_fuse_true_heading(radians(25.0f), radians(35.0f), radians(45.0f), radians(0.0f));
    test_fuse_true_heading(radians(25.0f), radians(35.0f), radians(-45.0f), radians(0.0f));
    test_fuse_true_heading(radians(25.0f), radians(35.0f), radians(45.0f), radians(30.0f));
    test_fuse_true_heading(radians(25.0f), radians(35.0f), radians(-45.0f), radians(30.0f));
    test_fuse_true_heading(radians(25.0f), radians(35.0f), radians(45.0f), radians(-30.0f));
    test_fuse_true_heading(radians(25.0f), radians(35.0f), radians(-45.0f), radians(-30.0f));
}

AP_GTEST_MAIN()
