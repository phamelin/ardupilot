#include <AP_HAL/AP_HAL.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_AHRS/AP_AHRS_NavEKF.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Notify/AP_BoardLED.h>

const AP_HAL::HAL &hal = AP_HAL::get_HAL();

// board specific config
AP_BoardConfig BoardConfig;
AP_BoardLED board_led;

// Setup a dummy vehicle to have a working AHRS
AP_InertialSensor ins;
Compass compass;
AP_GPS gps;
AP_Baro barometer;
AP_SerialManager serial_manager;

class DummyVehicle {
public:
    RangeFinder sonar {serial_manager, ROTATION_PITCH_270};
    AP_AHRS_NavEKF ahrs{ins, barometer, gps, sonar, EKF2, EKF3,
                        AP_AHRS_NavEKF::FLAG_ALWAYS_USE_EKF};
    NavEKF2 EKF2{&ahrs, barometer, sonar};
    NavEKF3 EKF3{&ahrs, barometer, sonar};
};

static DummyVehicle vehicle;

// choose which AHRS system to use
// AP_AHRS_DCM  ahrs(ins, baro, gps);
AP_AHRS_NavEKF ahrs(vehicle.ahrs);

void setup(void);
void loop(void);

void setup(void)
{
    // setup any board specific drivers
    BoardConfig.init();

    board_led.init();

    // wait for user input before beginning
    while (!hal.console->available()) {
        hal.scheduler->delay(20);
    }

    hal.console->printf("\n### AP_Compass_GPS test program ###\n\n");

    hal.console->printf("Loading parameters...");
    if(AP_Param::load_all(false))
    {
        hal.console->printf("Success!\n");
    }
    else
    {
        hal.console->printf("Failed!\n");
    }

    hal.console->printf("Initializing Compass...\n");
    compass.init();

    hal.console->printf("Initializing INS...\n");
    ins.init(100);

    hal.console->printf("Initializing AHRS...\n");
    ahrs.init();
    ahrs.set_compass(&compass);

    hal.console->printf("Initializing Serial Manager...\n");
    serial_manager.init();

    hal.console->printf("Initializing GPS...\n");
    gps.init(nullptr, serial_manager);

    // Display compass information
    hal.console->printf("Number of detected compass : %u\n", compass.get_count());
}

void loop(void)
{
    static uint32_t last_print, last_compass_gps;
    uint32_t now = AP_HAL::micros();

    ahrs.update();

    // Read compass and GPS at 10Hz
    if (now - last_compass_gps >= 100000 /* 100ms : 10hz */)
    {
        compass.read();
        gps.update();
        last_compass_gps = now;
    }

    if (now - last_print >= 200000 /* 200ms : 5hz */)
    {
        hal.console->printf("-----------------------------------------\n");
        for(uint8_t i=0; i<compass.get_count(); i++)
        {
            Vector3f field = compass.get_field(i);
            float heading = compass.calculate_heading(ahrs.get_rotation_body_to_ned(), i);
            hal.console->printf(
                    "COMPASS %u magx:%4.1f magy:%4.1f magz:%4.1f hdg:%4.1f healthy:%s\n",
                    (uint16_t)i,
                    (double)field.x,
                    (double)field.y,
                    (double)field.z,
                    (double)ToDeg(heading),
                    (compass.healthy(i) ? "YES" : "NO"));
        }

        hal.console->printf(
                "AHRS ATTITUDE r:%4.1f p:%4.1f y:%4.1f\n",
                (double)ToDeg(ahrs.roll),
                (double)ToDeg(ahrs.pitch),
                (double)ToDeg(ahrs.yaw));

        // Acquire location
        const Location &loc = gps.location();

        // Print the contents of message
        hal.console->printf("GPS ");
        hal.console->printf("Lat: ");
        print_latlon(hal.console, loc.lat);
        hal.console->printf(" Lon: ");
        print_latlon(hal.console, loc.lng);
        hal.console->printf(" Alt: %.2fm GSP: %.2fm/s CoG: %d SAT: %d TIM: %u/%lu STATUS: %u Hdg: %4.1f Hdg_ok:%s\n",
                            (double)(loc.alt * 0.01f),
                            (double)gps.ground_speed(),
                            (int)gps.ground_course_cd() / 100,
                            gps.num_sats(),
                            gps.time_week(),
                            (long unsigned int)gps.time_week_ms(),
                            gps.status(),
                            (double)gps.true_heading(),
                            (gps.true_heading_ok() ? "YES" : "NO"));
        last_print = now;
    }
}

GCS _gcs;

AP_HAL_MAIN();
