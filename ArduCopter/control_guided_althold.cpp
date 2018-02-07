#include "Copter.h"

// GUIDED_ALTHOLD mode's times out after 1 second with no updates
#define GUIDED_ALTHOLD_TIMEOUT_MS 1000

// Smoothing gain for attitude control (12 = max)
#define GUIDED_ALTHOLD_SMOOTHING_GAIN 12.0f

// Guided althold state variables
struct
{
    // Target roll angle in centi-degrees
    float target_roll_cd;

    // Flag to use roll command
    bool use_roll;

    // Target pitch angle in centi-degrees
    float target_pitch_cd;

    // Flag to use pitch command
    bool use_pitch;

    // Target yaw rate in centi-degrees/sec
    float target_yaw_rate_cds;

    // Flag to use yaw rate
    bool use_yaw_rate;

    // Last update timestamp in milliseconds
    uint64_t update_time_ms;

    // Will be set to true if there's no communication with external computer
    bool timeout;
} static guided_althold_state;

/*
 * Init and run calls for guided althold, flight mode
 */

// guided_althold_init - initialise guided althold controller
bool Copter::guided_althold_init(bool ignore_checks)
{
    // initialize states
    guided_althold_state.target_roll_cd = 0.0f;
    guided_althold_state.use_roll = false;
    guided_althold_state.target_pitch_cd = 0.0f;
    guided_althold_state.use_pitch = false;
    guided_althold_state.target_yaw_rate_cds = 0.0f;
    guided_althold_state.use_yaw_rate = false;
    guided_althold_state.update_time_ms = 0;
    guided_althold_state.timeout = true;

    // initialize vertical speeds and leash lengths
    pos_control->set_speed_z(-g.pilot_velocity_z_max, g.pilot_velocity_z_max);
    pos_control->set_accel_z(g.pilot_accel_z);

    // initialise position and desired velocity
    if (!pos_control->is_active_z()) {
        pos_control->set_alt_target_to_current_alt();
        pos_control->set_desired_velocity_z(inertial_nav.get_velocity_z());
    }

    // stop takeoff if running
    takeoff_stop();

    return true;
}

// guided_althold_run - runs the guided althold controller
// should be called at 100hz or more
void Copter::guided_althold_run()
{
    AltHoldModeState althold_state;
    float takeoff_climb_rate = 0.0f;

    // Force disable remote pitch control for now
    guided_althold_state.use_pitch = false;

    // Check for external computer timeout
    guided_althold_check_timeout();

    // initialize vertical speeds and acceleration
    pos_control->set_speed_z(-g.pilot_velocity_z_max, g.pilot_velocity_z_max);
    pos_control->set_accel_z(g.pilot_accel_z);

    // apply SIMPLE mode transform to pilot inputs
    update_simple_mode();

    // get pilot desired lean angles
    float target_roll, target_pitch;
    get_pilot_desired_lean_angles(channel_roll->get_control_in(), channel_pitch->get_control_in(), target_roll, target_pitch, attitude_control->get_althold_lean_angle_max());

    if(guided_althold_state.use_roll)
    {
        target_roll = guided_althold_state.target_roll_cd;
    }

    if(guided_althold_state.use_pitch)
    {
        target_pitch = guided_althold_state.target_pitch_cd;
    }

    // get pilot's desired yaw rate
    float target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());

    if(guided_althold_state.use_yaw_rate)
    {
        target_yaw_rate = guided_althold_state.target_yaw_rate_cds;
    }

    // get pilot desired climb rate
    float target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->get_control_in());
    target_climb_rate = constrain_float(target_climb_rate, -g.pilot_velocity_z_max, g.pilot_velocity_z_max);

#if FRAME_CONFIG == HELI_FRAME
    // helicopters are held on the ground until rotor speed runup has finished
    bool takeoff_triggered = (ap.land_complete && (target_climb_rate > 0.0f) && motors->rotor_runup_complete());
#else
    bool takeoff_triggered = ap.land_complete && (target_climb_rate > 0.0f);
#endif

    // Alt Hold State Machine Determination
    if (!motors->armed() || !motors->get_interlock()) {
        althold_state = AltHold_MotorStopped;
    } else if (takeoff_state.running || takeoff_triggered) {
        althold_state = AltHold_Takeoff;
    } else if (!ap.auto_armed || ap.land_complete) {
        althold_state = AltHold_Landed;
    } else {
        althold_state = AltHold_Flying;
    }

    // Alt Hold State Machine
    switch (althold_state) {

    case AltHold_MotorStopped:

        motors->set_desired_spool_state(AP_Motors::DESIRED_SHUT_DOWN);
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());
        attitude_control->reset_rate_controller_I_terms();
        attitude_control->set_yaw_target_to_current_heading();
#if FRAME_CONFIG == HELI_FRAME    
        // force descent rate and call position controller
        pos_control->set_alt_target_from_climb_rate(-abs(g.land_speed), G_Dt, false);
        heli_flags.init_targets_on_arming=true;
#else
        pos_control->relax_alt_hold_controllers(0.0f);   // forces throttle output to go to zero
#endif
        pos_control->update_z_controller();
        break;

    case AltHold_Takeoff:
#if FRAME_CONFIG == HELI_FRAME    
        if (heli_flags.init_targets_on_arming) {
            heli_flags.init_targets_on_arming=false;
        }
#endif
        // set motors to full range
        motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

        // initiate take-off
        if (!takeoff_state.running) {
            takeoff_timer_start(constrain_float(g.pilot_takeoff_alt,0.0f,1000.0f));
            // indicate we are taking off
            set_land_complete(false);
            // clear i terms
            set_throttle_takeoff();
        }

        // get take-off adjusted pilot and takeoff climb rates
        takeoff_get_climb_rates(target_climb_rate, takeoff_climb_rate);

        // get avoidance adjusted climb rate
        target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

        // call attitude controller
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());

        // call position controller
        pos_control->set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt, false);
        pos_control->add_takeoff_climb_rate(takeoff_climb_rate, G_Dt);
        pos_control->update_z_controller();
        break;

    case AltHold_Landed:
        // set motors to spin-when-armed if throttle below deadzone, otherwise full range (but motors will only spin at min throttle)
        if (target_climb_rate < 0.0f) {
            motors->set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);
        } else {
            motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);
        }

#if FRAME_CONFIG == HELI_FRAME    
        if (heli_flags.init_targets_on_arming) {
            attitude_control->reset_rate_controller_I_terms();
            attitude_control->set_yaw_target_to_current_heading();
            if (motors->get_interlock()) {
                heli_flags.init_targets_on_arming=false;
            }
        }
#else
        attitude_control->reset_rate_controller_I_terms();
        attitude_control->set_yaw_target_to_current_heading();
#endif
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());
        pos_control->relax_alt_hold_controllers(0.0f);   // forces throttle output to go to zero
        pos_control->update_z_controller();
        break;

    case AltHold_Flying:
        motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

#if AC_AVOID_ENABLED == ENABLED
        // apply avoidance
        avoid.adjust_roll_pitch(target_roll, target_pitch, aparm.angle_max);
#endif

        // call attitude controller
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate, GUIDED_ALTHOLD_SMOOTHING_GAIN);

        // adjust climb rate using rangefinder
        if (rangefinder_alt_ok()) {
            // if rangefinder is ok, use surface tracking
            target_climb_rate = get_surface_tracking_climb_rate(target_climb_rate, pos_control->get_alt_target(), G_Dt);
        }

        // get avoidance adjusted climb rate
        target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

        // call position controller
        pos_control->set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt, false);
        pos_control->update_z_controller();
        break;
    }
}

void Copter::guided_althold_set_target_attitude(float roll, float pitch)
{
    // Convert from radians to centi-degrees
    guided_althold_state.target_roll_cd = (18000/M_PI)*roll;
    guided_althold_state.target_pitch_cd = (18000/M_PI)*pitch;
    guided_althold_state.use_roll = true;
    guided_althold_state.use_pitch = true;
    guided_althold_reset_timeout();
}

void Copter::guided_althold_unset_target_attitude()
{
    guided_althold_state.use_roll = false;
    guided_althold_state.use_pitch = false;
    guided_althold_reset_timeout();
}

void Copter::guided_althold_set_target_yaw_rate(float yaw_rate)
{
    // Convert from radians/sec to centi-degrees/sec
    guided_althold_state.target_yaw_rate_cds = (18000/M_PI)*yaw_rate;
    guided_althold_state.use_yaw_rate = true;
    guided_althold_reset_timeout();
}

void Copter::guided_althold_unset_target_yaw_rate()
{
    guided_althold_state.use_yaw_rate = false;
    guided_althold_reset_timeout();
}

void Copter::guided_althold_check_timeout()
{
    // Do not check if already in timeout
    if(guided_althold_state.timeout) return;

    uint64_t tnow = AP_HAL::millis64();

    // Check if last command is too old
    if((tnow - guided_althold_state.update_time_ms) >= GUIDED_ALTHOLD_TIMEOUT_MS)
    {
        // Give control back to pilot
        guided_althold_state.use_roll = false;
        guided_althold_state.use_pitch = false;
        guided_althold_state.use_yaw_rate = false;

        hal.console->printf("GUIDED_ALTHOLD: Ext. computer timeout!\n");
        gcs_send_text(MAV_SEVERITY_EMERGENCY, "Ext. computer timeout!");

        // Notify this as an mode change fail
        AP_Notify::events.user_mode_change_failed = 1;

        guided_althold_state.timeout = true;
    }
}

void Copter::guided_althold_reset_timeout()
{
    guided_althold_state.update_time_ms = AP_HAL::millis64();
    if(guided_althold_state.timeout)
    {
        gcs_send_text(MAV_SEVERITY_INFO, "Ext. computer is back.");
    }
    guided_althold_state.timeout = false;
}
