#include "Torpedo.h"

/*
 * control_circle.pde - init and run calls for circle flight mode
 */

// circle_init - initialise circle controller flight mode
bool ModeCircle::init(bool ignore_checks)
{
    if (!torpedo.position_ok()) {
        return false;
    }

    torpedo.circle_pilot_yaw_override = false;

    // initialize speeds and accelerations
    position_control->set_max_speed_accel_xy(torpedo.wp_nav.get_default_speed_xy(), torpedo.wp_nav.get_wp_acceleration());
    position_control->set_correction_speed_accel_xy(torpedo.wp_nav.get_default_speed_xy(), torpedo.wp_nav.get_wp_acceleration());
    position_control->set_max_speed_accel_z(-torpedo.get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);
    position_control->set_correction_speed_accel_z(-torpedo.get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

    // initialise circle controller including setting the circle center based on vehicle speed
    torpedo.circle_nav.init();

    return true;
}

// circle_run - runs the circle flight mode
// should be called at 100hz or more
void ModeCircle::run()
{
    float target_yaw_rate = 0;
    float target_climb_rate = 0;

    // update parameters, to allow changing at runtime
    position_control->set_max_speed_accel_xy(torpedo.wp_nav.get_default_speed_xy(), torpedo.wp_nav.get_wp_acceleration());
    position_control->set_max_speed_accel_z(-torpedo.get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

    // if not armed set throttle to zero and exit immediately
    if (!motors.armed()) {
        // To-Do: add some initialisation of position controllers
        motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        // Torpedo vehicles do not stabilize roll/pitch/yaw when disarmed
        attitude_control->set_throttle_out(0,true,g.throttle_filt);
        attitude_control->relax_attitude_controllers();
        torpedo.circle_nav.init();
        return;
    }

    // process pilot inputs
    // get pilot's desired yaw rate
    target_yaw_rate = torpedo.get_pilot_desired_yaw_rate(channel_yaw->get_control_in());
    if (!is_zero(target_yaw_rate)) {
        torpedo.circle_pilot_yaw_override = true;
    }

    // get pilot desired climb rate
    target_climb_rate = torpedo.get_pilot_desired_climb_rate(channel_throttle->get_control_in());

    // set motors to full range
    motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // run circle controller
    torpedo.failsafe_terrain_set_status(torpedo.circle_nav.update());

    ///////////////////////
    // update xy outputs //

    float lateral_out, forward_out;
    torpedo.translate_circle_nav_rp(lateral_out, forward_out);

    // Send to forward/lateral outputs
    motors.set_lateral(lateral_out);
    motors.set_forward(forward_out);

    // call attitude controller
    if (torpedo.circle_pilot_yaw_override) {
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(channel_roll->get_control_in(), channel_pitch->get_control_in(), target_yaw_rate);
    } else {
        attitude_control->input_euler_angle_roll_pitch_yaw(channel_roll->get_control_in(), channel_pitch->get_control_in(), torpedo.circle_nav.get_yaw(), true);
    }

    // update altitude target and call position controller
    position_control->set_pos_target_z_from_climb_rate_cm(target_climb_rate);
    position_control->update_z_controller();
}
