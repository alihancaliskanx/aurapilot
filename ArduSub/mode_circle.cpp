#include "Sub.h"

/*
 * control_circle.pde - init and run calls for circle flight mode
 */

// circle_init - initialise circle controller flight mode
bool ModeCircle::init(bool ignore_checks)
{
    if (!sub.position_ok()) {
        return false;
    }

    sub.circle_pilot_yaw_override = false;

    // initialize speeds and accelerations
    // All limits must be positive
    position_control->NE_set_max_speed_accel_cm(sub.wp_nav.get_default_speed_NE_cms(), sub.wp_nav.get_wp_acceleration_cmss());
    position_control->NE_set_correction_speed_accel_cm(sub.wp_nav.get_default_speed_NE_cms(), sub.wp_nav.get_wp_acceleration_cmss());
    position_control->D_set_max_speed_accel_cm(sub.get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);
    position_control->D_set_correction_speed_accel_cm(sub.get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

    // initialise circle controller including setting the circle center based on vehicle speed
    sub.circle_nav.init();

    return true;
}

// circle_run - runs the circle flight mode
// should be called at 100hz or more
void ModeCircle::run()
{
    float target_yaw_rate = 0;
    float target_climb_rate = 0;

    // update parameters, to allow changing at runtime
    // All limits must be positive
    position_control->NE_set_max_speed_accel_cm(sub.wp_nav.get_default_speed_NE_cms(), sub.wp_nav.get_wp_acceleration_cmss());
    position_control->D_set_max_speed_accel_cm(sub.get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

    // check for any change in params and update in real time
    sub.circle_nav.check_param_change();

    // if not armed set throttle to zero and exit immediately
    if (!motors.armed()) {
        // To-Do: add some initialisation of position controllers
        motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        // Sub vehicles do not stabilize roll/pitch/yaw when disarmed
        attitude_control->set_throttle_out(NEUTRAL_THROTTLE,true,g.throttle_filt);
        attitude_control->relax_attitude_controllers();
        sub.circle_nav.init();
        return;
    }

    // process pilot inputs
    // get pilot's desired yaw rate
    target_yaw_rate = sub.get_pilot_desired_yaw_rate(channel_yaw->get_control_in());
    if (!is_zero(target_yaw_rate)) {
        sub.circle_pilot_yaw_override = true;
    }

    // get pilot desired climb rate
    target_climb_rate = sub.get_pilot_desired_climb_rate(channel_throttle->get_control_in());

    // set motors to full range
    motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // Run circle controller. The pilot's vertical demand is fed in HERE.
    //
    // It used to call update_cms() with no argument while the vertical axis was driven a
    // SECOND TIME below with D_set_pos_target_from_climb_rate_cms(). Both paths ADVANCE the
    // vertical target by one dt through update_pos_vel_accel() inside
    // AC_PosControl::input_vel_accel_D_m (AC_PosControl.cpp), so the target was integrated
    // twice every loop: the depth target drifted at ~2x the requested rate and the jerk shaper
    // was pulled first to 0 and then to the pilot's demand in the same tick. One call is left.
    sub.failsafe_terrain_set_status(sub.circle_nav.update_cms(target_climb_rate));

    ///////////////////////
    // update xy outputs //

    float lateral_out, forward_out;
    sub.translate_circle_nav_rp(lateral_out, forward_out);

    // Send to forward/lateral outputs
    motors.set_lateral(lateral_out);
    motors.set_forward(forward_out);

    // call attitude controller
    if (sub.circle_pilot_yaw_override) {
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_cd(channel_roll->get_control_in(), channel_pitch->get_control_in(), target_yaw_rate);
    } else {
        attitude_control->input_euler_angle_roll_pitch_yaw_cd(channel_roll->get_control_in(), channel_pitch->get_control_in(), sub.circle_nav.get_yaw_cd(), true);
    }

    // The vertical target was set up above inside update_cms(target_climb_rate); here only
    // the controller is run (Copter's ModeCircle does exactly the same thing).
    position_control->D_update_controller();
}
