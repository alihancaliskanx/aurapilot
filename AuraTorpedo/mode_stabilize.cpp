#include "Torpedo.h"


bool ModeStabilize::init(bool ignore_checks) {
    // set target altitude to zero for reporting
    position_control->set_pos_target_z_cm(0);
    torpedo.last_pilot_heading = ahrs.yaw_sensor;

    return true;
}

void ModeStabilize::run()
{
  uint32_t tnow = AP_HAL::millis();
    float target_roll, target_pitch;

    // if not armed set throttle to zero and exit immediately
    if (!motors.armed()) {
        motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        attitude_control->set_throttle_out(0,true,g.throttle_filt);
        attitude_control->relax_attitude_controllers();
        torpedo.last_pilot_heading = ahrs.yaw_sensor;
        return;
    }

    motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // convert pilot input to lean angles
    // To-Do: convert torpedo.get_pilot_desired_lean_angles to return angles as floats
    // TODO2: move into mode.h
    // AURA torpido: lateral cubugu pitch'e remap edilir (toggle'siz dalis/tirmanis)
    const int16_t trpd_pitch_in = constrain_int16(channel_pitch->get_control_in() + channel_lateral->get_control_in(), -1000, 1000);
    torpedo.get_pilot_desired_lean_angles(channel_roll->get_control_in(), trpd_pitch_in, target_roll, target_pitch, torpedo.aparm.angle_max);

    // get pilot's desired yaw rate
    float yaw_input = channel_yaw->pwm_to_angle_dz_trim(channel_yaw->get_dead_zone() * torpedo.gain, channel_yaw->get_radio_trim());
    float target_yaw_rate = torpedo.get_pilot_desired_yaw_rate(yaw_input);

    // call attitude controller
    // update attitude controller targets

    if (!is_zero(target_yaw_rate)) { // call attitude controller with rate yaw determined by pilot input
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);
        torpedo.last_pilot_heading = ahrs.yaw_sensor;
        torpedo.last_pilot_yaw_input_ms = tnow; // time when pilot last changed heading

    } else { // hold current heading

        // this check is required to prevent bounce back after very fast yaw maneuvers
        // the inertia of the vehicle causes the heading to move slightly past the point when pilot input actually stopped
        if (tnow < torpedo.last_pilot_yaw_input_ms + 250) { // give 250ms to slow down, then set target heading
            target_yaw_rate = 0;  // Stop rotation on yaw axis

            // call attitude controller with target yaw rate = 0 to decelerate on yaw axis
            attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);
            torpedo.last_pilot_heading = ahrs.yaw_sensor; // update heading to hold

        } else { // call attitude controller holding absolute absolute bearing
            attitude_control->input_euler_angle_roll_pitch_yaw(target_roll, target_pitch, torpedo.last_pilot_heading, true);
        }
    }

    // output pilot's throttle
    attitude_control->set_throttle_out(channel_throttle->norm_input(), false, g.throttle_filt);

    //control_in is range -1000-1000
    //radio_in is raw pwm value
    motors.set_forward(channel_forward->norm_input());
    // AURA torpido: lateral cikisi yok (cubuk pitch'e remap edildi)
    motors.set_lateral(0.0f);
}
