#include "Sub.h"


bool ModeSurface::init(bool ignore_checks)
{
    nobaro_mode = !sub.control_check_barometer();

    // initialize vertical speeds and acceleration
    // All limits must be positive
    position_control->D_set_max_speed_accel_cm(sub.get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);
    position_control->D_set_correction_speed_accel_cm(sub.get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

    // initialise position and desired velocity
    position_control->D_init_controller();

    // Her mod girisinde yeniden tirmanmaya baslanir; tutus ancak satha varilinca
    // ve SURFMDSW "ayni mod" derse acilir.
    satihta_tut = false;

    return true;

}

void ModeSurface::run()
{
    float target_roll, target_pitch;

    // if not armed set throttle to zero and exit immediately
    if (!motors.armed()) {
        motors.output_min();
        motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        attitude_control->set_throttle_out(NEUTRAL_THROTTLE,true,g.throttle_filt);
        attitude_control->relax_attitude_controllers();
        position_control->D_init_controller();
        return;
    }

    // Give the motors their full range. This line WAS MISSING and the spool state is
    // STICKY (AP_Motors::armed() does not touch it): a vehicle disarmed and re-armed in
    // SURFACE stayed in the GROUND_IDLE written by the branch above and AP_Motors6DOF
    // drove 1500 PWM to every thruster - ARMED, thrust ZERO, while meant to be surfacing.
    motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // If no barometer is available, use the surface_nobaro_thrust parameter to set the throttle output
    if (nobaro_mode) {
        float thrust_output = 0.5f + g2.surface_nobaro_thrust * 0.005f; // map -100, 100 to 0, 1
        attitude_control->set_throttle_out(thrust_output, true, g.throttle_filt);
    } else {
        // Satha varildi: SURFMDSW ne diyorsa onu yap.
        if (sub.ap.at_surface && !satihta_tut) {
            const Mode::Number hedef = (Mode::Number)g.surface_mode_switch.get();
            if (hedef == Mode::Number::SURFACE) {
                // "Ayni mod" secilmis: modda KAL ve araci satihta tut.
                satihta_tut = true;
                gcs().send_text(MAV_SEVERITY_INFO, "Surface: holding at surface");
            } else if (!set_mode(hedef, ModeReason::SURFACE_COMPLETE)) {
                // Istenen mod baslamayi reddetti (ornegin konum cozumu olmadan
                // PosHold). Tirmanmaya DEVAM ETMEK yanlis olur: arac zaten satihta,
                // yarim batmis iticilerle bosuna itip kavitasyon yapar. Tutmaya gec.
                gcs().send_text(MAV_SEVERITY_WARNING, "Surface: mode %d refused, holding",
                                (int)g.surface_mode_switch.get());
                satihta_tut = true;
            }
        }

        // convert pilot input to lean angles
        // To-Do: convert sub.get_pilot_desired_lean_angles to return angles as floats
        sub.get_pilot_desired_lean_angles(channel_roll->get_control_in(), channel_pitch->get_control_in(), target_roll, target_pitch, attitude_control->lean_angle_max_cd());

        // get pilot's desired yaw rate
        float target_yaw_rate = sub.get_pilot_desired_yaw_rate(channel_yaw->get_control_in());

        // call attitude controller
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_cd(target_roll, target_pitch, target_yaw_rate);

        if (satihta_tut) {
            // BALANCED VERTICAL THRUST: instead of commanding a climb rate, the depth
            // target is locked to SURFACE_DEPTH in CLOSED LOOP. The difference matters:
            //   - a continuous climb command drags the target upward forever once the
            //     vehicle is already at the surface; the thrusters end up half out of
            //     the water, cavitate and draw current for nothing;
            //   - with the depth target locked, the position controller produces exactly
            //     the thrust the vehicle's buoyancy calls for. On a negatively buoyant
            //     vehicle that is a steady slight upward thrust (hence "balanced"), and
            //     at neutral buoyancy it is close to zero.
            float hedef_d_m = -(float)g.surface_depth * 0.01f;   // D (down positive), m
            float hiz_d_ms = 0.0f;
            position_control->input_pos_vel_accel_D_m(hedef_d_m, hiz_d_ms, 0.0f);
        } else {
            // set target climb rate
            float cmb_rate_cms = constrain_float(fabsf(sub.wp_nav.get_default_speed_up_cms()), 1, position_control->get_max_speed_up_cms());

            // update altitude target and call position controller
            position_control->D_set_pos_target_from_climb_rate_cms(cmb_rate_cms);
        }
        position_control->D_update_controller();
    }
    // pilot has control for repositioning
    motors.set_forward(channel_forward->norm_input());
    motors.set_lateral(channel_lateral->norm_input());
}
