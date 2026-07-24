#include "Torpedo.h"

// AURA torpido derinlik->pitch dongusu (Faz 3.2): hesap ortak yardimcida
// (Mode::trpd_derinlik_pitch_cd, mode.cpp) — AUTO ile paylasilir.
static constexpr float TRPD_SATIH_TAVANI_CM = -30.0f; // aura satih tavani: hedef >= 0.3 m derinlik


bool ModeAlthold::init(bool ignore_checks) {
    if(!torpedo.control_check_barometer()) {
        return false;
    }

    // initialize vertical maximum speeds and acceleration
    // sets the maximum speed up and down returned by position controller
    position_control->set_max_speed_accel_z(-torpedo.get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);
    position_control->set_correction_speed_accel_z(-torpedo.get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

    // initialise position and desired velocity
    position_control->init_z_controller();

    torpedo.last_pilot_heading = ahrs.yaw_sensor;

    return true;
}

// althold_run - runs the althold controller
// should be called at 100hz or more
void ModeAlthold::run()
{
    // AURA: once derinlik dongusu pitch hedefini hesaplar, run_pre onu kullanir
    control_depth();
    run_pre();
    run_post();
}

void ModeAlthold::run_pre()
{
    uint32_t tnow = AP_HAL::millis();

    // initialize vertical speeds and acceleration
    position_control->set_max_speed_accel_z(-torpedo.get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

    if (!motors.armed()) {
        motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        // Torpedo vehicles do not stabilize roll/pitch/yaw when not auto-armed (i.e. on the ground, pilot has never raised throttle)
        attitude_control->set_throttle_out(0.5,true,g.throttle_filt);
        attitude_control->relax_attitude_controllers();
        position_control->relax_z_controller(motors.get_throttle_hover());
        torpedo.last_pilot_heading = ahrs.yaw_sensor;
        return;
    }

    motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // get pilot desired lean angles
    float target_roll, target_pitch;

    // Check if set_attitude_target_no_gps is valid
    if (tnow - torpedo.set_attitude_target_no_gps.last_message_ms < 5000) {
        float target_yaw;
        Quaternion(
            torpedo.set_attitude_target_no_gps.packet.q
        ).to_euler(
            target_roll,
            target_pitch,
            target_yaw
        );
        target_roll = degrees(target_roll);
        target_pitch = degrees(target_pitch);
        target_yaw = degrees(target_yaw);

        attitude_control->input_euler_angle_roll_pitch_yaw(target_roll * 1e2f, target_pitch * 1e2f, target_yaw * 1e2f, true);
        return;
    }

    torpedo.get_pilot_desired_lean_angles(channel_roll->get_control_in(), channel_pitch->get_control_in(), target_roll, target_pitch, attitude_control->get_althold_lean_angle_max_cd());

    // AURA torpido: derinlik dongusunun pitch talebi pilot pitch'inin ustune biner
    // (pilot pitch genelde 0; toggle ile mudahale trim gibi davranir)
    target_pitch = constrain_float(target_pitch + _trpd_pitch_cd, -4000.0f, 4000.0f);

    // get pilot's desired yaw rate
    float yaw_input = channel_yaw->pwm_to_angle_dz_trim(channel_yaw->get_dead_zone() * torpedo.gain, channel_yaw->get_radio_trim());
    float target_yaw_rate = torpedo.get_pilot_desired_yaw_rate(yaw_input);

    // call attitude controller
    if (!is_zero(target_yaw_rate)) { // call attitude controller with rate yaw determined by pilot input
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);
        torpedo.last_pilot_heading = ahrs.yaw_sensor;
        torpedo.last_pilot_yaw_input_ms = tnow; // time when pilot last changed heading

    } else { // hold current heading

        // this check is required to prevent bounce back after very fast yaw maneuvers
        // the inertia of the vehicle causes the heading to move slightly past the point when pilot input actually stopped
        if (tnow < torpedo.last_pilot_yaw_input_ms + 250) { // give 250ms to slow down, then set target heading
            target_yaw_rate = 0; // Stop rotation on yaw axis

            // call attitude controller with target yaw rate = 0 to decelerate on yaw axis
            attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);
            torpedo.last_pilot_heading = ahrs.yaw_sensor; // update heading to hold

        } else { // call attitude controller holding absolute bearing
            attitude_control->input_euler_angle_roll_pitch_yaw(target_roll, target_pitch, torpedo.last_pilot_heading, true);
        }
    }
}

void ModeAlthold::run_post()
{
    motors.set_forward(channel_forward->norm_input());
    // AURA torpido: lateral yok (holonomik degil); cikis 0
    motors.set_lateral(0.0f);
}

void ModeAlthold::control_depth() {
    float target_climb_rate_cm_s = torpedo.get_pilot_desired_climb_rate(channel_throttle->get_control_in());
    target_climb_rate_cm_s = constrain_float(target_climb_rate_cm_s, -torpedo.get_pilot_speed_dn(), g.pilot_speed_up);

    // desired_climb_rate returns 0 when within the deadzone.
    //we allow full control to the pilot, but as soon as there's no input, we handle being at surface/bottom
    if (fabsf(target_climb_rate_cm_s) < 0.05f)  {
        if (torpedo.ap.at_surface) {
            position_control->set_pos_target_z_cm(MIN(position_control->get_pos_target_z_cm(), g.surface_depth - 5.0f)); // set target to 5 cm below surface level
        } else if (torpedo.ap.at_bottom) {
            position_control->set_pos_target_z_cm(MAX(inertial_nav.get_position_z_up_cm() + 10.0f, position_control->get_pos_target_z_cm())); // set target to 10 cm above bottom
        }
    }

    position_control->set_pos_target_z_from_climb_rate_cm(target_climb_rate_cm_s);

    // === AURA torpido (Faz 3.2): derinlik hatasi -> pitch hedefi ===
    // update_z_controller CAGRILMAZ: throttle cikisi torpidoda hicbir motora
    // baglanmiyor (frame'de throttle faktorleri 0) ve integrator birikmesin.
    // Aura satih tavani: hedef derinlik satihtan en az 0.3 m altta kalir.
    position_control->set_pos_target_z_cm(MIN(position_control->get_pos_target_z_cm(), TRPD_SATIH_TAVANI_CM));

    _trpd_pitch_cd = trpd_derinlik_pitch_cd();
}
