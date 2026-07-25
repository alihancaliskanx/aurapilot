#include "Torpedo.h"


bool ModeSurface::init(bool ignore_checks)
{
    if(!torpedo.control_check_barometer()) {
        return false;
    }

    // initialize vertical speeds and acceleration
    position_control->set_max_speed_accel_z(-torpedo.get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);
    position_control->set_correction_speed_accel_z(-torpedo.get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

    // initialise position and desired velocity
    position_control->init_z_controller();

    return true;

}

void ModeSurface::run()
{
    // AURA torpido SURFACE: dikey itici yok — satha PITCH (burun yukari) + OTO-GAZ
    // ile cikilir (fin otoritesi yol ister). Failsafe'lerin cagirdigi mod oldugundan
    // pilot girdisiz de kendi kendine calisir. Satihta ALT_HOLD'a devreder (Sub gibi).
    // ALT_HOLD'daki windup kiskaci + saplanma korumasinin aynilari uygulanir.
    static constexpr float TRPD_SATIH_TAVANI_CM = -30.0f;  // mode_althold.cpp ile ayni deger

    float target_roll, target_pitch;

    // if not armed set throttle to zero and exit immediately
    if (!motors.armed()) {
        motors.output_min();
        motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        attitude_control->set_throttle_out(0,true,g.throttle_filt);
        attitude_control->relax_attitude_controllers();
        position_control->init_z_controller();
        return;
    }

    // Already at surface, hold depth at surface
    if (torpedo.ap.at_surface) {
        set_mode(Mode::Number::ALT_HOLD, ModeReason::SURFACE_COMPLETE);
        return;   // mod degisti (ALT_HOLD devraldi); run()'in kalani gereksiz
    }

    // set target climb rate (log tutarliligi icin Sub'daki gibi kaydedilir)
    float cmb_rate = constrain_float(fabsf(torpedo.wp_nav.get_default_speed_up()), 1, position_control->get_max_speed_up_cms());
    torpedo.desired_climb_rate = cmb_rate;

    // Hedef derinligi yukari sur + windup kiskaci (arac ±3 m bandi) + satih tavani.
    // update_z_controller CAGRILMAZ (throttle yolu olu; derinlik pitch ile).
    position_control->set_pos_target_z_from_climb_rate_cm(cmb_rate);
    const float z_cm = inertial_nav.get_position_z_up_cm();
    position_control->set_pos_target_z_cm(constrain_float(position_control->get_pos_target_z_cm(), z_cm - 300.0f, z_cm + 300.0f));
    position_control->set_pos_target_z_cm(MIN(position_control->get_pos_target_z_cm(), TRPD_SATIH_TAVANI_CM));

    // convert pilot input to lean angles
    torpedo.get_pilot_desired_lean_angles(channel_roll->get_control_in(), channel_pitch->get_control_in(), target_roll, target_pitch, torpedo.aparm.angle_max);

    // derinlik->pitch dongusu pilot pitch'inin ustune biner
    target_pitch = constrain_float(target_pitch + trpd_derinlik_pitch_cd(), -4000.0f, 4000.0f);

    // get pilot's desired yaw rate
    float target_yaw_rate = torpedo.get_pilot_desired_yaw_rate(channel_yaw->get_control_in());

    // call attitude controller
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);

    // Oto-gaz: 1 m'den derindeyken ileri komut en az 0.6 (pilot vermese de);
    // saplanma korumasi: cikilacakken burun asagi sapliysa gaz kesilir —
    // pozitif sephiye + dogrultma momenti burnu kurtarir, gaz geri gelir.
    float trpd_ileri = channel_forward->norm_input();
    bool trpd_gaz = (z_cm < -100.0f);
    if (trpd_gaz && ahrs.pitch_sensor < -1000) {
        trpd_gaz = false;
    }
    if (trpd_gaz) {
        trpd_ileri = MAX(trpd_ileri, 0.6f);
    }
    motors.set_forward(trpd_ileri);
    // AURA torpido: lateral yok
    motors.set_lateral(0.0f);
}
