#include "Sub.h"

/*
 * mode_smart_rtl.cpp - AURA: return home by retracing the track already travelled
 *
 * AP_SmartRTL accumulates 3D breadcrumb points as the vehicle moves (NED metres,
 * relative to the EKF origin) and in the background applies both simplification
 * (Ramer-Douglas-Peucker) and loop pruning. This mode consumes those points BACKWARDS.
 *
 * The difference from a straight-line RTL is exactly what matters for an AUV: under ice,
 * inside a wreck, between pier piles or while following a cable/rope, the straight line
 * home GOES THROUGH THE OBSTACLE. The path already travelled is by definition passable.
 *
 * Copter's SmartRTL climbs 2 m above the last point and LANDS; Rover's stops at the last
 * point. The right shape for an AUV is Rover's: there is NO landing/surfacing here.
 */

// Give up after too many consecutive pop failures (the same budget as in Copter).
#define SRTL_POP_HATA_ZAMAN_MS  10000

bool ModeSmartRtl::init(bool ignore_checks)
{
    if (!sub.g2.smart_rtl.is_active()) {
        // The library is disabled: either SRTL_POINTS=0, or the buffer/position stayed
        // bad for 15 s, or the home point was never recorded because we never armed.
        // Instead of refusing silently, say the reason - the operator has to learn
        // this during the dive.
        gcs().send_text(MAV_SEVERITY_WARNING, "SmartRTL not active");
        return false;
    }

    // Start wp_nav from the current stopping point. No speed argument given -> WP_SPD.
    Vector3p durus_ned_m;
    sub.wp_nav.get_wp_stopping_point_NED_m(durus_ned_m);
    sub.wp_nav.wp_and_spline_init_m(0.0f, durus_ned_m);
    if (!sub.wp_nav.set_wp_destination_NED_m(durus_ned_m)) {
        return false;
    }

    tuketilmis_gecerli = false;
    son_basarili_pop_ms = AP_HAL::millis();
    durum = Durum::YOL_TEMIZLIGI;

    // While transiting the nose turns to the leg bearing; if a camera heading is wanted the
    // operator exits the mode and steers by hand (SmartRTL is an escape manoeuvre, not a photo leg).
    set_auto_yaw_mode(get_default_auto_yaw_mode(true));

    return true;
}

// 3 Hz scheduler task. The AP_SmartRTL header says "call at 3 Hz or faster, NO MATTER
// WHICH MODE THE VEHICLE IS IN" - the breadcrumb path must accumulate in every mode,
// otherwise there is no path when the operator wanders in MANUAL and then hits SmartRTL.
void ModeSmartRtl::save_position()
{
    // While SmartRTL ITSELF is running NO point is added; otherwise the path we are
    // retracing rewrites itself and the vehicle never gets home.
    const bool kaydet = sub.motors.armed() && (sub.control_mode != Mode::Number::SMART_RTL);
    sub.g2.smart_rtl.update(sub.position_ok(), kaydet);
}

void ModeSmartRtl::cikis_temizligi()
{
    // Put the popped but not-yet-reached point back on the path: otherwise every exit
    // from the mode punches a hole in the track.
    if (tuketilmis_gecerli) {
        sub.g2.smart_rtl.add_point(tuketilmis_nokta_ned_m);
        tuketilmis_gecerli = false;
    }
    // Cancel the pending thorough cleanup request; if it is left, the background
    // cleanup stays permanently blocked.
    sub.g2.smart_rtl.cancel_request_for_thorough_cleanup();
}

void ModeSmartRtl::run()
{
    // If the vehicle is disarmed: ArduSub does not stabilize attitude while disarmed.
    if (!sub.motors.armed()) {
        sub.motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        attitude_control->set_throttle_out(NEUTRAL_THROTTLE, true, sub.g.throttle_filt);
        attitude_control->relax_attitude_controllers();
        sub.wp_nav.wp_and_spline_init_m();
        return;
    }

    sub.motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    switch (durum) {
    case Durum::YOL_TEMIZLIGI:
        yol_temizligi_kos();
        break;
    case Durum::IZI_SUR:
        izi_sur_kos();
        break;
    case Durum::DERINLIKTE_TUT:
        tut_kos();
        break;
    }

    wp_cikislarini_sur();
}

// Wait at the current point until the library's thorough cleanup is finished.
// request_thorough_cleanup() is a LATCH: it is called over and over until it returns true.
void ModeSmartRtl::yol_temizligi_kos()
{
    if (sub.g2.smart_rtl.request_thorough_cleanup()) {
        durum = Durum::IZI_SUR;
        gcs().send_text(MAV_SEVERITY_INFO, "SmartRTL: %u points",
                        (unsigned)sub.g2.smart_rtl.get_num_points());
    }
}

void ModeSmartRtl::izi_sur_kos()
{
    if (!sub.wp_nav.reached_wp_destination()) {
        return;
    }

    // The backup of the point we reached is no longer needed.
    tuketilmis_gecerli = false;

    Vector3p nokta_ned_m;
    if (!sub.g2.smart_rtl.pop_point(nokta_ned_m)) {
        // The semaphore may be busy - it is retried on the next loop.
        if (sub.g2.smart_rtl.get_num_points() == 0) {
            // The path is exhausted: home has been reached.
            gcs().send_text(MAV_SEVERITY_INFO, "SmartRTL: path complete");
            durum = Durum::DERINLIKTE_TUT;
        } else if (AP_HAL::millis() - son_basarili_pop_ms > SRTL_POP_HATA_ZAMAN_MS) {
            // There are points but they cannot be taken: stopping and holding here is
            // better than pressing on blindly or shooting up to the surface.
            gcs().send_text(MAV_SEVERITY_ERROR, "SmartRTL: path stalled, holding");
            durum = Durum::DERINLIKTE_TUT;
        }
        return;
    }

    son_basarili_pop_ms = AP_HAL::millis();
    tuketilmis_nokta_ned_m = nokta_ned_m;
    tuketilmis_gecerli = true;

    // CAUTION: Copter ADDS 2 m to the target here (because it will land on top of the
    // last point). There is NO such thing here - we go exactly to the breadcrumb point;
    // the protection against surfacing is the HARD surface ceiling in wp_cikislarini_sur().
    if (!sub.wp_nav.set_wp_destination_NED_m(nokta_ned_m)) {
        sub.failsafe_terrain_on_event();
        return;
    }

    // Report the next point as well so the S-curve does not stop at every breadcrumb.
    Vector3p sonraki_ned_m;
    if (sub.g2.smart_rtl.peek_point(sonraki_ned_m)) {
        sub.wp_nav.set_wp_destination_next_NED_m(sonraki_ned_m);
    }
}

// Station-keep at the last point. The target is not changed; wp_nav is already locked there.
void ModeSmartRtl::tut_kos()
{
}

// Common output path - its body was taken from ModeAuto::auto_wp_run. Copter's
// input_thrust_vector_heading() call has no equivalent in ArduSub: on a vehicle with 6
// degrees of freedom the lateral/forward thrusters are driven separately.
void ModeSmartRtl::wp_cikislarini_sur()
{
    sub.failsafe_terrain_set_status(sub.wp_nav.update_wpnav());

    // HARD surface ceiling (-0.30 m), REGARDLESS of the commanded target.
    //
    // This is the most important AUV-specific decision in this mode. AP_SmartRTL's home
    // point is recorded with set_home() AT THE MOMENT OF ARMING; because ROVs are armed at
    // the surface, the last breadcrumb point sits at ~0 m depth. A SmartRTL without a
    // ceiling would end the mission with exactly what AURA forbids: a vertical ascent to the
    // surface and horizontal transit up there. The ceiling stops the vehicle going shallower
    // than 0.3 m; if surfacing is wanted the operator exits the mode and goes to SURFACE.
    aura_satih_tavani_cekirdek(position_control, -0.30f, true);

    float lateral_out, forward_out;
    sub.translate_wpnav_rp(lateral_out, forward_out);
    sub.motors.set_lateral(lateral_out);
    sub.motors.set_forward(forward_out);

    position_control->D_update_controller();

    float target_roll, target_pitch;
    sub.get_pilot_desired_lean_angles(channel_roll->get_control_in(),
                                      channel_pitch->get_control_in(),
                                      target_roll, target_pitch,
                                      attitude_control->lean_angle_max_cd());

    if (sub.auto_yaw_mode == AUTO_YAW_HOLD) {
        float target_yaw_rate = 0;
        if (!sub.failsafe.pilot_input) {
            target_yaw_rate = sub.get_pilot_desired_yaw_rate(channel_yaw->get_control_in());
        }
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_cd(target_roll, target_pitch, target_yaw_rate);
    } else {
        attitude_control->input_euler_angle_roll_pitch_yaw_cd(target_roll, target_pitch, get_auto_heading(), true);
    }
}
