#include "Sub.h"

/*
 * mode_anchor.cpp - AURA: ANCHOR flight mode
 *
 * Holds a point that is fed CONTINUOUSLY from outside (GCS / Jetson).
 *
 * The mission anchor item (MAV_CMD_AURA_ANCHOR 31010) IS STILL IN PLACE, not removed.
 * The two use the same station-keeping physics but exist for different things:
 *   - item  : a mission step, parameters come from the plan, completes and moves on
 *   - mode  : fed live, switches to ANCHOR_MDSW on timeout or on data loss
 *
 * Photo/shutter is DELIBERATELY ABSENT: the mission anchor carries it inside itself
 * because there the do-command queue ordering was not guaranteed (CLAUDE.md 10).
 * Here the operator sends the shutter himself whenever he wants it.
 */

// How long after a refused transition to ANCHOR_MDSW before it is retried.
#define ANCHOR_MOD_DENEME_ARALIK_MS  5000

// The refusal warning is not printed more often than this interval. Retrying goes on every
// 5 seconds, but a message per retry would flood the GCS: 53 lines in a 45 second SITL run.
#define ANCHOR_UYARI_ARALIK_MS       30000

// If the anchor point moved less than this, NO NEW target is written to wp_nav (see hedefi_uygula).
#define ANCHOR_HEDEF_ESIK_M          0.05f

bool ModeAnchor::init(bool ignore_checks)
{
    if (!sub.position_ok()) {
        // A point cannot be held without a position solution. requires_GPS() already
        // makes set_mode reject it; this second gate is for paths such as optflow.
        return false;
    }

    // Set up the vertical and horizontal limits.
    position_control->NE_set_max_speed_accel_cm(sub.wp_nav.get_default_speed_NE_cms(),
                                                sub.wp_nav.get_wp_acceleration_cmss());
    position_control->D_set_max_speed_accel_cm(sub.get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

    Vector3p durus_ned_m;
    sub.wp_nav.get_wp_stopping_point_NED_m(durus_ned_m);
    sub.wp_nav.wp_and_spline_init_m(0.0f, durus_ned_m);

    // The stopping point goes through the SAME clamp.
    //
    // For a while the clamp was only inside nokta_kilitle(), so it ran only when the FIRST
    // anchor data message arrived. If the vehicle is put into ANCHOR while floating at the
    // surface and no data is ever sent (using the mode as a manual "stop here" button -
    // explicitly supported below), the target stayed at the ~2 cm depth the vehicle was
    // floating at; and because the surface ceiling opens up to the commanded target, the
    // ceiling opened to the water surface and the vertical controller started chasing
    // wave/barometer noise - the very behaviour we were trying to fix came back.
    hedefi_uygula(durus_ned_m, true);

    giris_ms = AP_HAL::millis();
    son_veri_ms = 0;            // no anchor data has arrived yet
    son_deneme_ms = 0;
    son_uyari_ms = 0;
    yeniden_kilitle = false;

    // Yaw: we FORCE nothing when entering the mode.
    //
    // sub.auto_yaw_mode is a global member and is not reset on a mode change, so a
    // CONDITION_YAW or a DO_SET_ROI issued before entering ANCHOR stays in effect
    // here - which is what we want. If the anchor data carries a yaw it overrides it
    // (demir_verisi_al); if it does not, the externally given yaw lives on.
    // If there is neither, whatever WP_YAW_BEHAVIOR says.

    gcs().send_text(MAV_SEVERITY_INFO, "Anchor: holding, %s",
                    g.anchor_time > 0 ? "timed" : "indefinite");
    return true;
}

// Clamp the target, store it and (if needed) write it to wp_nav.
void ModeAnchor::hedefi_uygula(const Vector3p &istenen_ned_m, bool zorla)
{
    Vector3p yeni_ned_m = istenen_ned_m;

    // The surface decision is made on the COMMANDED depth, not on the vehicle's current
    // measurement: we look at intent, not at wave noise. SURFACE_DEPTH (cm, negative) is
    // the threshold for counting as surfaced; the sign is flipped because D is positive down.
    const float satih_d_m = -(float)g.surface_depth * 0.01f;
    satihta_tut = ((float)yeni_ned_m.z < satih_d_m);

    if (satihta_tut) {
        // SURFACE ANCHOR: the target depth is pulled to SURFACE_DEPTH and FROM HERE ON the
        // vertical axis is driven ONLY by wp_nav.
        //
        // In the first version the target was left as it was and the vertical axis was ALSO
        // driven inside kontrolcuyu_sur() with input_pos_vel_accel_D_m. That was the same
        // DOUBLE INTEGRATION that was fixed in ModeCircle. Measured in SITL: on a surface
        // anchor the vertical thruster variability came out at PWM_std=36.8 - same as a deep
        // anchor (37.0) and as POSHOLD (34.6), i.e. the surface branch was doing nothing at
        // all. Pulling the target to SURFACE_DEPTH settled the vehicle at exactly the depth
        // SURFACE mode settles at (-0.21 m) and, against the mission anchor, vertical effort
        // dropped 224 -> 30 PWM and variability dropped 100 -> 34.
        yeni_ned_m.z = satih_d_m;
    }

    // If the target did not really change, DO NOT TOUCH wp_nav.
    //
    // set_wp_destination_NED_m, when called while the target has not been reached yet,
    // rebuilds AC_WPNav's S-curve leg from scratch. With anchor data flowing at 2 Hz
    // that means the approach trajectory is reset twice per second: the velocity/accel
    // feed-forward starts from zero every time and the vehicle moves in jerks. When the
    // same point keeps arriving (the normal case) doing nothing is the right thing.
    const bool degisti = !hedef_var ||
                         (yeni_ned_m - hedef_ned_m).length() > ANCHOR_HEDEF_ESIK_M;
    if (zorla || degisti) {
        if (!sub.wp_nav.set_wp_destination_NED_m(yeni_ned_m)) {
            sub.failsafe_terrain_on_event();
            return;
        }
    }

    hedef_ned_m = yeni_ned_m;
    hedef_d_m = (float)yeni_ned_m.z;
    hedef_var = true;
}

// Anchor data input. GCS_MAVLink_Sub.cpp decodes msg 86 and hands it to here.
bool ModeAnchor::demir_verisi_al(const Vector3f &konum_neu_cm, bool yaw_var, float yaw_cd)
{
    nokta_kilitle(konum_neu_cm);
    son_veri_ms = AP_HAL::millis();

    if (yaw_var) {
        // The yaw in the anchor data has the HIGHEST priority: if there is an
        // externally issued CONDITION_YAW/ROI, it overrides it.
        sub.mode_auto.set_auto_yaw_look_at_heading(yaw_cd * 0.01f, 0.0f, 0, 0);
    }
    // If yaw_var is false, auto_yaw_mode is NOT TOUCHED: whatever it is at that moment
    // (an externally given angle, a ROI, or the default behaviour) stays. This is the
    // priority order the user wants: anchor data > external yaw command > default.
    return true;
}

void ModeAnchor::nokta_kilitle(const Vector3f &konum_neu_cm)
{
    const Vector3p istenen_ned_m {
        konum_neu_cm.x * 0.01f,
        konum_neu_cm.y * 0.01f,
       -konum_neu_cm.z * 0.01f,     // NEU(up) -> NED(down)
    };
    hedefi_uygula(istenen_ned_m, false);
}

void ModeAnchor::run()
{
    // If the vehicle is disarmed: ArduSub does not stabilize attitude while disarmed.
    if (!sub.motors.armed()) {
        sub.motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        attitude_control->set_throttle_out(NEUTRAL_THROTTLE, true, g.throttle_filt);
        attitude_control->relax_attitude_controllers();

        // KEEP the position controller ALIVE. These lines were missing and it cost dearly:
        //
        // AC_PosControl's NE/D controllers check "was I called last loop as well" with
        // a tick counter (dt_ticks <= 1). Because the scheduler runs regardless of arm
        // state, the counter grows on every loop spent waiting disarmed. On the first
        // armed loop update_wpnav() -> NE_update_controller() sees this and
        // re-initialises itself AND raises INTERNAL_ERROR(flow_of_control); right
        // after it D_update_controller() raises a second one. This flag is PERMANENT
        // and when AP_Arming sees it, it refuses to arm saying "Internal errors 0x..."
        // - and on top of that that check CANNOT BE DISABLED with ARMING_CHECK. So the
        // vehicle WILL NOT ARM AGAIN UNTIL REBOOTED. The mode's advertised usage
        // ("put it in ANCHOR first, then arm") triggered exactly this path.
        //
        // ALL other position/depth holding modes of ArduSub do this in their disarm
        // branch (ModePoshold: NE_init_controller_stopping_point + D_relax_controller).
        position_control->NE_init_controller_stopping_point();
        position_control->D_relax_controller(sub.motors.get_throttle_hover());

        // The counters start together with ARM. Otherwise waiting in ANCHOR while
        // disarmed would run out ANCHOR_TIME and ANCHOR_DTIM and the mode would
        // change instantly on the FIRST loop after arming.
        giris_ms = AP_HAL::millis();
        son_veri_ms = 0;

        // The inits above reset the target to the current position; once armed the
        // anchor point has to be written back.
        yeniden_kilitle = true;
        return;
    }

    if (yeniden_kilitle) {
        yeniden_kilitle = false;
        sub.wp_nav.wp_and_spline_init_m();
        if (hedef_var) {
            hedefi_uygula(hedef_ned_m, true);
        }
    }

    sub.motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // If the mode changed, do nothing else IN THIS LOOP: otherwise ANCHOR's
    // controller overwrites the targets the new mode's init() has just set up.
    if (cikis_kosulu_denetle()) {
        return;
    }

    kontrolcuyu_sur();
}

// Has ANCHOR_TIME expired, has the anchor data been lost?
// Returns: did the mode CHANGE (if true the caller must return immediately).
bool ModeAnchor::cikis_kosulu_denetle()
{
    const Mode::Number hedef_mod = (Mode::Number)g.anchor_mode_switch.get();

    // If ANCHOR_MDSW points at this mode itself there is no exit condition at all:
    // neither the timeout nor data loss matters, the vehicle holds the last commanded
    // point INDEFINITELY. This is exactly the behaviour the user asked for.
    if (hedef_mod == Mode::Number::ANCHOR) {
        return false;
    }

    const uint32_t simdi = AP_HAL::millis();
    const char *sebep = nullptr;

    // 1) Has the anchor data been LOST? ANCHOR_DTIM = 0 -> timeout disabled.
    //
    //    The timeout only starts running AFTER THE FIRST DATA ARRIVES. It was first
    //    written as "if no data ever came, start the counter at mode entry" and it
    //    blew up immediately in SITL: the time between the operator switching mode
    //    and sending the first message ran out the timeout and dropped the mode.
    //    "Lost" is a flow that stops; a flow that never started cannot be lost.
    //
    //    If data never arrives the vehicle keeps holding the entry point - which means
    //    the mode can also be used as a manual "stop here" button.
    const float dtim_s = g.anchor_data_timeout;
    if (is_positive(dtim_s) && son_veri_ms != 0) {
        if (simdi - son_veri_ms > (uint32_t)(dtim_s * 1000.0f)) {
            sebep = "data lost";
        }
    }

    // 2) Has ANCHOR_TIME expired? 0 = indefinite.
    if (sebep == nullptr && g.anchor_time > 0) {
        if (simdi - giris_ms > (uint32_t)g.anchor_time * 1000UL) {
            sebep = "time expired";
        }
    }

    if (sebep == nullptr) {
        return false;
    }

    // If the transition is refused do not give up PERMANENTLY, back off and retry.
    // Saying "I already tried once" with a flag meant that a single refused
    // transition disabled both ANCHOR_TIME and ANCHOR_DTIM for the rest of the dive
    // - whereas the reason for the refusal (e.g. the position solution being lost at
    // that moment) is usually temporary.
    if (son_deneme_ms != 0 && simdi - son_deneme_ms < ANCHOR_MOD_DENEME_ARALIK_MS) {
        return false;
    }
    son_deneme_ms = simdi;

    if (!set_mode(hedef_mod, ModeReason::MISSION_END)) {
        // The requested mode refused to start (for example PosHold with no position
        // solution). We KEEP holding: dropping out of position control is always
        // worse than continuing to hold. The same decision was made for SURFMDSW.
        // We keep retrying but we thin out the warning.
        if (son_uyari_ms == 0 || simdi - son_uyari_ms >= ANCHOR_UYARI_ARALIK_MS) {
            son_uyari_ms = simdi;
            gcs().send_text(MAV_SEVERITY_WARNING, "Anchor: %s, mode %d refused, holding",
                            sebep, (int)g.anchor_mode_switch.get());
        }
        return false;
    }

    gcs().send_text(MAV_SEVERITY_INFO, "Anchor: %s", sebep);
    return true;
}

// Common output path. Its body was taken from ModeAuto::auto_wp_run; Copter's
// input_thrust_vector_heading() call has no equivalent in ArduSub.
void ModeAnchor::kontrolcuyu_sur()
{
    sub.failsafe_terrain_set_status(sub.wp_nav.update_wpnav());

    ///////////////////////
    // vertical axis
    //
    // The vertical axis is driven ONLY by wp_nav; there is NO SECOND call here.
    // On a surface anchor the target itself is pulled to SURFACE_DEPTH inside
    // hedefi_uygula() - the rationale is there.
    //
    // The surface ceiling applies in both cases: it opens up to the commanded target
    // depth, so a deliberately requested shallow anchor is not choked, but an
    // unintended surfacing is prevented.
    aura_satih_tavani_cekirdek(position_control, -hedef_d_m, false);

    ///////////////////////
    // horizontal axis
    float lateral_out, forward_out;
    sub.translate_wpnav_rp(lateral_out, forward_out);
    sub.motors.set_lateral(lateral_out);
    sub.motors.set_forward(forward_out);

    position_control->D_update_controller();

    ///////////////////////
    // attitude
    float target_roll, target_pitch;
    sub.get_pilot_desired_lean_angles(channel_roll->get_control_in(),
                                      channel_pitch->get_control_in(),
                                      target_roll, target_pitch,
                                      attitude_control->lean_angle_max_cd());

    if (sub.auto_yaw_mode == AUTO_YAW_HOLD) {
        // No auto yaw: the heading is driven from the pilot's yaw stick.
        float target_yaw_rate = 0;
        if (!sub.failsafe.pilot_input) {
            target_yaw_rate = sub.get_pilot_desired_yaw_rate(channel_yaw->get_control_in());
        }
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_cd(target_roll, target_pitch, target_yaw_rate);
    } else {
        // The yaw from the anchor data, or an externally issued CONDITION_YAW / ROI.
        attitude_control->input_euler_angle_roll_pitch_yaw_cd(target_roll, target_pitch, get_auto_heading(), true);
    }
}
