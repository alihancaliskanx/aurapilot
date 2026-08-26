#include "Sub.h"

/*
 * control_auto.cpp
 *  Contains the mission, waypoint navigation and NAV_CMD item implementation
 *
 *  While in the auto flight mode, navigation or do/now commands can be run.
 *  Code in this file implements the navigation commands
 */

// AURA shallow-water surface ceiling (GOREV_ALGORITMASI.md §7): WHILE TRANSITING the vertical
// target is held at least 0.3 m below the surface, so the vehicle does not surface
// unintentionally while moving underwater. In shallow water the terrain target ("+1 m off the
// bottom") can fall ABOVE water level -> the vehicle would push up at the surface forever.
//
// EXCEPTION: a surface waypoint the mission COMMANDS EXPLICITLY (for example alt = -0.1 m) is
// allowed if it is shallower than the ceiling; the ceiling only prevents UNINTENTIONAL
// surfacing, it does not choke an order to surface. (The surface WP is -0.1 m: hull ~35 cm and
// the depth sensor is vertically centred -> the sensor stays 10 cm underwater while the camera
// is out of the water.) The exception is only for legs that are NOT terrain-frame: on a terrain
// leg the target z is measured from the bottom and cannot be compared with the surface ceiling.
//
// The get/set_pos_target_z_cm pair of 4.5 does not exist in 4.7: the final target is built
// only inside D_update_controller() (AC_PosControl.cpp:1094,
// _pos_target = desired + offset + terrain) and only desired can be written from outside.
// CAUTION: this function runs BETWEEN update_wpnav() and D_update_controller(), so
// get_pos_target_NED_m() returns a value one loop STALE. Subtracting a "difference" from a
// stale target produces a period-2 oscillation (the ceiling holds for one loop and misses the
// next; in shallow water the average target stays above the surface and the ceiling is
// completely wasted). That is why this loop's target is computed locally and desired is written
// ABSOLUTELY -> idempotent, re-evaluated every loop (it follows rangefinder/terrain changes).
// Units are metres, U = up positive.
//
// The core takes the commanded target from outside: wp_nav's target is meaningful only on
// transit legs, on a circle leg wp_nav holds a stale/irrelevant target. The circle must be
// subject to the same ceiling too (see aura_daire_satih_tavani_uygula).
void aura_satih_tavani_cekirdek(AC_PosControl *position_control,
                                float komut_u_m, bool terrain_mi)
{
    constexpr float SATIH_TAVANI_U_M = -0.30f;   // U (up, m; 0 = surface)
    float tavan_u_m = SATIH_TAVANI_U_M;
    if (!terrain_mi) {
        // if the commanded target is shallower (a surfacing WP) open the ceiling up to it
        tavan_u_m = MAX(tavan_u_m, komut_u_m);
    }
    // hedef_u = desired_u + ofset_u ; ofset_u = -(pos_offset_D + pos_terrain_D)
    const float ofset_u_m = -(float)(position_control->get_pos_offset_NED_m().z
                                     + position_control->get_pos_terrain_D_m());
    const float hedef_u_m = position_control->get_pos_desired_U_m() + ofset_u_m;
    if (hedef_u_m > tavan_u_m) {
        position_control->set_pos_desired_U_m(tavan_u_m - ofset_u_m);

        // Clamping the position target ALONE was not enough: AC_WPNav writes the trajectory
        // as position, VELOCITY and ACCELERATION together (set_pos_vel_accel_NED_m) and
        // D_update_controller ADDS both of them to the target. With the ceiling active the P
        // term held the target at -0.30 m while the feed-forward still commanded UP, so the
        // vehicle overshot ABOVE the ceiling until position error beat the feed-forward.
        // The up component of the clamped trajectory is no longer valid and is zeroed.
        // (Downward feed-forward is not touched: it already moves away from the ceiling.)
        if (is_negative(position_control->get_vel_desired_NED_ms().z)) {
            position_control->set_vel_desired_D_ms(0.0f);
        }
        if (is_negative(position_control->get_accel_desired_D_mss())) {
            position_control->set_accel_desired_D_mss(0.0f);
        }
    }
}

// Transit legs: the commanded target comes from wp_nav.
static void aura_satih_tavani_uygula(AC_PosControl *position_control, const AC_WPNav &wp_nav)
{
    aura_satih_tavani_cekirdek(position_control,
                               wp_nav.get_wp_destination_NEU_cm().z * 0.01f,
                               wp_nav.origin_and_destination_are_terrain_alt());
}

// Circle leg: the commanded target is the circle's centre altitude.
static void aura_daire_satih_tavani_uygula(AC_PosControl *position_control, const AC_Circle &circle_nav)
{
    aura_satih_tavani_cekirdek(position_control,
                               (float)circle_nav.get_center_NEU_cm().z * 0.01f,
                               circle_nav.center_is_terrain_alt());
}
bool ModeAuto::init(bool ignore_checks) {
     if (!sub.position_ok() || !sub.mission.present()) {
        return false;
    }

    sub.auto_mode = Auto_Loiter;

    // stop ROI from carrying over from previous runs of the mission
    // To-Do: reset the yaw as part of auto_wp_start when the previous command was not a wp command to remove the need for this special ROI check
    if (sub.auto_yaw_mode == AUTO_YAW_ROI) {
        set_auto_yaw_mode(AUTO_YAW_HOLD);
    }

    // initialise waypoint controller
    sub.wp_nav.wp_and_spline_init_m();

    // clear guided limits
    guided_limit_clear();

    // TURN OFF the guided overlay on every entry into AUTO. The rationale is the same as
    // guided_limit_clear: a setting that hands over control authority must not survive a
    // mode change made by the operator. If the plan wants the overlay, the
    // MAV_CMD_AURA_GUIDED_SETUP item turns it back on.
    sub.guided_overlay_acik = false;
    sub.guided_overlay_etkin = false;

    // The mission is NOT started HERE - we wait until the vehicle is armed (inside run()).
    //
    // There used to be a direct mission.start_or_resume() here and run() called
    // mission.update() regardless of arm state. The result: the mission started running
    // the moment AUTO was entered while disarmed, and because auto_wp_run's disarm branch
    // reset the target to the current position every loop with wp_and_spline_init_m(),
    // reached_wp_destination() returned true immediately and the WHOLE plan was consumed
    // on the spot. Measured in SITL: a 5 waypoint plan reported "mission complete" in 0.01
    // seconds without the vehicle moving at all. When the operator then armed, there was
    // no work left to do. This is not theoretical, because it is the vehicle's normal flow
    // (AUTO first, then ARM - gorev_yukle.py --auto --arm does exactly this).
    //
    // Copter does the same thing with its waiting_to_start flag (ArduCopter/mode_auto.cpp).
    gorev_arm_bekliyor = true;
    return true;
}

// auto_run - runs the appropriate auto controller
// according to the current auto_mode
void ModeAuto::run()
{
    // The mission only advances while the vehicle is ARMED.
    //
    // Calling mission.update() while disarmed did damage in two separate ways:
    //   1) because auto_wp_run's disarm branch reset the target every loop, waypoints
    //      counted as "reached" instantly and the plan was consumed on the spot (see init()).
    //   2) the AURA guard timers (nav_wp_guard_ms, the anchor guard) ran without looking
    //      at arm state, so timed items advanced by themselves as well.
    // The arm check on the anchor shutter (commands_logic.cpp, "In AUTO the mission runs
    // regardless of arm state") was a patch applied to the SYMPTOM of this problem; this
    // was the cause and it is now closed here.
    if (motors.armed()) {
        if (gorev_arm_bekliyor) {
            // start/resume the mission (based on MIS_RESTART parameter)
            sub.mission.start_or_resume();
            gorev_arm_bekliyor = false;
        }
        // Evaluate the guided overlay BEFORE mission.update(): the overlay blocks
        // verify_nav_wp, so the decision has to have been made before this loop's
        // verify.
        guided_overlay_degerlendir();

        sub.mission.update();
    }

    // call the correct auto controller
    switch (sub.auto_mode) {

    case Auto_WP:
    case Auto_CircleMoveToEdge:
        auto_wp_run();
        break;

    case Auto_Circle:
        auto_circle_run();
        break;

    case Auto_NavGuided:
#if NAV_GUIDED
        auto_nav_guided_run();
#endif
        break;

    case Auto_Loiter:
        auto_loiter_run();
        break;

    case Auto_TerrainRecover:
        auto_terrain_recover_run();
        break;

    case Auto_Anchor:
        auto_anchor_run();
        break;

    case Auto_NavAttitudeTime:
        auto_nav_attitude_time_run();
        break;

    // There is DELIBERATELY no default:. All AutoSubMode values are covered above;
    // adding a default: would silence -Wswitch and make it impossible to catch AT
    // COMPILE TIME that a sub-mode added later is a "state that produces no output".
    // The guard would have hidden the very thing it is supposed to guard.
    }
}

// auto_wp_start - initialises waypoint controller to implement flying to a particular destination
void ModeAuto::auto_wp_start(const Vector3f& destination)
{
    sub.auto_mode = Auto_WP;

    // initialise wpnav (no need to check return status because terrain data is not used)
    sub.wp_nav.set_wp_destination_NEU_cm(destination, false);

    // initialise yaw
    // To-Do: reset the yaw only when the previous navigation command is not a WP.  this would allow removing the special check for ROI
    if (sub.auto_yaw_mode != AUTO_YAW_ROI) {
        set_auto_yaw_mode(get_default_auto_yaw_mode(false));
    }
}

// auto_wp_start - initialises waypoint controller to implement flying to a particular destination
void ModeAuto::auto_wp_start(const Location& dest_loc)
{
    sub.auto_mode = Auto_WP;

    // send target to waypoint controller
    if (!sub.wp_nav.set_wp_destination_loc(dest_loc)) {
        // failure to set destination can only be because of missing terrain data
        gcs().send_text(MAV_SEVERITY_WARNING, "Terrain data (rangefinder) not available");
        sub.failsafe_terrain_on_event();
        return;
    }

    // initialise yaw
    // To-Do: reset the yaw only when the previous navigation command is not a WP.  this would allow removing the special check for ROI
    if (sub.auto_yaw_mode != AUTO_YAW_ROI) {
        set_auto_yaw_mode(get_default_auto_yaw_mode(false));
    }
}

// auto_wp_run - runs the auto waypoint controller
//      called by auto_run at 100hz or more
void ModeAuto::auto_wp_run()
{
    // if not armed set throttle to zero and exit immediately
    if (!motors.armed()) {
        // To-Do: reset waypoint origin to current location because vehicle is probably on the ground so we don't want it lurching left or right on take-off
        //    (of course it would be better if people just used take-off)
        // call attitude controller
        // Sub vehicles do not stabilize roll/pitch/yaw when disarmed
        motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        attitude_control->set_throttle_out(NEUTRAL_THROTTLE,true,g.throttle_filt);
        attitude_control->relax_attitude_controllers();
        sub.wp_nav.wp_and_spline_init_m();                                                // Reset xy target
        return;
    }

    // process pilot's yaw input
    float target_yaw_rate = 0;
    if (!sub.failsafe.pilot_input) {
        // get pilot's desired yaw rate
        target_yaw_rate = sub.get_pilot_desired_yaw_rate(channel_yaw->get_control_in());
        if (!is_zero(target_yaw_rate)) {
            set_auto_yaw_mode(AUTO_YAW_HOLD);
        }
    }

    // set motors to full range
    motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // run waypoint controller
    // TODO implement waypoint radius individually for each waypoint based on cmd.p2
    // TODO fix auto yaw heading to switch to something appropriate when mission complete and switches to loiter
    sub.failsafe_terrain_set_status(sub.wp_nav.update_wpnav());
    aura_satih_tavani_uygula(position_control, sub.wp_nav);   // in transit the target is >= 0.3 m below the surface (except a surface WP)

    ///////////////////////
    // update xy outputs //

    float lateral_out, forward_out;
    sub.translate_wpnav_rp(lateral_out, forward_out);

    // Send to forward/lateral outputs
    motors.set_lateral(lateral_out);
    motors.set_forward(forward_out);

    // WP_Nav has set the vertical position control targets
    // run the vertical position controller and set output throttle
    position_control->D_update_controller();

    ////////////////////////////
    // update attitude output //

    // get pilot desired lean angles
    float target_roll, target_pitch;
    sub.get_pilot_desired_lean_angles(channel_roll->get_control_in(), channel_pitch->get_control_in(), target_roll, target_pitch, attitude_control->lean_angle_max_cd());

    // call attitude controller
    if (sub.auto_yaw_mode == AUTO_YAW_HOLD) {
        // roll & pitch & yaw rate from pilot
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_cd(target_roll, target_pitch, target_yaw_rate);
    } else {
        // roll, pitch from pilot, yaw heading from auto_heading()
        attitude_control->input_euler_angle_roll_pitch_yaw_cd(target_roll, target_pitch, get_auto_heading(), true);
    }
}

// auto_circle_movetoedge_start - initialise waypoint controller to move to edge of a circle with it's center at the specified location
//  we assume the caller has set the circle's circle with sub.circle_nav.set_center()
//  we assume the caller has performed all required GPS_ok checks
// radius_m : 0 = use the CIRCLE_RADIUS_M parameter
// rate_degs: SIGNED angular rate (+ clockwise, - counter-clockwise); 0 = use the
//            CIRCLE_RATE parameter together with its sign
void ModeAuto::auto_circle_movetoedge_start(const Location &circle_center, float radius_m, float rate_degs)
{
    // set circle center
    sub.circle_nav.set_center(circle_center);

    // The radius is ALWAYS written. A zero radius used to mean "do not touch", and because
    // AC_Circle::update_ms uses the raw _radius_m (get_radius_m()'s fallback to the
    // parameter does NOT apply THERE) whatever value was left behind got flown: 0 on the
    // first boot -> spinning in place, 0 after a previous circle -> that circle's radius.
    // Neither is what the mission item said. In AURA 0 = "use the CIRCLE_RADIUS_M
    // parameter". get_radius_m() IS WRONG FOR THIS JOB: it only falls back while no
    // run-time radius has ever been written, and once one has it returns that stale value
    // - i.e. exactly the bug we are trying to fix. The parameter is read directly.
    sub.circle_nav.set_radius_m(is_zero(radius_m) ? sub.circle_nav.get_radius_parm_m() : radius_m);

    // The angular rate can be given per item as well. Previously only a "ccw" flag arrived
    // here and the magnitude was always read from CIRCLE_RATE; that is, every circle in the
    // mission had to turn at the same rate. Now the signed rate arrives directly, and if it
    // is 0 we fall back to the parameter (including its sign).
    sub.circle_nav.set_rate_degs(is_zero(rate_degs) ? sub.circle_nav.get_rate_degs() : rate_degs);

    // check our distance from edge of circle
    Vector3f circle_edge_neu_cm;
    float dist_to_edge;
    sub.circle_nav.get_closest_point_on_circle_NEU_cm(circle_edge_neu_cm, dist_to_edge);

    // if more than 3m then fly to edge
    if (dist_to_edge > 300.0f) {
        // The state is written BEFORE set_wp_destination_loc and it must stay that way.
        //
        // At one point it was moved to the end (on the grounds that "an intervening run()
        // would fly to the previous target") but that reasoning was WRONG: mission.update()
        // is already called from inside ModeAuto::run(), BEFORE the controller switch, so
        // run() cannot come between do_* and this. Worse, the move produced a real bug: if
        // the set_wp_destination_loc below fails, failsafe_terrain_on_event() runs, which
        // sets auto_mode to Auto_TerrainRecover and stops the mission - and the assignment
        // at the end would OVERWRITE that, silently cancelling the terrain failsafe.
        sub.auto_mode = Auto_CircleMoveToEdge;

        // convert circle_edge_neu_cm to Location
        Location circle_edge(circle_edge_neu_cm, Location::AltFrame::ABOVE_ORIGIN);

        // convert altitude to same as command
        circle_edge.copy_alt_from(circle_center);

        // initialise wpnav to move to edge of circle
        if (!sub.wp_nav.set_wp_destination_loc(circle_edge)) {
            // failure to set destination can only be because of missing terrain data
            sub.failsafe_terrain_on_event();
        }

        // if we are outside the circle, point at the edge, otherwise hold yaw
        // If a ROI is active do not touch it: DO_SET_ROI locks the camera onto a point and
        // the leg going to the circle used to overwrite it silently. auto_wp_start already
        // has the same guard, and so does Copter (mode_auto.cpp circle_movetoedge_start).
        if (sub.auto_yaw_mode != AUTO_YAW_ROI) {
            float dist_to_center = get_horizontal_distance(inertial_nav.get_position_xy_cm().topostype(), sub.circle_nav.get_center_NEU_cm().xy());
            if (dist_to_center > sub.circle_nav.get_radius_cm() && dist_to_center > 500) {
                set_auto_yaw_mode(get_default_auto_yaw_mode(false));
            } else {
                // vehicle is within circle so hold yaw to avoid spinning as we move to edge of circle
                set_auto_yaw_mode(AUTO_YAW_HOLD);
            }
        }

    } else {
        auto_circle_start();
    }
}

// auto_circle_start - initialises controller to fly a circle in AUTO flight mode
//   assumes that circle_nav object has already been initialised with circle center and radius
void ModeAuto::auto_circle_start()
{
    // Yaw mode 1 = "keep the heading you had when ENTERING the circle". That is the moment
    // the circle starts, not the moment the item starts: if there is a move-to-edge leg in
    // between, the vehicle turns there with LOOK_AT_NEXT_WP, so the angle is captured here.
    if (sub.daire_yaw_kip == 1) {
        sub.daire_yaw_cd = sub.ahrs.yaw_sensor;
    }

    // Passing get_rate_degs() to init_NEU_cm THREW THE TURN DIRECTION AWAY.
    // get_rate_degs() reads the CIRCLE_RATE PARAMETER, not the sign that
    // auto_circle_movetoedge_start wrote with set_rate_degs() (AC_Circle.h: those are two
    // separate members, _rate_parm_degs vs _rotation_rate_max_rads). The result: the
    // mission's param3<0 (CCW) request was silently dropped every time and the vehicle
    // turned according to the sign of CIRCLE_RATE (default +2 deg/s -> clockwise).
    // get_rate_max_degs() gives back the signed value that was just written.
    // NOTE: Copter has the same bug (ArduCopter/mode_auto.cpp circle_start).
    sub.circle_nav.init_NEU_cm(sub.circle_nav.get_center_NEU_cm(),
                               sub.circle_nav.center_is_terrain_alt(),
                               sub.circle_nav.get_rate_max_degs());

    // The state goes last: init_NEU_cm resets the position controller.
    sub.auto_mode = Auto_Circle;
}

// auto_circle_run - circle in AUTO flight mode
//      called by auto_run at 100hz or more
// The circle's vertical axis: a shaped climb rate towards the centre altitude (m/s, U).
//
// On the NON-terrain branch AC_Circle::update_ms does not build the vertical target itself -
// it makes the target "-get_pos_desired_U_m()", the current target itself (AC_Circle.cpp:241),
// and drives the vertical axis only with the given climb_rate. With no climb rate (the old
// state) that means "hold the depth you are at" and the mission item's altitude is NEVER
// applied. This is glaring when the vehicle starts closer than 3 m to the circle edge: the
// move-to-edge leg is skipped (that leg carried the depth via copy_alt_from) and the circle
// never commands a depth either - even if the item says -12 m it turns at its current depth.
float ModeAuto::aura_daire_dikey_hiz_ms() const
{
    if (sub.circle_nav.center_is_terrain_alt()) {
        // on the terrain branch the library already drives the target with input_pos_vel_accel_D_m
        return 0.0f;
    }
    const float hedef_u_m = (float)sub.circle_nav.get_center_NEU_cm().z * 0.01f;
    const float hata_u_m = hedef_u_m - position_control->get_pos_desired_U_m();
    const float hiz_u_ms = sqrt_controller(hata_u_m,
                                           position_control->D_get_pos_p().kP(),
                                           position_control->D_get_max_accel_mss(),
                                           position_control->get_dt_s());
    return constrain_float(hiz_u_ms,
                           -position_control->get_max_speed_down_ms(),
                           position_control->get_max_speed_up_ms());
}

void ModeAuto::auto_circle_run()
{
    // If the vehicle is disarmed: this function used to have NO arm guard at all; the motor
    // outputs and the vertical controller were driven while disarmed too. The guard in
    // auto_wp_run was taken as the model - BUT we do NOT RESET the target the way it does.
    // auto_wp_run's disarm branch calls wp_and_spline_init_m(), which moves the target to the
    // current position so reached_wp_destination() becomes true instantly; that is why AUTO
    // entered while disarmed consumes the mission on the spot. To avoid that trap for the
    // circle, circle_nav state is untouched: angles do not accumulate, verify_circle does not complete.
    if (!motors.armed()) {
        motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        // Sub vehicles do not stabilize roll/pitch/yaw when disarmed
        attitude_control->set_throttle_out(NEUTRAL_THROTTLE, true, g.throttle_filt);
        attitude_control->relax_attitude_controllers();
        return;
    }

    // CAUTION: check_param_change() is NOT CALLED here.
    //
    // ModeCircle calls it and there that is right - in that mode the radius is the parameter
    // itself. In AUTO the radius is set by the MISSION ITEM and check_param_change()
    // overwrites it: _last_radius_param_m is only set inside AC_Circle::init(), and because
    // AUTO uses the init_NEU_cm() path that field stays at 0. So the first call always
    // thought "the parameter changed" and wrote CIRCLE_RADIUS_M into _radius_m. Measured in
    // SITL: the item asked for 2.50 m while the vehicle turned at 10.00 m radius (the
    // parameter value). The parameter must not overwrite the item.
    // Set the motors to full range. This line was missing: if the circle is the mission's
    // FIRST nav command (and the move-to-edge leg was skipped too) the spool state is never
    // pulled to THROTTLE_UNLIMITED, the vehicle stayed in GROUND_IDLE and never turned.
    motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // call circle controller
    sub.failsafe_terrain_set_status(sub.circle_nav.update_ms(aura_daire_dikey_hiz_ms()));

    // The surface ceiling applies on a circle leg too (in transit the target is >= 0.3 m
    // below the surface). It used to be applied only on transit legs; there was no
    // protection at all against the circle surfacing unintentionally.
    aura_daire_satih_tavani_uygula(position_control, sub.circle_nav);

    float lateral_out, forward_out;
    sub.translate_circle_nav_rp(lateral_out, forward_out);

    // Send to forward/lateral outputs
    motors.set_lateral(lateral_out);
    motors.set_forward(forward_out);

    // circle_nav has set the vertical position control targets
    // run the vertical position controller and set output throttle
    position_control->D_update_controller();

    // roll & pitch from waypoint controller, yaw from the item's yaw mode
    attitude_control->input_euler_angle_roll_pitch_yaw_cd(channel_roll->get_control_in(),
                                                          channel_pitch->get_control_in(),
                                                          aura_daire_yaw_cd(), true);
}

// Where the nose points during a circle. MAV_CMD_AURA_CIRCLE selects it per item;
// NAV_LOITER_TURNS always sets mode 0, so the old behaviour is preserved exactly.
float ModeAuto::aura_daire_yaw_cd()
{
    // If the mission EXPLICITLY states a heading (CONDITION_YAW or DO_SET_ROI) that wins.
    //
    // auto_yaw_mode used to be ignored entirely during a circle, and that did not just mean
    // "ROI does not work": verify_yaw() returns false until the heading is within 2 degrees
    // of the target. Because the circle never applied that target the condition was never
    // met, _flags.do_cmd_loaded got stuck at true and ALL SUBSEQUENT do-commands in that nav
    // block (including the shutter) never ran at all.
    if (sub.auto_yaw_mode == AUTO_YAW_ROI || sub.auto_yaw_mode == AUTO_YAW_LOOK_AT_HEADING) {
        return get_auto_heading();
    }

    // When CIRCLE_OPTIONS bit 1 is set, AC_Circle::update_ms adds the +/-90 degrees to
    // get_yaw_cd() ITSELF. A caller computing its own tangent that does not know this applies
    // the angle twice: mode 3 points exactly the wrong way (180 error), mode 0 looks 90 to the side.
    const bool kutuphane_teget = sub.circle_nav.face_direction_of_travel();
    // The sign is exactly the same as in AC_Circle: positive rate -> -90, negative rate -> +90.
    const float taraf_cd = is_negative(sub.circle_nav.get_rate_max_degs()) ? 9000.0f : -9000.0f;

    switch (sub.daire_yaw_kip) {
    case 1:     // hold the heading fixed (the heading when entering the circle)
    case 2:     // fixed heading angle
        return sub.daire_yaw_cd;

    case 3:     // tangent: look along the direction of travel
        return kutuphane_teget ? sub.circle_nav.get_yaw_cd()
                               : wrap_360_cd(sub.circle_nav.get_yaw_cd() + taraf_cd);

    case 0:     // look at the centre (default)
    default:
        return kutuphane_teget ? wrap_360_cd(sub.circle_nav.get_yaw_cd() - taraf_cd)
                               : sub.circle_nav.get_yaw_cd();
    }
}

#if NAV_GUIDED
// auto_nav_guided_start - hand over control to external navigation controller in AUTO mode
void ModeAuto::auto_nav_guided_start()
{
    sub.mode_guided.init(true);
    sub.auto_mode = Auto_NavGuided;
    // initialise guided start time and position as reference for limit checking
    sub.mode_auto.guided_limit_init_time_and_pos();
}

// auto_nav_guided_run - allows control by external navigation controller
//      called by auto_run at 100hz or more
void ModeAuto::auto_nav_guided_run()
{
    // call regular guided flight mode run function
    sub.mode_guided.run();
}
#endif  // NAV_GUIDED

// auto_loiter_start - initialises loitering in auto mode
//  returns success/failure because this can be called by exit_mission
bool ModeAuto::auto_loiter_start()
{
    // return failure if GPS is bad
    if (!sub.position_ok()) {
        return false;
    }
    sub.auto_mode = Auto_Loiter;

    // calculate stopping point
    Vector3f stopping_point_neu_cm;
    sub.wp_nav.get_wp_stopping_point_NEU_cm(stopping_point_neu_cm);

    // initialise waypoint controller target to stopping point
    sub.wp_nav.set_wp_destination_NEU_cm(stopping_point_neu_cm);

    // hold yaw at current heading
    set_auto_yaw_mode(AUTO_YAW_HOLD);

    return true;
}

// auto_loiter_run - loiter in AUTO flight mode
//      called by auto_run at 100hz or more
void ModeAuto::auto_loiter_run(bool honour_auto_yaw)
{
    // if not armed set throttle to zero and exit immediately
    if (!motors.armed()) {
        motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        // Sub vehicles do not stabilize roll/pitch/yaw when disarmed
        attitude_control->set_throttle_out(NEUTRAL_THROTTLE,true,g.throttle_filt);
        attitude_control->relax_attitude_controllers();

        sub.wp_nav.wp_and_spline_init_m();                                                // Reset xy target
        return;
    }

    // accept pilot input of yaw
    float target_yaw_rate = 0;
    if (!sub.failsafe.pilot_input) {
        target_yaw_rate = sub.get_pilot_desired_yaw_rate(channel_yaw->get_control_in());
    }

    // set motors to full range
    motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // run waypoint and z-axis position controller
    sub.failsafe_terrain_set_status(sub.wp_nav.update_wpnav());
    aura_satih_tavani_uygula(position_control, sub.wp_nav);   // in transit the target is >= 0.3 m below the surface (except a surface WP)

    ///////////////////////
    // update xy outputs //
    float lateral_out, forward_out;
    sub.translate_wpnav_rp(lateral_out, forward_out);

    // Send to forward/lateral outputs
    motors.set_lateral(lateral_out);
    motors.set_forward(forward_out);

    // WP_Nav has set the vertical position control targets
    // run the vertical position controller and set output throttle
    position_control->D_update_controller();

    // get pilot desired lean angles
    float target_roll, target_pitch;
    sub.get_pilot_desired_lean_angles(channel_roll->get_control_in(), channel_pitch->get_control_in(), target_roll, target_pitch, attitude_control->lean_angle_max_cd());

    // AURA: stock loiter ignores auto_yaw_mode entirely, so a CONDITION_YAW running as
    // the mission's do-command has no effect and verify_yaw() never reaches its target.
    // Callers that need the mission to steer the heading pass honour_auto_yaw=true;
    // this mirrors auto_wp_run() above.
    // NOTE (4.7): get_auto_heading() returns centidegrees -> the _cd variant is mandatory,
    // the suffix-less name does not exist in 4.7 and the _rad variant expects radians.
    if (honour_auto_yaw && sub.auto_yaw_mode != AUTO_YAW_HOLD) {
        // roll, pitch from pilot, yaw heading from auto_heading()
        attitude_control->input_euler_angle_roll_pitch_yaw_cd(target_roll, target_pitch, get_auto_heading(), true);
    } else {
        // roll & pitch & yaw rate from pilot
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_cd(target_roll, target_pitch, target_yaw_rate);
    }
}

// AURA: auto_anchor_start - drop anchor
//  Makes the current stopping point the wp_nav destination; the vehicle holds that
//  point closed-loop (AC_PosControl XY PID, the same controller as POSHOLD).
//  Fails when there is no position source.
bool ModeAuto::auto_anchor_start()
{
    // how far the locked point may sit from where the vehicle would coast to a stop
    constexpr float ANCHOR_MAX_PULL_CM = 500.0f;

    if (!sub.position_ok()) {
        return false;
    }

    sub.auto_mode = Auto_Anchor;

    // Lock the point the mission asked for, not the point the vehicle drifted to.
    // get_wp_stopping_point_NEU_cm() projects from the *measured* position, so whatever
    // tracking error existed the instant the anchor dropped became permanent - and
    // the settle gate cannot catch it, because it measures distance to the locked
    // point rather than to the target. 30 Jul log_266 t=789.2: the surface waypoint
    // was at N=-3.87, the anchor froze N=-4.21 and duly reported "settled" 34 cm off.
    // Re-using the previous leg's destination closes that error instead of accepting
    // it, and it is the right point at both call sites (the surface waypoint for a
    // photo anchor, the dive-in-place waypoint for the departure anchor).
    Vector3f stopping_point;
    sub.wp_nav.get_wp_stopping_point_NEU_cm(stopping_point);

    const Vector3f target = sub.wp_nav.get_wp_destination_NEU_cm();
    // A terrain-frame destination is not re-usable: the anchor makes no rangefinder
    // guarantee. And an anchor with no waypoint before it (first mission item, or
    // stale wp_nav state) would otherwise send the vehicle off to whatever is left in
    // _destination - the anchor is a stop, never a leg, so anything out of reach
    // falls back to the stopping point.
    // reached_wp_destination() is the load-bearing one here. Re-using the previous
    // target only closes a small tracking error while that target was actually
    // reached: set_wp_destination_NEU_cm() then leaves origin == destination and nothing
    // moves. If the waypoint gave up on its guard instead, _flags.reached_destination
    // is false, set_wp_destination_NEU_cm() runs wp_and_spline_init_m() and
    // the anchor turns into a real shaped leg of up to ANCHOR_MAX_PULL_CM - flown at
    // the surface, because aura_satih_tavani_uygula opens the ceiling to the anchor's
    // own -0.1 m target. Travelling at the surface is the one thing this pattern never
    // does, and a waypoint the vehicle could not reach is exactly where it must stop.
    const bool target_usable = sub.wp_nav.reached_wp_destination() &&
                               !sub.wp_nav.origin_and_destination_are_terrain_alt() &&
                               (target - stopping_point).length() < ANCHOR_MAX_PULL_CM;
    sub.wp_nav.set_wp_destination_NEU_cm(target_usable ? target : stopping_point);

    // Default to holding the current heading: without this the previous leg's
    // AUTO_YAW_LOOK_AT_NEXT_WP would survive and the vehicle would slew towards the
    // next waypoint while anchored. A CONDITION_YAW in the mission still wins, because
    // verify_yaw() re-asserts AUTO_YAW_LOOK_AT_HEADING on the next mission update.
    set_auto_yaw_mode(AUTO_YAW_HOLD);

    return true;
}

// AURA: auto_anchor_run - controller while anchored
//  Called by auto_run at 100 Hz or above.
//  Like loiter - wp_nav holds the locked destination, surface ceiling applies - but the
//  heading follows auto_yaw_mode so a CONDITION_YAW can aim the camera before the photo.
void ModeAuto::auto_anchor_run()
{
    auto_loiter_run(true);
}


// set_auto_yaw_look_at_heading - sets the yaw look at heading for auto mode
void ModeAuto::set_auto_yaw_look_at_heading(float angle_deg, float turn_rate_dps, int8_t direction, uint8_t relative_angle)
{
    // get current yaw
    int32_t curr_yaw_target = attitude_control->get_att_target_euler_cd().z;

    // get final angle, 1 = Relative, 0 = Absolute
    if (relative_angle == 0) {
        // absolute angle
        sub.yaw_look_at_heading = wrap_360_cd(angle_deg * 100);
    } else {
        // relative angle
        if (direction < 0) {
            angle_deg = -angle_deg;
        }
        sub.yaw_look_at_heading = wrap_360_cd((angle_deg * 100 + curr_yaw_target));
    }

    // get turn speed
    if (is_zero(turn_rate_dps)) {
        // default to regular auto slew rate
        sub.yaw_look_at_heading_slew = AUTO_YAW_SLEW_RATE;
    } else {
        sub.yaw_look_at_heading_slew = MIN(turn_rate_dps, AUTO_YAW_SLEW_RATE);    // deg / sec
    }

    // set yaw mode
    set_auto_yaw_mode(AUTO_YAW_LOOK_AT_HEADING);

    // TO-DO: restore support for clockwise and counter clockwise rotation held in cmd.content.yaw.direction.  1 = clockwise, -1 = counterclockwise
}


// sets the desired yaw rate
void ModeAuto::set_yaw_rate(float turn_rate_dps)
{    
    // set sub to desired yaw rate
    sub.yaw_look_at_heading_slew = MIN(turn_rate_dps, AUTO_YAW_SLEW_RATE);    // deg / sec

    // set yaw mode
    set_auto_yaw_mode(AUTO_YAW_RATE);
}

// set_auto_yaw_roi - sets the yaw to look at roi for auto mode
void ModeAuto::set_auto_yaw_roi(const Location &roi_location)
{
    // if location is zero lat, lon and altitude turn off ROI
    if (!roi_location.initialised()) {
        // set auto yaw mode back to default assuming the active command is a waypoint command.  A more sophisticated method is required to ensure we return to the proper yaw control for the active command
        set_auto_yaw_mode(get_default_auto_yaw_mode(false));
#if HAL_MOUNT_ENABLED
        // switch off the camera tracking if enabled
        sub.camera_mount.clear_roi_target();
#endif  // HAL_MOUNT_ENABLED
    } else {
#if HAL_MOUNT_ENABLED
        // check if mount type requires us to rotate the sub
        if (!sub.camera_mount.has_pan_control()) {
            if (roi_location.get_vector_from_origin_NEU_cm(sub.roi_WP)) {
                set_auto_yaw_mode(AUTO_YAW_ROI);
            }
        }
        // send the command to the camera mount
        sub.camera_mount.set_roi_target(roi_location);

        // TO-DO: expand handling of the do_nav_roi to support all modes of the MAVLink.  Currently we only handle mode 4 (see below)
        //      0: do nothing
        //      1: point at next waypoint
        //      2: point at a waypoint taken from WP# parameter (2nd parameter?)
        //      3: point at a location given by alt, lon, lat parameters
        //      4: point at a target given a target id (can't be implemented)
#else
        // if we have no camera mount aim the sub at the location
        if (roi_location.get_vector_from_origin_NEU_cm(sub.roi_WP)) {
            set_auto_yaw_mode(AUTO_YAW_ROI);
        }
#endif  // HAL_MOUNT_ENABLED
    }
}

// Return true if it is possible to recover from a rangefinder failure
bool ModeAuto::auto_terrain_recover_start()
{
#if AP_RANGEFINDER_ENABLED
    // Check rangefinder status to see if recovery is possible
    switch (sub.rangefinder.status_orient(ROTATION_PITCH_270)) {

    case RangeFinder::Status::OutOfRangeLow:
    case RangeFinder::Status::OutOfRangeHigh:

        // RangeFinder::Good if just one valid sample was obtained recently, but ::rangefinder_state.alt_healthy
        // requires several consecutive valid readings for wpnav to accept rangefinder data
    case RangeFinder::Status::Good:
        sub.auto_mode = Auto_TerrainRecover;
        break;

        // Not connected or no data
    default:
        return false; // Rangefinder is not connected, or has stopped responding
    }

    // Initialize recovery timeout time
    sub.fs_terrain_recover_start_ms = AP_HAL::millis();

    // Stop mission
    sub.mission.stop();

    // Reset xy target
    sub.loiter_nav.clear_pilot_desired_acceleration();
    sub.loiter_nav.init_target();

    // Reset z axis controller
    position_control->D_relax_controller(motors.get_throttle_hover());

    // initialize vertical maximum speeds and acceleration
    // All limits must be positive
    position_control->D_set_max_speed_accel_cm(sub.wp_nav.get_default_speed_down_cms(), sub.wp_nav.get_default_speed_up_cms(), sub.wp_nav.get_accel_D_cmss());
    position_control->D_set_correction_speed_accel_cm(sub.wp_nav.get_default_speed_down_cms(), sub.wp_nav.get_default_speed_up_cms(), sub.wp_nav.get_accel_D_cmss());

    gcs().send_text(MAV_SEVERITY_WARNING, "Attempting auto failsafe recovery");
    return true;
#else
    return false;
#endif
}

// Attempt recovery from terrain failsafe
// If recovery is successful resume mission
// If recovery fails revert to failsafe action
// AURA: the guided overlay (MAV_CMD_AURA_GUIDED_SETUP) state machine.
//
// While it is ON and a LIVE guided setpoint exists, the vehicle goes to that
// setpoint instead of the AUTO leg; when the data goes quiet it resumes its leg
// WHERE IT LEFT OFF.
//
// It is only valid on NAV_WAYPOINT legs. The anchor, circle, surfacing and position
// fix items are each exactly what the operator asked for; letting a setpoint
// overwrite them would make the plan unreadable. A waypoint, on the other hand, only
// says "go over there" - which is exactly what a companion computer may want to refine.
void ModeAuto::guided_overlay_degerlendir()
{
    const bool uygun = sub.guided_overlay_acik
                       && sub.mission.get_current_nav_id() == MAV_CMD_NAV_WAYPOINT
                       && sub.guided_verisi_taze(sub.guided_overlay_zaman_ms);

    if (uygun && !sub.guided_overlay_etkin) {
        // ENTER: first save the leg, then hand over to guided.
        //
        // We take the target from wp_nav, not from the mission item: do_nav_wp puts the
        // current position in for an item with lat/lon==0, so the item itself does not
        // always give the point actually flown. The resolved target in wp_nav is correct.
        sub.guided_overlay_wp_neu_cm = sub.wp_nav.get_wp_destination_NEU_cm();
        sub.guided_overlay_giris_ms = AP_HAL::millis();
        sub.guided_overlay_etkin = true;
        auto_nav_guided_start();
        gcs().send_text(MAV_SEVERITY_INFO, "GuidedSetup: overlay engaged");
        return;
    }

    if (!uygun && sub.guided_overlay_etkin) {
        guided_overlay_birak();
    }
}

// Leave the overlay and hand back the interrupted NAV_WAYPOINT leg.
void ModeAuto::guided_overlay_birak()
{
    if (!sub.guided_overlay_etkin) {
        return;
    }
    sub.guided_overlay_etkin = false;

    // Rebuild the leg. auto_wp_start restarts wp_nav from WHERE THE VEHICLE IS towards
    // the same target - wherever guided may have taken it in the meantime.
    // We DO NOT TOUCH the AP_Mission state: set_current_cmd(the same index) would drop
    // the do-command queue running in parallel with that leg.
    auto_wp_start(sub.guided_overlay_wp_neu_cm);

    // SHIFT the guard clock FORWARD by the overlay duration. The clock kept running
    // through the overlay; without the shift a long overlay would make the leg be
    // skipped immediately on return with "guard time expired".
    const uint32_t gecen = AP_HAL::millis() - sub.guided_overlay_giris_ms;
    sub.nav_wp_start_ms += gecen;

    // The hold counter was running through the overlay too; the leg starts again.
    sub.loiter_time = 0;

    gcs().send_text(MAV_SEVERITY_INFO, "GuidedSetup: leg resumed");
}

// NAV_ATTITUDE_TIME (42703): hold the given attitude for N seconds.
//
// On an AUV the equivalent of this is "point the camera that way and wait there": to take
// photo/video while looking at a wall, a pile or a boat hull. Position is NOT HELD - only
// attitude and vertical rate are commanded; if there is a current the vehicle drifts. If
// the point has to be held, MAV_CMD_AURA_ANCHOR is used.
void ModeAuto::auto_nav_attitude_time_start(const AP_Mission::Mission_Command& cmd)
{
    nav_attitude_time.roll_deg      = cmd.content.nav_attitude_time.roll_deg;
    nav_attitude_time.pitch_deg     = cmd.content.nav_attitude_time.pitch_deg;
    nav_attitude_time.yaw_deg       = cmd.content.nav_attitude_time.yaw_deg;
    nav_attitude_time.climb_rate_ms = cmd.content.nav_attitude_time.climb_rate;
    nav_attitude_time.start_ms      = AP_HAL::millis();
    sub.auto_mode = Auto_NavAttitudeTime;
}

void ModeAuto::auto_nav_attitude_time_run()
{
    // if not armed set throttle to zero and exit immediately
    if (!motors.armed()) {
        motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        attitude_control->set_throttle_out(NEUTRAL_THROTTLE, true, g.throttle_filt);
        attitude_control->relax_attitude_controllers();
        position_control->D_relax_controller(motors.get_throttle_hover());
        return;
    }

    motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // There is NO position control: the horizontal thrusters are left idle. In Copter this
    // mode makes horizontal acceleration out of the lean angle so nothing extra is needed;
    // on a 6 degrees of freedom AUV the lateral/forward thrusters are driven separately, and
    // if they are not zeroed the value left from the previous leg would keep driving them.
    motors.set_lateral(0.0f);
    motors.set_forward(0.0f);

    // Copter's lean_angle_max limit is NOT APPLIED HERE: that limit comes from the fact
    // that on a multirotor the lean angle is also the thrust vector. On an AUV roll/pitch
    // is a real attitude command, and clamping it breaks what the item asked for.
    attitude_control->input_euler_angle_roll_pitch_yaw_cd(nav_attitude_time.roll_deg * 100.0f,
                                                          nav_attitude_time.pitch_deg * 100.0f,
                                                          nav_attitude_time.yaw_deg * 100.0f,
                                                          true);

    const float climb_rate_ms = constrain_float(nav_attitude_time.climb_rate_ms,
                                                -position_control->get_max_speed_down_ms(),
                                                 position_control->get_max_speed_up_ms());
    position_control->D_set_pos_target_from_climb_rate_ms(climb_rate_ms);
    // Surface ceiling: this command drives the vertical rate DIRECTLY and there is no
    // commanded target depth, so a fixed -0.3 m ceiling as in terrain recovery.
    // A plan that deliberately wants to surface uses NAV_LAND.
    aura_satih_tavani_cekirdek(position_control, -0.30f, true);
    position_control->D_update_controller();
}

void ModeAuto::auto_terrain_recover_run()
{
    float target_climb_rate = 0;

    // if not armed set throttle to zero and exit immediately
    if (!motors.armed()) {
        motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        attitude_control->set_throttle_out(NEUTRAL_THROTTLE,true,g.throttle_filt);
        attitude_control->relax_attitude_controllers();

        sub.loiter_nav.init_target();                                       // Reset xy target
        position_control->D_relax_controller(motors.get_throttle_hover());  // Reset z axis controller
        return;
    }

    // SET the motors to full range. This line was missing and the spool state is STICKY:
    // AP_Motors::armed() does not touch it. On a bottom-following mission, if the vehicle is
    // disarmed and armed again after the terrain failsafe triggered (the branch above will
    // have written GROUND_IDLE), _spool_desired stays at GROUND_IDLE; in that state
    // AP_Motors6DOF puts 1500 PWM on all thrusters. The result: the vehicle is ARMED, in
    // AUTO, the position controller thinks it is in command, but thrust is ZERO - free drift.
    motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

#if AP_RANGEFINDER_ENABLED
    static uint32_t rangefinder_recovery_ms = 0;
    switch (sub.rangefinder.status_orient(ROTATION_PITCH_270)) {

    case RangeFinder::Status::OutOfRangeLow:
        target_climb_rate = sub.wp_nav.get_default_speed_up_cms();
        rangefinder_recovery_ms = 0;
        break;

    case RangeFinder::Status::OutOfRangeHigh:
        target_climb_rate = sub.wp_nav.get_default_speed_down_cms();
        rangefinder_recovery_ms = 0;
        break;

    case RangeFinder::Status::Good: // exit on success (recovered rangefinder data)

        target_climb_rate = 0; // Attempt to hold current depth

        if (sub.rangefinder_state.alt_healthy) {

            // Start timer as soon as rangefinder is healthy
            if (rangefinder_recovery_ms == 0) {
                rangefinder_recovery_ms = AP_HAL::millis();
                position_control->D_relax_controller(motors.get_throttle_hover()); // Reset alt hold targets
            }

            // 1.5 seconds of healthy rangefinder means we can resume mission with terrain enabled
            if (AP_HAL::millis() > rangefinder_recovery_ms + 1500) {
                gcs().send_text(MAV_SEVERITY_INFO, "Terrain failsafe recovery successful!");
                sub.failsafe_terrain_set_status(true); // Reset failsafe timers
                sub.failsafe.terrain = false; // Clear flag
                sub.auto_mode = Auto_Loiter; // Switch back to loiter for next iteration
                sub.mission.resume(); // Resume mission
                rangefinder_recovery_ms = 0; // Reset for subsequent recoveries
            }

        }
        break;

        // Not connected, or no data
    default:
        // Terrain failsafe recovery has failed, terrain data is not available
        // and rangefinder is not connected, or has stopped responding
        gcs().send_text(MAV_SEVERITY_CRITICAL, "Terrain failsafe recovery failure: No Rangefinder!");
        sub.failsafe_terrain_act();
        rangefinder_recovery_ms = 0;
        return;
    }
#else
    gcs().send_text(MAV_SEVERITY_CRITICAL, "Terrain failsafe recovery failure: No Rangefinder!");
    sub.failsafe_terrain_act();
    // failsafe_terrain_act() CHANGES MODE (POSHOLD/ALT_HOLD/SURFACE) or disarms. Without
    // the return this function carried on and overwrote the position/attitude targets the
    // new mode's init() had just set up. The "No Rangefinder" branch above already gets
    // its return right; these two were the ones that were missing it.
    return;
#endif

    // exit on failure (timeout)
    if (AP_HAL::millis() > sub.fs_terrain_recover_start_ms + FS_TERRAIN_RECOVER_TIMEOUT_MS) {
        // Recovery has failed, revert to failsafe action
        gcs().send_text(MAV_SEVERITY_CRITICAL, "Terrain failsafe recovery timeout!");
        sub.failsafe_terrain_act();
        return;     // the rationale above
    }

    // run loiter controller
    sub.loiter_nav.update();

    ///////////////////////
    // update xy targets //
    float lateral_out, forward_out;
    sub.translate_wpnav_rp(lateral_out, forward_out);

    // Send to forward/lateral outputs
    motors.set_lateral(lateral_out);
    motors.set_forward(forward_out);

    /////////////////////
    // update z target //
    position_control->D_set_pos_target_from_climb_rate_cms(target_climb_rate);
    // The surface ceiling is needed most HERE: the OutOfRangeLow branch climbs UP at
    // WPNAV_SPEED_UP, and in shallow water a vehicle near the bottom reports exactly
    // OutOfRangeLow. Without a ceiling the recovery climbs for the whole
    // FS_TERRAIN_RECOVER_TIMEOUT_MS and surfaces the vehicle - in the very configuration
    // (terrain-frame / SURFTRAK) the ceiling exists for. No commanded target, so fixed -0.3 m.
    aura_satih_tavani_cekirdek(position_control, -0.30f, true);
    position_control->D_update_controller();

    ////////////////////////////
    // update angular targets //
    float target_roll = 0;
    float target_pitch = 0;

    // convert pilot input to lean angles
    // To-Do: convert sub.get_pilot_desired_lean_angles to return angles as floats
    sub.get_pilot_desired_lean_angles(channel_roll->get_control_in(), channel_pitch->get_control_in(), target_roll, target_pitch, attitude_control->lean_angle_max_cd());

    float target_yaw_rate = 0;

    // call attitude controller
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_cd(target_roll, target_pitch, target_yaw_rate);
}
