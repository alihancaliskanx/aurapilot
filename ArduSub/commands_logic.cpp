#include "Sub.h"

#include <AP_RTC/AP_RTC.h>

static enum AutoSurfaceState auto_surface_state = AUTO_SURFACE_STATE_GO_TO_LOCATION;

// start_command - this function will be called when the ap_mission lib wishes to start a new command
bool Sub::start_command(const AP_Mission::Mission_Command& cmd)
{
    // Altitude-frame validation only makes sense for commands that actually carry a
    // Location: DO/CONDITION commands (e.g. CONDITION_YAW, DO_DIGICAM_CONTROL) store
    // their payload in the same union as `location`, so reading it as a Location
    // yields garbage and the check below would reject them with "Bad alt frame"
    // (seen with CONDITION_YAW).
    // is_nav_cmd() alone is NOT enough: NAV_DELAY, NAV_RETURN_TO_LAUNCH,
    // NAV_SET_YAW_SPEED and AURA_ANCHOR are nav commands without a Location.
    // stored_in_location() is the correct predicate.
    if (AP_Mission::is_nav_cmd(cmd) && AP_Mission::stored_in_location(cmd.id)) {
        const Location &target_loc = cmd.content.location;
        auto alt_frame = target_loc.get_alt_frame();

        if (alt_frame == Location::AltFrame::ABOVE_HOME) {
            if (target_loc.alt > 0) {
                gcs().send_text(MAV_SEVERITY_WARNING, "Alt above home must be negative");
                return false;
            }
        } else if (alt_frame == Location::AltFrame::ABOVE_TERRAIN) {
            if (target_loc.alt < 0) {
                gcs().send_text(MAV_SEVERITY_WARNING, "Alt above terrain must be positive");
                return false;
            }
        } else {
            gcs().send_text(MAV_SEVERITY_WARNING, "Bad alt frame");
            return false;
        }
    }

    switch (cmd.id) {

        ///
        /// navigation commands
        ///
    case MAV_CMD_NAV_WAYPOINT:                  // 16  Navigate to Waypoint
        do_nav_wp(cmd);
        break;

    case MAV_CMD_NAV_LAND:              // 21 LAND to Waypoint
        do_surface(cmd);
        break;

    case MAV_CMD_NAV_RETURN_TO_LAUNCH:
        do_RTL();
        break;

    case MAV_CMD_NAV_LOITER_UNLIM:              // 17 Loiter indefinitely
        do_loiter_unlimited(cmd);
        break;

    case MAV_CMD_NAV_LOITER_TURNS:              //18 Loiter N Times
        do_circle(cmd);
        break;

    case MAV_CMD_NAV_LOITER_TIME:              // 19
        do_loiter_time(cmd);
        break;

#if NAV_GUIDED
    case MAV_CMD_NAV_GUIDED_ENABLE:             // 92  accept navigation commands from external nav computer
        do_nav_guided_enable(cmd);
        break;
#endif

    case MAV_CMD_NAV_DELAY:                    // 93 Delay the next navigation command
        do_nav_delay(cmd);
        break;

    case AP_Mission::MAV_CMD_AURA_ANCHOR:       // 31010 AURA: drop anchor
        do_anchor(cmd);
        break;

        //
        // conditional commands
        //
    case MAV_CMD_CONDITION_DELAY:             // 112
        do_wait_delay(cmd);
        break;

    case MAV_CMD_CONDITION_DISTANCE:             // 114
        do_within_distance(cmd);
        break;

    case MAV_CMD_CONDITION_YAW:             // 115
        do_yaw(cmd);
        break;

        ///
        /// do commands
        ///
    case MAV_CMD_DO_CHANGE_SPEED:             // 178
        do_change_speed(cmd);
        break;

    case MAV_CMD_DO_SET_HOME:             // 179
        do_set_home(cmd);
        break;

    case MAV_CMD_DO_SET_ROI_LOCATION:       // 195
    case MAV_CMD_DO_SET_ROI_NONE:           // 197
    case MAV_CMD_DO_SET_ROI:                // 201
        // point the vehicle and camera at a region of interest (ROI)
        // ROI_NONE can be handled by the regular ROI handler because lat, lon, alt are always zero
        do_roi(cmd);
        break;

    case MAV_CMD_DO_MOUNT_CONTROL:          // 205
        // point the camera to a specified angle
        do_mount_control(cmd);
        break;

#if NAV_GUIDED
    case MAV_CMD_DO_GUIDED_LIMITS:                      // 222  accept guided mode limits
        do_guided_limits(cmd);
        break;
#endif

    default:
        // unable to use the command, allow the vehicle to try the next command
        gcs().send_text(MAV_SEVERITY_WARNING, "Ignoring command %d", cmd.id);
        return false;
    }

    // always return success
    return true;
}

/********************************************************************************/
// Verify command Handlers
/********************************************************************************/

// check to see if current command goal has been achieved
// called by mission library in mission.update()
bool Sub::verify_command_callback(const AP_Mission::Mission_Command& cmd)
{
    if (control_mode == Mode::Number::AUTO) {
        bool cmd_complete = verify_command(cmd);

        // send message to GCS
        if (cmd_complete) {
            gcs().send_mission_item_reached_message(cmd.index);
        }

        return cmd_complete;
    }
    return false;
}


// check if current mission command has completed
bool Sub::verify_command(const AP_Mission::Mission_Command& cmd)
{
    switch (cmd.id) {
        //
        // navigation commands
        //
    case MAV_CMD_NAV_WAYPOINT:
        return verify_nav_wp(cmd);

    case MAV_CMD_NAV_LAND:
        return verify_surface(cmd);

    case MAV_CMD_NAV_RETURN_TO_LAUNCH:
        return verify_RTL();

    case MAV_CMD_NAV_LOITER_UNLIM:
        return verify_loiter_unlimited();

    case MAV_CMD_NAV_LOITER_TURNS:
        return verify_circle(cmd);

    case MAV_CMD_NAV_LOITER_TIME:
        return verify_loiter_time();

#if NAV_GUIDED
    case MAV_CMD_NAV_GUIDED_ENABLE:
        return verify_nav_guided_enable(cmd);
#endif

    case MAV_CMD_NAV_DELAY:
        return verify_nav_delay(cmd);

    case AP_Mission::MAV_CMD_AURA_ANCHOR:
        return verify_anchor(cmd);

        ///
        /// conditional commands
        ///
    case MAV_CMD_CONDITION_DELAY:
        return verify_wait_delay();

    case MAV_CMD_CONDITION_DISTANCE:
        return verify_within_distance();

    case MAV_CMD_CONDITION_YAW:
        return verify_yaw();

        // do commands (always return true)
    case MAV_CMD_DO_CHANGE_SPEED:
    case MAV_CMD_DO_SET_HOME:
    case MAV_CMD_DO_SET_ROI_LOCATION:
    case MAV_CMD_DO_SET_ROI_NONE:
    case MAV_CMD_DO_SET_ROI:
    case MAV_CMD_DO_MOUNT_CONTROL:
    case MAV_CMD_DO_SET_CAM_TRIGG_DIST:
    case MAV_CMD_DO_GUIDED_LIMITS:
        return true;

    default:
        // error message
        gcs().send_text(MAV_SEVERITY_WARNING,"Skipping invalid cmd #%i",cmd.id);
        // return true if we do not recognize the command so that we move on to the next command
        return true;
    }
}

// exit_mission - function that is called once the mission completes
void Sub::exit_mission()
{
    // play a tone
    AP_Notify::events.mission_complete = 1;

    // Try to enter loiter, if that fails, go to depth hold
    if (!mode_auto.auto_loiter_start()) {
        set_mode(Mode::Number::ALT_HOLD, ModeReason::MISSION_END);
    }
}

/********************************************************************************/
//  Nav (Must) commands
/********************************************************************************/

// do_nav_wp - initiate move to next waypoint
void Sub::do_nav_wp(const AP_Mission::Mission_Command& cmd)
{
    Location target_loc(cmd.content.location);
    // use current lat, lon if zero
    if (target_loc.lat == 0 && target_loc.lng == 0) {
        target_loc.lat = current_loc.lat;
        target_loc.lng = current_loc.lng;
    }

    // this will be used to remember the time in millis after we reach or pass the WP.
    loiter_time = 0;
    // this is the delay, stored in seconds
    loiter_time_max = cmd.p1;

    // Set wp navigation target
    mode_auto.auto_wp_start(target_loc);

    // AURA: guard ceiling. verify_nav_wp only ever asked "arrived + held", so a
    // waypoint the vehicle cannot close on parked the mission for good - there is no
    // other mission-item timeout in the tree except the anchor's own guard. On
    // 30 Jul (log_266 item #12) the vehicle sat 91 s at the surface, 1.8 cm outside
    // WPNAV_RADIUS, while a stale XY velocity integrator unwound at PSC_VELXY_I=0.02.
    // Budget three times the straight-line travel time plus the hold, never under a
    // minute; the legs that behave use a small fraction of that (the 14.6 m cruise
    // leg in the same mission took 56 s against a ~140 s budget).
    nav_wp_start_ms = AP_HAL::millis();
    const float leg_cm = (wp_nav.get_wp_destination() - wp_nav.get_wp_origin()).length();
    const float speed_cms = MAX(wp_nav.get_default_speed_xy(), 10.0f);
    nav_wp_guard_ms = MAX((uint32_t)(3000.0f * leg_cm / speed_cms)
                          + loiter_time_max * 1000UL + 30000UL,
                          60000UL);
}

// do_surface - initiate surface procedure
void Sub::do_surface(const AP_Mission::Mission_Command& cmd)
{
    Location target_location;

    // if location provided we fly to that location at current altitude
    if (cmd.content.location.lat != 0 || cmd.content.location.lng != 0) {
        // set state to go to location
        auto_surface_state = AUTO_SURFACE_STATE_GO_TO_LOCATION;

        // calculate and set desired location below surface target
        // convert to location class
        target_location = Location(cmd.content.location);

        // decide if we will use terrain following
        int32_t curr_terr_alt_cm, target_terr_alt_cm;
        if (current_loc.get_alt_cm(Location::AltFrame::ABOVE_TERRAIN, curr_terr_alt_cm) &&
                target_location.get_alt_cm(Location::AltFrame::ABOVE_TERRAIN, target_terr_alt_cm)) {
            // if using terrain, set target altitude to current altitude above terrain
            target_location.set_alt_cm(curr_terr_alt_cm, Location::AltFrame::ABOVE_TERRAIN);
        } else {
            // set target altitude to current altitude above home
            target_location.set_alt_cm(current_loc.alt, Location::AltFrame::ABOVE_HOME);
        }
    } else {
        // set surface state to ascend
        auto_surface_state = AUTO_SURFACE_STATE_ASCEND;

        // Set waypoint destination to current location at zero depth
        target_location = Location(current_loc.lat, current_loc.lng, 0, Location::AltFrame::ABOVE_HOME);
    }

    // Go to wp location
    mode_auto.auto_wp_start(target_location);
}

void Sub::do_RTL()
{
    mode_auto.auto_wp_start(ahrs.get_home());
}

// do_loiter_unlimited - start loitering with no end conditions
// note: caller should set yaw_mode
void Sub::do_loiter_unlimited(const AP_Mission::Mission_Command& cmd)
{
    // convert back to location
    Location target_loc(cmd.content.location);

    // use current location if not provided
    if (target_loc.lat == 0 && target_loc.lng == 0) {
        // To-Do: make this simpler
        Vector3f temp_pos;
        wp_nav.get_wp_stopping_point_NE_cm(temp_pos.xy());
        const Location temp_loc(temp_pos, Location::AltFrame::ABOVE_ORIGIN);
        target_loc.lat = temp_loc.lat;
        target_loc.lng = temp_loc.lng;
    }

    // start way point navigator and provide it the desired location
    mode_auto.auto_wp_start(target_loc);
}

// do_circle - initiate moving in a circle
void Sub::do_circle(const AP_Mission::Mission_Command& cmd)
{
    Location circle_center(cmd.content.location);

    // default lat/lon to current position if not provided
    // To-Do: use stopping point or position_controller's target instead of current location to avoid jerk?
    if (circle_center.lat == 0 && circle_center.lng == 0) {
        circle_center.lat = current_loc.lat;
        circle_center.lng = current_loc.lng;
    }
    // calculate radius
    uint16_t circle_radius_m = HIGHBYTE(cmd.p1); // circle radius held in high byte of p1
    if (cmd.type_specific_bits & (1U << 0)) {
        circle_radius_m *= 10;
    }


    // true if circle should be ccw
    const bool circle_direction_ccw = cmd.content.location.loiter_ccw;

    // move to edge of circle (verify_circle) will ensure we begin circling once we reach the edge
    mode_auto.auto_circle_movetoedge_start(circle_center, circle_radius_m, circle_direction_ccw);
}

// do_loiter_time - initiate loitering at a point for a given time period
// note: caller should set yaw_mode
void Sub::do_loiter_time(const AP_Mission::Mission_Command& cmd)
{
    // re-use loiter unlimited
    do_loiter_unlimited(cmd);

    // setup loiter timer
    loiter_time     = 0;
    loiter_time_max = cmd.p1;     // units are (seconds)
}

#if NAV_GUIDED
// do_nav_guided_enable - initiate accepting commands from external nav computer
void Sub::do_nav_guided_enable(const AP_Mission::Mission_Command& cmd)
{
    if (cmd.p1 > 0) {
        // initialise guided limits
        mode_auto.guided_limit_init_time_and_pos();

        // set navigation target
        mode_auto.auto_nav_guided_start();
    }
}
#endif  // NAV_GUIDED

// do_nav_delay - Delay the next navigation command
void Sub::do_nav_delay(const AP_Mission::Mission_Command& cmd)
{
    nav_delay_time_start_ms = AP_HAL::millis();

    if (cmd.content.nav_delay.seconds > 0) {
        // relative delay
        nav_delay_time_max_ms = cmd.content.nav_delay.seconds * 1000; // convert seconds to milliseconds
    } else {
        // absolute delay to utc time
#if AP_RTC_ENABLED
        nav_delay_time_max_ms = AP::rtc().get_time_utc(cmd.content.nav_delay.hour_utc, cmd.content.nav_delay.min_utc, cmd.content.nav_delay.sec_utc, 0);
#else
        nav_delay_time_max_ms = 0;
#endif
    }
    gcs().send_text(MAV_SEVERITY_INFO, "Delaying %u sec", (unsigned)(nav_delay_time_max_ms/1000));
}

// AURA: do_anchor - drop anchor. The vehicle locks its current point and does
// not advance to the next waypoint until it has settled there.
//   param1 -> duration_s       : anchor duration (s), counted AFTER the sequence
//   param2 -> settle_radius_cm : settle radius (cm), 0 = settle gate disabled
//   param3 -> settle_time_s    : uninterrupted time required inside the radius (s)
//   param4 -> guard_time_s     : overall ceiling (s), 0 = auto
//   x      -> yaw_deg          : camera heading (deg), negative = no turn
//   y      -> take_photo       : 1 = fire the shutter
//   z      -> photo_delay_s    : settle-on-heading wait before the shutter (s)
void Sub::do_anchor(const AP_Mission::Mission_Command& cmd)
{
    anchor_start_ms = AP_HAL::millis();
    anchor_settle_ms = 0;
    anchor_ready_ms = 0;
    anchor_hold_ms = 0;
    anchor_skipped = false;
    anchor_settled = false;
    anchor_photo_done = false;

    if (!mode_auto.auto_anchor_start()) {
        // no position source (DVL/UGPS/EKF) -> cannot anchor.
        // Skip the command so the mission is not blocked.
        gcs().send_text(MAV_SEVERITY_WARNING, "Anchor: no position, skipping");
        anchor_skipped = true;
        return;
    }

    // Camera heading. auto_anchor_start() has just reset the yaw mode to HOLD, so
    // this has to come after it. Turn rate 0 = the default AUTO_YAW_SLEW_RATE; the
    // attitude controller slews at ATC_SLEW_YAW regardless, so a per-command rate
    // would be ignored anyway.
    if (cmd.content.aura_anchor.yaw_valid) {
        mode_auto.set_auto_yaw_look_at_heading(cmd.content.aura_anchor.yaw_deg, 0.0f, 0, 0);
    }

    gcs().send_text(MAV_SEVERITY_INFO, "Anchor set: %u s%s%s",
                    (unsigned)cmd.content.aura_anchor.duration_s,
                    cmd.content.aura_anchor.yaw_valid ? ", turn" : "",
                    cmd.content.aura_anchor.take_photo ? ", photo" : "");
}

#if NAV_GUIDED
// do_guided_limits - pass guided limits to guided controller
void Sub::do_guided_limits(const AP_Mission::Mission_Command& cmd)
{
    mode_guided.guided_limit_set(cmd.p1 * 1000, // convert seconds to ms
                     cmd.content.guided_limits.alt_min * 100.0f,    // convert meters to cm
                     cmd.content.guided_limits.alt_max * 100.0f,    // convert meters to cm
                     cmd.content.guided_limits.horiz_max * 100.0f); // convert meters to cm
}
#endif

/********************************************************************************/
//  Verify Nav (Must) commands
/********************************************************************************/

// verify_nav_wp - check if we have reached the next way point
bool Sub::verify_nav_wp(const AP_Mission::Mission_Command& cmd)
{
    // AURA: guard ceiling first - the mission must never park on a waypoint the
    // vehicle cannot reach (see do_nav_wp for the budget). reached_wp_destination()
    // is a one-way latch on a 3D WPNAV_RADIUS test, so without this there is no
    // escape at all. Report how far short we gave up: that number is the whole
    // diagnosis when it fires.
    if (nav_wp_guard_ms != 0 && AP_HAL::millis() - nav_wp_start_ms >= nav_wp_guard_ms) {
        gcs().send_text(MAV_SEVERITY_WARNING, "WP #%i: guard time expired, %.2fm short",
                        cmd.index,
                        (double)(wp_nav.get_wp_distance_to_destination() * 0.01f));
        return true;
    }

    // check if we have reached the waypoint
    if (!wp_nav.reached_wp_destination()) {
        return false;
    }

    // play a tone
    AP_Notify::events.waypoint_complete = 1;

    // start timer if necessary
    if (loiter_time == 0) {
        loiter_time = AP_HAL::millis();
    }

    // check if timer has run out
    if (((AP_HAL::millis() - loiter_time) / 1000) >= loiter_time_max) {
        gcs().send_text(MAV_SEVERITY_INFO, "Reached command #%i",cmd.index);
        return true;
    }

    return false;
}

// verify_surface - returns true if surface procedure has been completed
bool Sub::verify_surface(const AP_Mission::Mission_Command& cmd)
{
    bool retval = false;

    switch (auto_surface_state) {
        case AUTO_SURFACE_STATE_GO_TO_LOCATION:
            // check if we've reached the location
            if (wp_nav.reached_wp_destination()) {
                // Set target to current xy and zero depth
                // TODO get xy target from current wp destination, because current location may be acceptance-radius away from original destination
                Location target_location(cmd.content.location.lat, cmd.content.location.lng, 0, Location::AltFrame::ABOVE_HOME);

                mode_auto.auto_wp_start(target_location);

                // advance to next state
                auto_surface_state = AUTO_SURFACE_STATE_ASCEND;
            }
            break;

        case AUTO_SURFACE_STATE_ASCEND:
            if (wp_nav.reached_wp_destination()) {
                retval = true;
            }
            break;

        default:
            // this should never happen
            // TO-DO: log an error
            retval = true;
            break;
    }

    // true is returned if we've successfully surfaced
    return retval;
}

bool Sub::verify_RTL() {
    return wp_nav.reached_wp_destination();
}

bool Sub::verify_loiter_unlimited()
{
    return false;
}

// verify_loiter_time - check if we have loitered long enough
bool Sub::verify_loiter_time()
{
    // return immediately if we haven't reached our destination
    if (!wp_nav.reached_wp_destination()) {
        return false;
    }

    // start our loiter timer
    if (loiter_time == 0) {
        loiter_time = AP_HAL::millis();
    }

    // check if loiter timer has run out
    return (((AP_HAL::millis() - loiter_time) / 1000) >= loiter_time_max);
}

// verify_circle - check if we have circled the point enough
bool Sub::verify_circle(const AP_Mission::Mission_Command& cmd)
{
    // check if we've reached the edge
    if (auto_mode == Auto_CircleMoveToEdge) {
        if (wp_nav.reached_wp_destination()) {
            Vector3f circle_center;
            UNUSED_RESULT(cmd.content.location.get_vector_from_origin_NEU_cm(circle_center));

            // set lat/lon position if not provided
            if (cmd.content.location.lat == 0 && cmd.content.location.lng == 0) {
                circle_center.xy() = inertial_nav.get_position_xy_cm();
            }

            // start circling
            mode_auto.auto_circle_start();
        }
        return false;
    }
    const float turns = cmd.get_loiter_turns();

    // check if we have completed circling
    return fabsf(sub.circle_nav.get_angle_total_rad()/M_2PI) >= turns;
}

#if NAV_GUIDED
// verify_nav_guided - check if we have breached any limits
bool Sub::verify_nav_guided_enable(const AP_Mission::Mission_Command& cmd)
{
    // if disabling guided mode then immediately return true so we move to next command
    if (cmd.p1 == 0) {
        return true;
    }

    // check time and position limits
    return mode_auto.guided_limit_check();
}
#endif  // NAV_GUIDED

// verify_nav_delay - check if we have waited long enough
bool Sub::verify_nav_delay(const AP_Mission::Mission_Command& cmd)
{
    if (AP_HAL::millis() - nav_delay_time_start_ms > nav_delay_time_max_ms) {
        nav_delay_time_max_ms = 0;
        return true;
    }
    return false;
}

// AURA: verify_anchor - is the anchor done?
// Order: (1) guard ceiling, (2) settle gate, (3) camera heading, (4) shutter,
//        (5) anchor duration.
// Waiting on the duration alone is not enough without the settle gate: once
// WPNAV_RADIUS has been entered, AC_WPNav latches its reached_destination flag
// and the mission advances even if the vehicle drifts away (AC_WPNav.cpp:540).
// Steps 3 and 4 used to be separate CONDITION_YAW / CONDITION_DELAY /
// DO_DIGICAM_CONTROL items running as do-commands beside this one. That only
// worked when the queue happened to finish inside the hold, because
// advance_current_nav_cmd() drops a pending queue the moment the nav command
// completes - a short hold silently meant no photo. Here the order is guaranteed.
bool Sub::verify_anchor(const AP_Mission::Mission_Command& cmd)
{
    if (anchor_skipped) {
        return true;
    }

    const AP_Mission::Aura_Anchor_Command& anchor = cmd.content.aura_anchor;
    const uint32_t now = AP_HAL::millis();

    // 1) guard ceiling - guarantees the mission advances under any condition
    uint32_t guard_ms = anchor.guard_time_s * 1000UL;
    if (guard_ms == 0) {
        // Auto ceiling. The turn and the pre-shutter wait now happen inside the
        // anchor, so they have to be part of the budget: a plain duration*3 + 30
        // could expire mid-turn and the shutter would never fire.
        guard_ms = (anchor.duration_s + anchor.photo_delay_s) * 3000UL + 30000UL;
        if (anchor.yaw_valid) {
            // a 180 deg turn at the default slew rate takes the better part of 20 s
            guard_ms += 30000UL;
        }
    }
    if (now - anchor_start_ms >= guard_ms) {
        // Last resort: the heading gate below is enforced continuously, so a vehicle
        // that can never hold the aim would otherwise reach this point having taken
        // no photo at all. A frame off-aim still beats an empty slot in the survey,
        // and the distinct text says which one it was.
        if (anchor.take_photo && !anchor_photo_done) {
            anchor_photo_done = true;
#if AP_CAMERA_ENABLED
            camera.take_picture();
#endif
            gcs().send_text(MAV_SEVERITY_WARNING, "Anchor: photo off-aim (guard)");
        }
        gcs().send_text(MAV_SEVERITY_WARNING, "Anchor: guard time expired");
        return true;
    }

    // 2) settle gate: horizontal distance to the locked target must stay inside
    // the radius CONTINUOUSLY (get_wp_distance_to_destination is horizontal only)
    const uint16_t radius_cm = anchor.settle_radius_cm;
    if (radius_cm > 0) {
        if (wp_nav.get_wp_distance_to_destination() > (float)radius_cm) {
            anchor_settle_ms = 0;      // left the radius -> reset the counter
            return false;
        }
        if (anchor_settle_ms == 0) {
            anchor_settle_ms = now;
        }
        if (now - anchor_settle_ms < anchor.settle_time_s * 1000UL) {
            return false;
        }
    }
    if (!anchor_settled) {
        anchor_settled = true;
        gcs().send_text(MAV_SEVERITY_INFO, "Anchor: settled");
    }

    // 3) on station -> turn to the camera heading, then 4) let the swing die out.
    // Same 2 deg tolerance as verify_yaw(); the guard above is the only timeout.
    // The window is re-checked every call while a shutter is still pending, not
    // latched once: a large turn overshoots and crosses the window on the way past,
    // so a single latch starts the pre-shutter wait mid-swing and the photo goes off
    // wherever the nose happens to be. 30 Jul log_266 t=1025.5, a 133 deg turn to
    // 180: first inside the window at t=1029.28, then overshot to 172.95 (7.05 deg
    // past), and the CAM record at t=1032.20 reads Y=174.43 - 5.57 deg off aim. The
    // 64 deg turn at t=789.2 did not overshoot and landed on 90.14. Leaving the
    // window restarts the wait; once the shutter is done (or there is none) the
    // heading stops gating so the plain hold can finish.
    const bool shutter_pending = anchor.take_photo && !anchor_photo_done;
    if (anchor.yaw_valid && (shutter_pending || anchor_ready_ms == 0) &&
        abs(wrap_180_cd(ahrs.yaw_sensor - yaw_look_at_heading)) > 200) {
        anchor_ready_ms = 0;
        return false;
    }
    if (anchor_ready_ms == 0) {
        anchor_ready_ms = now;
    }
    if (anchor.take_photo && now - anchor_ready_ms < anchor.photo_delay_s * 1000UL) {
        return false;
    }

    // 4) shutter, exactly once. This is the same code path the DO_DIGICAM_CONTROL
    // mission item took (AP_Mission::start_command_camera -> camera.take_picture),
    // so CAM1_TYPE = MAVLink is still what makes the command reach the network.
    if (anchor.take_photo && !anchor_photo_done) {
        anchor_photo_done = true;
#if AP_CAMERA_ENABLED
        camera.take_picture();
#endif
        gcs().send_text(MAV_SEVERITY_INFO, "Anchor: photo");
    }

    // 5) everything done -> count the anchor duration
    if (anchor_hold_ms == 0) {
        anchor_hold_ms = now;
    }
    if (now - anchor_hold_ms >= anchor.duration_s * 1000UL) {
        gcs().send_text(MAV_SEVERITY_INFO, "Anchor released");
        return true;
    }
    return false;
}

/********************************************************************************/
//  Condition (May) commands
/********************************************************************************/

void Sub::do_wait_delay(const AP_Mission::Mission_Command& cmd)
{
    condition_start = AP_HAL::millis();
    condition_value = cmd.content.delay.seconds * 1000;     // convert seconds to milliseconds
}

void Sub::do_within_distance(const AP_Mission::Mission_Command& cmd)
{
    condition_value  = cmd.content.distance.meters;
}

void Sub::do_yaw(const AP_Mission::Mission_Command& cmd)
{
    sub.mode_auto.set_auto_yaw_look_at_heading(
        cmd.content.yaw.angle_deg,
        cmd.content.yaw.turn_rate_dps,
        cmd.content.yaw.direction,
        cmd.content.yaw.relative_angle);
}


/********************************************************************************/
// Verify Condition (May) commands
/********************************************************************************/

bool Sub::verify_wait_delay()
{
    if (AP_HAL::millis() - condition_start > (uint32_t)MAX(condition_value, 0)) {
        condition_value = 0;
        return true;
    }
    return false;
}

bool Sub::verify_within_distance()
{
    if (wp_nav.get_wp_distance_to_destination_cm() < (uint32_t)MAX(condition_value,0)) {
        condition_value = 0;
        return true;
    }
    return false;
}

// verify_yaw - return true if we have reached the desired heading
bool Sub::verify_yaw()
{
    // set yaw mode if it has been changed (the waypoint controller often retakes control of yaw as it executes a new waypoint command)
    if (auto_yaw_mode != AUTO_YAW_LOOK_AT_HEADING) {
        sub.mode_auto.set_auto_yaw_mode(AUTO_YAW_LOOK_AT_HEADING);
    }

    // check if we are within 2 degrees of the target heading
    return (abs(wrap_180_cd(ahrs.yaw_sensor-yaw_look_at_heading)) <= 200);
}

/********************************************************************************/
//  Do (Now) commands
/********************************************************************************/

// do_guided - start guided mode
bool Sub::do_guided(const AP_Mission::Mission_Command& cmd)
{
    // only process guided waypoint if we are in guided mode
    if (control_mode != Mode::Number::GUIDED && !(control_mode == Mode::Number::AUTO && auto_mode == Auto_NavGuided)) {
        return false;
    }

    // switch to handle different commands
    switch (cmd.id) {

    case MAV_CMD_NAV_WAYPOINT: {
        // set wp_nav's destination
        return sub.mode_guided.guided_set_destination(cmd.content.location);
    }

    case MAV_CMD_CONDITION_YAW:
        do_yaw(cmd);
        return true;

    default:
        // reject unrecognised command
        return false;
    }

    return true;
}

void Sub::do_change_speed(const AP_Mission::Mission_Command& cmd)
{
    if (cmd.content.speed.target_ms > 0) {
        wp_nav.set_speed_NE_cms(cmd.content.speed.target_ms * 100.0f);
    }
}

void Sub::do_set_home(const AP_Mission::Mission_Command& cmd)
{
    if (cmd.p1 == 1 || !cmd.content.location.initialised()) {
        if (!set_home_to_current_location(false)) {
            // silently ignore this failure
        }
    } else {
        if (!set_home(cmd.content.location, false)) {
            // silently ignore this failure
        }
    }
}

// do_roi - starts actions required by MAV_CMD_NAV_ROI
//          this involves either moving the camera to point at the ROI (region of interest)
//          and possibly rotating the vehicle to point at the ROI if our mount type does not support a yaw feature
//  TO-DO: add support for other features of MAV_CMD_DO_SET_ROI including pointing at a given waypoint
void Sub::do_roi(const AP_Mission::Mission_Command& cmd)
{
    sub.mode_auto.set_auto_yaw_roi(cmd.content.location);
}

// point the camera to a specified angle
void Sub::do_mount_control(const AP_Mission::Mission_Command& cmd)
{
#if HAL_MOUNT_ENABLED
    camera_mount.set_angle_target(cmd.content.mount_control.roll, cmd.content.mount_control.pitch, cmd.content.mount_control.yaw, false);
#endif
}
