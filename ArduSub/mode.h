#pragma once

#include "Sub.h"
class Parameters;
class ParametersG2;

class GCS_Sub;

// Guided modes
enum GuidedSubMode {
    Guided_WP,
    Guided_Velocity,
    Guided_PosVel,
    Guided_Angle,
};

// AURA shallow-water surface ceiling. The definition is in mode_auto.cpp; it was taken out
// of file-static scope so that SmartRTL can use it too. komut_u_m = the vertical target this
// leg commands (U, up positive, m). If terrain_mi is true the commanded target is IGNORED and
// a fixed -0.30 m ceiling is applied; that is the right behaviour for legs that "do not
// command a target depth" (terrain recovery, NAV_ATTITUDE_TIME, SmartRTL).
void aura_satih_tavani_cekirdek(AC_PosControl *position_control, float komut_u_m, bool terrain_mi);

// Auto modes
enum AutoSubMode {
    Auto_WP,
    Auto_CircleMoveToEdge,
    Auto_Circle,
    Auto_NavGuided,
    Auto_Loiter,
    Auto_TerrainRecover,
    Auto_Anchor,         // AURA: drop anchor - lock the current point
    Auto_NavAttitudeTime // NAV_ATTITUDE_TIME - hold an attitude for N seconds
};

// RTL states
enum RTLState {
    RTL_InitialClimb,
    RTL_ReturnHome,
    RTL_LoiterAtHome,
    RTL_FinalDescent,
    RTL_Land
};

class Mode
{

public:

    // Auto Pilot Modes enumeration
    enum class Number : uint8_t {
        STABILIZE =     0,  // manual angle with manual depth/throttle
        ACRO =          1,  // manual body-frame angular rate with manual depth/throttle
        ALT_HOLD =      2,  // manual angle with automatic depth/throttle
        AUTO =          3,  // fully automatic waypoint control using mission commands
        GUIDED =        4,  // fully automatic fly to coordinate or fly at velocity/direction using GCS immediate commands
        CIRCLE =        7,  // automatic circular flight with automatic throttle
        SURFACE =       9,  // automatically return to surface, pilot maintains horizontal control
        POSHOLD =      16,  // automatic position hold with manual override, with automatic throttle
        MANUAL =       19,  // Pass-through input with no stabilization
        MOTOR_DETECT = 20,  // Automatically detect motors orientation
        SURFTRAK =     21,  // Track distance above seafloor (hold range)
        SMART_RTL =    22,  // AURA: retrace the recorded path back to where we armed
        ANCHOR =       23   // AURA: hold a commanded point, fed live over MAVLink
        // Mode number 30 reserved for "offboard" for external/lua control.
    };

    // constructor
    Mode(void);

    // do not allow copying
    CLASS_NO_COPY(Mode);

    // child classes should override these methods
    virtual bool init(bool ignore_checks) { return true; }
    virtual void run() = 0;
    virtual bool requires_GPS() const = 0;
    virtual bool requires_altitude() const = 0;
    virtual bool allows_arming(bool from_gcs) const = 0;
    virtual bool is_autopilot() const { return false; }
    virtual bool in_guided_mode() const { return false; }

    // return a string for this flightmode
    virtual const char *name() const = 0;
    virtual const char *name4() const = 0;

    // returns a unique number specific to this mode
    virtual Mode::Number number() const = 0;
  
    // pilot input processing
    void get_pilot_desired_angle_rates(int16_t roll_in, int16_t pitch_in, int16_t yaw_in, float &roll_out, float &pitch_out, float &yaw_out);


protected:

    // navigation support functions
    virtual void run_autopilot() {}

    // helper functions
    bool is_disarmed_or_landed() const;

    // functions to control landing
    // in modes that support landing
    void land_run_horizontal_control();
    void land_run_vertical_control(bool pause_descent = false);

    // convenience references to avoid code churn in conversion:
    Parameters &g;
    ParametersG2 &g2;
    AP_InertialNav &inertial_nav;
    AP_AHRS &ahrs;
    AP_Motors6DOF &motors;
    RC_Channel *&channel_roll;
    RC_Channel *&channel_pitch;
    RC_Channel *&channel_throttle;
    RC_Channel *&channel_yaw;
    RC_Channel *&channel_forward;
    RC_Channel *&channel_lateral;
    AC_PosControl *position_control;
    AC_AttitudeControl_Sub *attitude_control;
    // TODO: channels
    float &G_Dt;

public:

    // pass-through functions to reduce code churn on conversion;
    // these are candidates for moving into the Mode base
    // class.
    bool set_mode(Mode::Number mode, ModeReason reason);
    GCS_Sub &gcs();

    // end pass-through functions
};

class ModeManual : public Mode
{

public:
    // inherit constructor
    using Mode::Mode;
    virtual void run() override;
    bool init(bool ignore_checks) override;
    bool requires_GPS() const override { return false; }
    bool requires_altitude() const override { return false; }
    bool allows_arming(bool from_gcs) const override { return true; }
    bool is_autopilot() const override { return false; }

protected:

    const char *name() const override { return "Manual"; }
    const char *name4() const override { return "MANU"; }
    Mode::Number number() const override { return Mode::Number::MANUAL; }
};


class ModeAcro : public Mode
{

public:
    // inherit constructor
    using Mode::Mode;

    virtual void run() override;

    bool init(bool ignore_checks) override;
    bool requires_GPS() const override { return false; }
    bool requires_altitude() const override { return false; }
    bool allows_arming(bool from_gcs) const override { return true; }
    bool is_autopilot() const override { return false; }

protected:

    const char *name() const override { return "Acro"; }
    const char *name4() const override { return "ACRO"; }
    Mode::Number number() const override { return Mode::Number::ACRO; }
};


class ModeStabilize : public Mode
{

public:
    // inherit constructor
    using Mode::Mode;

    virtual void run() override;

    bool init(bool ignore_checks) override;
    bool requires_GPS() const override { return false; }
    bool requires_altitude() const override { return false; }
    bool allows_arming(bool from_gcs) const override { return true; }
    bool is_autopilot() const override { return false; }

protected:

    const char *name() const override { return "Stabilize"; }
    const char *name4() const override { return "STAB"; }
    Mode::Number number() const override { return Mode::Number::STABILIZE; }
};


class ModeAlthold : public Mode
{

public:
    // inherit constructor
    using Mode::Mode;

    virtual void run() override;

    bool init(bool ignore_checks) override;
    bool requires_GPS() const override { return false; }
    bool requires_altitude() const override { return true; }
    bool allows_arming(bool from_gcs) const override { return true; }
    bool is_autopilot() const override { return false; }
    void control_depth();

protected:

    void run_pre();
    void run_post();

    const char *name() const override { return "Depth Hold"; }
    const char *name4() const override { return "ALTH"; }
    Mode::Number number() const override { return Mode::Number::ALT_HOLD; }
};


class ModeSurftrak : public ModeAlthold
{

public:
    // constructor
    ModeSurftrak();

    void run() override;

    bool init(bool ignore_checks) override;

    float get_rangefinder_target_cm() const WARN_IF_UNUSED { return rangefinder_target_cm; }
    bool set_rangefinder_target_cm(float target_cm);

protected:

    const char *name() const override { return "Surftrak"; }
    const char *name4() const override { return "STRK"; }
    Mode::Number number() const override { return Mode::Number::SURFTRAK; }

private:

    void reset();
    void control_range();
    void update_surface_offset();

    float rangefinder_target_cm;

    bool pilot_in_control;            // pilot is moving up/down
    float pilot_control_start_z_cm;   // alt when pilot took control
};

class ModeGuided : public Mode
{

public:
    // inherit constructor
    using Mode::Mode;

    virtual void run() override;

    bool init(bool ignore_checks) override;
    bool requires_GPS() const override { return true; }
    bool requires_altitude() const override { return true; }
    bool allows_arming(bool from_gcs) const override { return true; }
    bool is_autopilot() const override { return true; }
    bool in_guided_mode() const override { return true; }
    bool guided_limit_check();
    void guided_limit_init_time_and_pos();
    void guided_set_angle(const Quaternion &q, float climb_rate_cms, bool use_yaw_rate, float yaw_rate_rads);
    void guided_set_angle(const Quaternion&, float);
    void guided_limit_set(uint32_t timeout_ms, float alt_min_cm, float alt_max_cm, float horiz_max_cm);
    bool guided_set_destination_posvel(const Vector3f& destination, const Vector3f& velocity);
    bool guided_set_destination_posvel(const Vector3f& destination, const Vector3f& velocity, bool use_yaw, float yaw_cd, bool use_yaw_rate, float yaw_rate_cds, bool relative_yaw);
    bool guided_set_destination(const Vector3f& destination);
    bool guided_set_destination(const Location&);
    bool guided_set_destination(const Vector3f& destination, bool use_yaw, float yaw_cd, bool use_yaw_rate, float yaw_rate_cds, bool relative_yaw);
    void guided_set_velocity(const Vector3f& velocity);
    void guided_set_velocity(const Vector3f& velocity, bool use_yaw, float yaw_cd, bool use_yaw_rate, float yaw_rate_cds, bool relative_yaw);
    void guided_set_yaw_state(bool use_yaw, float yaw_cd, bool use_yaw_rate, float yaw_rate_cds, bool relative_angle);
    float get_auto_heading();
    void guided_limit_clear();
    void set_auto_yaw_mode(autopilot_yaw_mode yaw_mode);

protected:

    const char *name() const override { return "Guided"; }
    const char *name4() const override { return "GUID"; }
    Mode::Number number() const override { return Mode::Number::GUIDED; }

    autopilot_yaw_mode get_default_auto_yaw_mode(bool rtl) const;

private:
    void guided_pos_control_run();
    void guided_vel_control_run();
    void guided_posvel_control_run();
    void guided_angle_control_run();
    void guided_takeoff_run();
    void guided_pos_control_start();
    void guided_vel_control_start();
    void guided_posvel_control_start();
    void guided_angle_control_start();
};



// AURA: Smart RTL - return home by retracing the track already travelled.
//
// AP_SmartRTL accumulates 3D breadcrumb points as the vehicle moves (NED metres,
// relative to the EKF origin) and this mode consumes them BACKWARDS. Unlike a
// straight-line RTL it returns along the path travelled: under ice, inside a wreck or
// in a closed structure that is the only safe way back.
//
// Copter's SmartRTL ends with a LANDING; there is no such thing here. Rover's shape was
// taken (StopAtHome): it stops at the last point and holds there. On top of that AURA's
// surface ceiling is applied HARD (-0.30 m): because the last breadcrumb point is the
// position at the moment of arming, i.e. usually the SURFACE, a SmartRTL without a ceiling
// would surface the vehicle at the end of the mission - what this fork explicitly forbids.
// It derives from ModeGuided, not from Mode: the automatic yaw helpers
// (set_auto_yaw_mode / get_default_auto_yaw_mode / get_auto_heading) are defined there.
// ModeAuto uses the same base for exactly this reason.
class ModeSmartRtl : public ModeGuided
{
public:
    using ModeGuided::ModeGuided;

    bool init(bool ignore_checks) override;
    void run() override;

    bool requires_GPS() const override { return true; }
    bool requires_altitude() const override { return true; }
    bool allows_arming(bool from_gcs) const override { return false; }
    bool is_autopilot() const override { return true; }

    // 3 Hz scheduler task: breadcrumb accumulation
    void save_position();

    // on leaving the mode: put the popped but not-yet-reached point back
    void cikis_temizligi();

protected:
    const char *name() const override { return "Smart RTL"; }
    const char *name4() const override { return "SRTL"; }
    Mode::Number number() const override { return Mode::Number::SMART_RTL; }

private:
    enum class Durum : uint8_t {
        YOL_TEMIZLIGI,   // wait until the library's thorough cleanup is finished
        IZI_SUR,         // consume the breadcrumb points backwards
        DERINLIKTE_TUT,  // station-keep at the last point, at depth
    };

    Durum durum;
    Vector3p tuketilmis_nokta_ned_m;   // point popped but not yet reached
    bool tuketilmis_gecerli;
    uint32_t son_basarili_pop_ms;

    void yol_temizligi_kos();
    void izi_sur_kos();
    void tut_kos();
    void wp_cikislarini_sur();
};

// AURA: ANCHOR - hold a point that is fed continuously from outside.
//
// The FLIGHT MODE counterpart of the mission anchor item (MAV_CMD_AURA_ANCHOR 31010).
// That item IS STILL IN PLACE and was not removed; the two use the same station-keeping
// physics but exist for different things: the item is a mission step and takes its
// parameters from the plan, while this mode is fed live from outside (GCS / Jetson).
//
// "Anchor data": SET_POSITION_TARGET_GLOBAL_INT (msg 86), lat/lon/alt + yaw.
// It is sent CONTINUOUSLY, not just once; if it goes quiet (ANCHOR_DTIM) the mode counts
// the link as lost and switches to ANCHOR_MDSW.
//
// Photo/shutter is DELIBERATELY ABSENT: the mission anchor carries it inside itself
// because there the do-command queue ordering was not guaranteed; here the operator
// sends his own command whenever he wants it.
class ModeAnchor : public ModeGuided
{
public:
    using ModeGuided::ModeGuided;

    bool init(bool ignore_checks) override;
    void run() override;

    bool requires_GPS() const override { return true; }
    bool requires_altitude() const override { return true; }
    // Arming in a holding mode is legitimate: the operator can put the vehicle in the water,
    // switch to ANCHOR first and then arm. (SmartRTL says false because it is an escape manoeuvre.)
    bool allows_arming(bool from_gcs) const override { return true; }
    bool is_autopilot() const override { return true; }
    // Inherited from ModeGuided: guided_limit etc. are specific to GUIDED; this mode is not GUIDED.
    bool in_guided_mode() const override { return false; }

    // Anchor data input (called from GCS_MAVLink_Sub.cpp).
    // konum_neu_cm : target relative to the EKF origin (NEU, cm)
    // yaw_var / yaw_cd : whether a heading was given, and if so in centidegrees
    bool demir_verisi_al(const Vector3f &konum_neu_cm, bool yaw_var, float yaw_cd);

protected:
    const char *name() const override { return "Anchor"; }
    const char *name4() const override { return "ANCH"; }
    Mode::Number number() const override { return Mode::Number::ANCHOR; }

private:
    uint32_t giris_ms = 0;          // when the counters started (together with ARM)
    uint32_t son_veri_ms = 0;       // when the last anchor data arrived; 0 = never arrived
    uint32_t son_deneme_ms = 0;     // when we last tried to switch to ANCHOR_MDSW
    uint32_t son_uyari_ms = 0;      // when the refusal warning was last printed
    bool     yeniden_kilitle = false; // came out of disarm -> anchor point must be written back
    bool     hedef_var = false;     // is there a locked point
    bool     satihta_tut = false;   // target depth is shallower than SURFACE_DEPTH
    float    hedef_d_m = 0.0f;      // locked depth (D, positive down, m)
    Vector3p hedef_ned_m;           // locked point (NED, m, relative to the EKF origin)

    void hedefi_uygula(const Vector3p &istenen_ned_m, bool zorla);
    void nokta_kilitle(const Vector3f &konum_neu_cm);
    bool cikis_kosulu_denetle();
    void kontrolcuyu_sur();
};

class ModeAuto : public ModeGuided
{

public:
    // inherit constructor
    using ModeGuided::ModeGuided;

    virtual void run() override;

    bool init(bool ignore_checks) override;
    bool requires_GPS() const override { return true; }
    bool requires_altitude() const override { return true; }
    bool allows_arming(bool from_gcs) const override { return true; }
    bool is_autopilot() const override { return true; }
    bool auto_loiter_start();
    void auto_wp_start(const Vector3f& destination);
    void auto_wp_start(const Location& dest_loc);
    void auto_circle_movetoedge_start(const Location &circle_center, float radius_m, float rate_degs);
    void auto_circle_start();
    float aura_daire_dikey_hiz_ms() const;
    float aura_daire_yaw_cd();
    void auto_nav_attitude_time_start(const AP_Mission::Mission_Command& cmd);
    void guided_overlay_degerlendir();
    void guided_overlay_birak();
    uint32_t nav_attitude_time_start_ms() const { return nav_attitude_time.start_ms; }
    void auto_nav_guided_start();
    void set_auto_yaw_roi(const Location &roi_location);
    void set_auto_yaw_look_at_heading(float angle_deg, float turn_rate_dps, int8_t direction, uint8_t relative_angle);
    void set_yaw_rate(float turn_rate_dps);
    bool auto_terrain_recover_start();
    bool auto_anchor_start();

protected:

    const char *name() const override { return "Auto"; }
    const char *name4() const override { return "AUTO"; }
    Mode::Number number() const override { return Mode::Number::AUTO; }

private:
    // AURA: the mission is not started until ARM. init() sets this true, and run() calls
    // mission.start_or_resume() and clears it to false once the vehicle is armed. See mode_auto.cpp.
    bool gorev_arm_bekliyor = false;

    void auto_wp_run();
    void auto_circle_run();
    void auto_nav_guided_run();
    // AURA: honour_auto_yaw=true lets auto_yaw_mode drive the heading (CONDITION_YAW,
    // ROI); false keeps the stock behaviour where yaw is the pilot's rate stick only.
    void auto_loiter_run(bool honour_auto_yaw = false);
    void auto_terrain_recover_run();
    void auto_nav_attitude_time_run();

    // NAV_ATTITUDE_TIME state
    struct {
        float roll_deg = 0.0f;
        float pitch_deg = 0.0f;
        float yaw_deg = 0.0f;
        float climb_rate_ms = 0.0f;
        uint32_t start_ms = 0;
    } nav_attitude_time;
    void auto_anchor_run();
};


class ModePoshold : public ModeAlthold
{

public:
    // inherit constructor
    using ModeAlthold::ModeAlthold;

    virtual void run() override;

    bool init(bool ignore_checks) override;

    bool requires_GPS() const override { return true; }
    bool requires_altitude() const override { return true; }
    bool allows_arming(bool from_gcs) const override { return true; }
    bool is_autopilot() const override { return true; }

protected:

    const char *name() const override { return "Position Hold"; }
    const char *name4() const override { return "POSH"; }
    Mode::Number number() const override { return Mode::Number::POSHOLD; }

private:

    void control_horizontal();
};


class ModeCircle : public Mode
{

public:
    // inherit constructor
    using Mode::Mode;

    virtual void run() override;

    bool init(bool ignore_checks) override;
    bool requires_GPS() const override { return true; }
    bool requires_altitude() const override { return true; }
    bool allows_arming(bool from_gcs) const override { return true; }
    bool is_autopilot() const override { return true; }

protected:

    const char *name() const override { return "Circle"; }
    const char *name4() const override { return "CIRC"; }
    Mode::Number number() const override { return Mode::Number::CIRCLE; }
};

class ModeSurface : public Mode
{

public:
    // inherit constructor
    using Mode::Mode;

    virtual void run() override;

    bool init(bool ignore_checks) override;
    bool requires_GPS() const override { return false; }
    bool requires_altitude() const override { return false; }
    bool allows_arming(bool from_gcs) const override { return true; }
    bool is_autopilot() const override { return true; }

protected:
    const char *name() const override { return "Surface"; }
    const char *name4() const override { return "SURF"; }
    Mode::Number number() const override { return Mode::Number::SURFACE; }
    bool nobaro_mode;
    // Becomes true after the surface is reached if SURFMDSW says "the same mode" (SURFACE):
    // no climb is commanded any more, the depth is locked to SURFACE_DEPTH.
    bool satihta_tut = false;
};


class ModeMotordetect : public Mode
{

public:
    // inherit constructor
    using Mode::Mode;

    virtual void run() override;

    bool init(bool ignore_checks) override;
    bool requires_GPS() const override { return false; }
    bool requires_altitude() const override { return false; }
    bool allows_arming(bool from_gcs) const override { return true; }
    bool is_autopilot() const override { return true; }

protected:

    const char *name() const override { return "Motor Detection"; }
    const char *name4() const override { return "DETE"; }
    Mode::Number number() const override { return Mode::Number::MOTOR_DETECT; }
};
