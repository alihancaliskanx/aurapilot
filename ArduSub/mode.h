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

// AURA sig-su satih tavani. Tanim mode_auto.cpp'de; SmartRTL de kullanabilsin diye
// dosya-statik olmaktan cikarildi. komut_u_m = bu bacagin komut ettigi dikey hedef
// (U, yukari pozitif, m). terrain_mi = true ise komut edilen hedef DIKKATE ALINMAZ ve
// sabit -0.30 m tavan uygulanir; bu, "hedef derinlik komut etmeyen" bacaklar
// (terrain kurtarmasi, NAV_ATTITUDE_TIME, SmartRTL) icin dogru olan davranistir.
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



// AURA: Smart RTL - gelinen izi geri surerek eve don.
//
// AP_SmartRTL arac ilerledikce 3B kirinti noktalari biriktirir (NED metre, EKF
// orijinine gore) ve bu mod onlari TERSTEN tuketir. Duz hat RTL'in aksine gelinen
// yoldan doner: buz altinda, enkap icinde ya da kapali bir yapida tek guvenli
// donus yolu budur.
//
// Copter'in SmartRTL'i INISLE biter; burada oyle bir sey yok. Rover'in sekli
// alindi (StopAtHome): son noktada durur ve orada tutar. Ustune AURA'nin satih
// tavani SERT olarak uygulanir (-0.30 m): son kirinti noktasi arm anindaki konum,
// yani genellikle SATIH oldugu icin, tavansiz bir SmartRTL gorevin sonunda araci
// yuzeye cikarirdi - bu fork'un acikca yasakladigi sey.
// ModeGuided'dan turuyor, Mode'dan degil: otomatik yaw yardimcilari
// (set_auto_yaw_mode / get_default_auto_yaw_mode / get_auto_heading) orada tanimli.
// ModeAuto da tam olarak bu sebeple ayni tabani kullaniyor.
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

    // 3 Hz zamanlayici gorevi: kirinti biriktirme
    void save_position();

    // moddan cikarken: tuketilmis ama henuz varilmamis noktayi geri koy
    void cikis_temizligi();

protected:
    const char *name() const override { return "Smart RTL"; }
    const char *name4() const override { return "SRTL"; }
    Mode::Number number() const override { return Mode::Number::SMART_RTL; }

private:
    enum class Durum : uint8_t {
        YOL_TEMIZLIGI,   // kutuphanenin kapsamli temizligi bitene kadar bekle
        IZI_SUR,         // kirinti noktalarini tersten tuket
        DERINLIKTE_TUT,  // son noktada, derinlikte istasyon tut
    };

    Durum durum;
    Vector3p tuketilmis_nokta_ned_m;   // pop edilip henuz varilmamis nokta
    bool tuketilmis_gecerli;
    uint32_t son_basarili_pop_ms;

    void yol_temizligi_kos();
    void izi_sur_kos();
    void tut_kos();
    void wp_cikislarini_sur();
};

// AURA: ANCHOR - disaridan surekli beslenen bir noktayi tut.
//
// Gorevdeki demir item'inin (MAV_CMD_AURA_ANCHOR 31010) UCUS MODU karsiligi. O
// item YERINDE DURUYOR ve silinmedi; ikisi ayni istasyon-tutma fizigini kullanir
// ama farkli seyler icindir: item bir gorev adimidir ve parametrelerini plandan
// alir, bu mod ise disaridan (GCS / Jetson) canli beslenir.
//
// "Demir verisi": SET_POSITION_TARGET_GLOBAL_INT (msg 86), lat/lon/alt + yaw.
// Sadece bir kez degil, SUREKLI gonderilir; susarsa (ANCHOR_DTIM) mod baglantiyi
// kopmus sayar ve ANCHOR_MDSW'e gecer.
//
// Foto/deklansor BILEREK YOKTUR: gorev demiri onu icinde tasir cunku orada
// do-komut kuyrugu sirasi garanti degildi; burada operator ne zaman isterse
// kendi komutunu gonderir.
class ModeAnchor : public ModeGuided
{
public:
    using ModeGuided::ModeGuided;

    bool init(bool ignore_checks) override;
    void run() override;

    bool requires_GPS() const override { return true; }
    bool requires_altitude() const override { return true; }
    // Bir tutma modunda arm etmek mesru: operator araci suya birakip once ANCHOR'a
    // alip sonra arm edebilir. (SmartRTL false diyor cunku o bir kacis manevrasi.)
    bool allows_arming(bool from_gcs) const override { return true; }
    bool is_autopilot() const override { return true; }
    // ModeGuided'dan miras: guided_limit vb. GUIDED'e ozgu; bu mod GUIDED degil.
    bool in_guided_mode() const override { return false; }

    // Demir verisi girisi (GCS_MAVLink_Sub.cpp'den cagrilir).
    // konum_neu_cm : EKF orijinine gore hedef (NEU, cm)
    // yaw_var / yaw_cd : bas acisi verildi mi, verildiyse santiderece
    bool demir_verisi_al(const Vector3f &konum_neu_cm, bool yaw_var, float yaw_cd);

protected:
    const char *name() const override { return "Anchor"; }
    const char *name4() const override { return "ANCH"; }
    Mode::Number number() const override { return Mode::Number::ANCHOR; }

private:
    uint32_t giris_ms = 0;          // sayaclarin basladigi an (ARM ile birlikte)
    uint32_t son_veri_ms = 0;       // son demir verisinin geldigi an; 0 = hic gelmedi
    uint32_t son_deneme_ms = 0;     // ANCHOR_MDSW'e en son ne zaman gecmeye calistik
    uint32_t son_uyari_ms = 0;      // reddedilme uyarisi en son ne zaman basildi
    bool     yeniden_kilitle = false; // disarm'dan cikildi -> demir noktasi geri yazilmali
    bool     hedef_var = false;     // kilitli bir nokta var mi
    bool     satihta_tut = false;   // hedef derinlik SURFACE_DEPTH'ten sig
    float    hedef_d_m = 0.0f;      // kilitli derinlik (D, asagi pozitif, m)
    Vector3p hedef_ned_m;           // kilitli nokta (NED, m, EKF orijinine gore)

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
    // AURA: gorev, ARM olana kadar baslatilmaz. init() bunu true yapar, run() arac
    // arm oldugunda mission.start_or_resume() cagirip false'a ceker. Bkz. mode_auto.cpp.
    bool gorev_arm_bekliyor = false;

    void auto_wp_run();
    void auto_circle_run();
    void auto_nav_guided_run();
    // AURA: honour_auto_yaw=true lets auto_yaw_mode drive the heading (CONDITION_YAW,
    // ROI); false keeps the stock behaviour where yaw is the pilot's rate stick only.
    void auto_loiter_run(bool honour_auto_yaw = false);
    void auto_terrain_recover_run();
    void auto_nav_attitude_time_run();

    // NAV_ATTITUDE_TIME durumu
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
    // SURFMDSW "ayni mod" (SURFACE) derse satha varildiktan sonra true olur:
    // artik tirmanma komut edilmez, derinlik SURFACE_DEPTH'e kilitlenir.
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
