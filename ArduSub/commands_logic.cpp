#include "Sub.h"

#include <AP_RTC/AP_RTC.h>

static enum AutoSurfaceState auto_surface_state = AUTO_SURFACE_STATE_GO_TO_LOCATION;

// AURA: how long MAV_CMD_AURA_POSITION_FIX holds the mission when param1 is left at 0.
// Not zero: the reset has to travel through the estimator, AP_InertialNav and the
// position controller before the next leg is computed from it.
static constexpr uint32_t AURA_POSITION_FIX_DEFAULT_DWELL_MS = 1000;

// start_command - this function will be called when the ap_mission lib wishes to start a new command
bool Sub::start_command(const AP_Mission::Mission_Command& cmd)
{
    // Altitude-frame validation only makes sense for commands that actually carry a
    // Location: DO/CONDITION commands (e.g. CONDITION_YAW, DO_DIGICAM_CONTROL) store
    // their payload in the same union as `location`, so reading it as a Location
    // yields garbage and the check below would reject them with "Bad alt frame"
    // (seen with CONDITION_YAW).
    // is_nav_cmd() alone is NOT enough: NAV_DELAY, NAV_RETURN_TO_LAUNCH,
    // NAV_SET_YAW_SPEED, AURA_ANCHOR and AURA_POSITION_FIX are nav commands without a
    // Location.
    // stored_in_location() is the correct predicate.
    // NAV_GUIDED_ENABLE stored_in_location listesindedir ama tasidigi Location
    // ATIL: do_nav_guided_enable yalnizca cmd.p1'i (ac/kapa bayragi) okur, irtifaya
    // hic bakmaz. Kapiya takilmasi gercek bir kusurdu: mavui (ve stok QGC) bu komutu
    // koordinat/irtifa BILDIRMEDEN tanimladigi icin SimpleMissionItem ona
    // MAV_FRAME_MISSION veriyor; o cerceve ABSOLUTE'a cozuluyor, kapi "Bad alt frame"
    // deyip start_command false donduruyor ve AP_Mission komutu SESSIZCE ATLIYOR.
    // Yani menuden konan "Guided enable" hicbir sey yapmiyordu. SITL'de olculdu:
    // "Mission: 2 GuidedEnable" hemen ardindan "Bad alt frame", sonra bir sonraki item.
    const bool irtifasi_atil = (cmd.id == MAV_CMD_NAV_GUIDED_ENABLE);

    if (AP_Mission::is_nav_cmd(cmd) && AP_Mission::stored_in_location(cmd.id) && !irtifasi_atil) {
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
    case MAV_CMD_NAV_VTOL_LAND:         // 85 - Copter'da da NAV_LAND'in takma adi.
                                        // QGC bazi planlarda bunu uretiyor; kabul
                                        // etmezsek satha cikis item'i sessizce atlanir.
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

    case AP_Mission::MAV_CMD_AURA_CIRCLE:       // 31020 daire, kendi parametreleriyle
        do_aura_circle(cmd);
        break;

    case MAV_CMD_NAV_ATTITUDE_TIME:            // 42703 tutumu N saniye koru
        mode_auto.auto_nav_attitude_time_start(cmd);
        break;

    case AP_Mission::MAV_CMD_AURA_GUIDED_MISSION:  // 31025 bacagi dis bilgisayara ver
        do_aura_guided_mission(cmd);
        break;

    case AP_Mission::MAV_CMD_AURA_GUIDED_SETUP:    // 31030 guided overlay ac/kapa (DO)
        do_aura_guided_setup(cmd);
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

    case AP_Mission::MAV_CMD_AURA_POSITION_FIX: // 31015 AURA: snap the solution to a point
        do_position_fix(cmd);
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

    // Isaretci komutlar: kendileri bir sey YAPMAZ, plandaki bir yeri isaretlerler.
    // AP_Mission bayraklari (in_landing_sequence / in_return_path) zaten kendisi
    // kuruyor; ArduSub'in tek yapmasi gereken komutu tanimak. Tanimadigi icin
    // QGC'nin standart olarak urettigi her inis dizisi planinda item BASINA IKI
    // uyari basiliyordu ("Ignoring command 189" + "Skipping invalid cmd #189"),
    // operator de gercek hatalari bu gurultunun icinde kaybediyordu.
    case MAV_CMD_DO_LAND_START:             // 189
    case MAV_CMD_DO_RETURN_PATH_START:      // 188
        break;

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
    case MAV_CMD_NAV_VTOL_LAND:
        return verify_surface(cmd);

    case MAV_CMD_NAV_RETURN_TO_LAUNCH:
        return verify_RTL();

    case MAV_CMD_NAV_LOITER_UNLIM:
        return verify_loiter_unlimited();

    case MAV_CMD_NAV_LOITER_TURNS:
        return verify_circle(cmd);

    case AP_Mission::MAV_CMD_AURA_CIRCLE:
        return verify_aura_circle(cmd);

    case MAV_CMD_NAV_ATTITUDE_TIME:
        return verify_nav_attitude_time(cmd);

    case AP_Mission::MAV_CMD_AURA_GUIDED_MISSION:
        return verify_aura_guided_mission(cmd);

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

    case AP_Mission::MAV_CMD_AURA_POSITION_FIX:
        return verify_position_fix(cmd);

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
    // Isaretciler - start_command tarafindaki gerekceye bak.
    case MAV_CMD_DO_LAND_START:             // 189
    case MAV_CMD_DO_RETURN_PATH_START:      // 188
    // DO_FENCE_ENABLE'i AP_Mission::start_command kendisi yakalar (start_command_fence),
    // yani bu arac fonksiyonuna hic ulasmaz - ama AP_Mission::verify_command onu
    // yakalamaz, buraya DUSER. Case yoksa cit gercekten acilip kapanirken ekrana
    // "Skipping invalid cmd #207" basiliyordu. Copter'da da tam olarak bu sebeple
    // yalniz verify tarafinda bir case var.
    case MAV_CMD_DO_FENCE_ENABLE:           // 207
    // Guided overlay ac/kapa: bir DO komutu, gorevi bloklamaz. Buraya EKLENMEZSE
    // her dongude "Skipping invalid cmd #31030" basardi.
    case AP_Mission::MAV_CMD_AURA_GUIDED_SETUP:
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
    aura_nav_guard_kur(loiter_time_max);
}

// AURA: seyir bacagi icin guard butcesi kur. Butce, duz hat seyir suresinin uc kati
// arti bekleme suresi, asla bir dakikanin altinda degil (gerekce do_nav_wp'de).
// do_nav_wp ve do_RTL ayni butceyi kullanir; formul tek yerde dursun diye ayrildi.
void Sub::aura_nav_guard_kur(uint32_t bekleme_s)
{
    nav_wp_start_ms = AP_HAL::millis();
    const float leg_cm = (wp_nav.get_wp_destination_NEU_cm() - wp_nav.get_wp_origin_NEU_cm()).length();
    const float speed_cms = MAX(wp_nav.get_default_speed_NE_cms(), 10.0f);
    nav_wp_guard_ms = MAX((uint32_t)(3000.0f * leg_cm / speed_cms)
                          + bekleme_s * 1000UL + 30000UL,
                          60000UL);
}

// Guard suresi doldu mu? (0 = guard yok)
bool Sub::aura_nav_guard_doldu() const
{
    return nav_wp_guard_ms != 0 && AP_HAL::millis() - nav_wp_start_ms >= nav_wp_guard_ms;
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

    // Satha cikis da ulasilamayabilir (negatif yuzerlik, aga takilma, buz).
    // Guard olmadan gorev orada kalici olarak park ederdi.
    aura_nav_guard_kur(0);
}

void Sub::do_RTL()
{
    // AURA: eve MEVCUT DERINLIKTE donulur, satha cikilmaz.
    //
    // Eskiden burada dogrudan ahrs.get_home() vardi ve bu araci SATHA cikariyordu:
    // Sub::set_home_to_current_location (commands.cpp) ev noktasini bilerek su
    // yuzeyine tasir ("Make home always at the water's surface", derinlikte
    // disarm/arm yapilabilsin diye), yani get_home().alt HER ZAMAN 0'dir. Hedef
    // olarak verilince arac eve giderken ayni zamanda derinlik 0'a tirmaniyordu -
    // uc kere yanlis: (1) satihta YATAY seyir, AURA kuralinin tam olarak yasakladigi
    // sey; (2) satih tavani da devreye girmiyor, cunku tavan komut edilen hedefi
    // "bilerek istenmis satha cikis" sayip kendini aciyor (mode_auto.cpp); (3)
    // RETURN_TO_LAUNCH bir Location tasimadigi icin start_command'in irtifa-cercevesi
    // denetimi de bu komutu hic gormuyor. SITL'de olculdu: -10 m'den -0.04 m'ye
    // duzgun bir tirmanis.
    //
    // Satha cikmak isteyen plan bunu ACIKCA soyler: RTL'den sonra NAV_LAND (do_surface).
    // DERINLIGI DONUSTURMEDEN, YENIDEN ETIKETLEYEREK al - do_surface (yukarida) da
    // birebir boyle yapiyor.
    //
    // get_alt_cm(ABOVE_HOME) BURADA YANLIS OLUR ve denenip geri alindi: current_loc.alt
    // aslinda EKF ORIJININE gore yukseklik (read_inertia: inertial_nav.get_position_z_up_cm())
    // ama current_loc varsayilan kurulmus oldugu icin cerceve bayraklari 0, yani
    // "ABSOLUTE" der. Gercek bir donusum bu sayiyi AMSL sanip ev irtifasini cikarir;
    // hedef sonra ABOVE_ORIGIN'e cevrilirken orijin irtifasi da cikar. Net hata tam
    // olarak EKF ORIJININ AMSL IRTIFASI kadardir: deniz seviyesinde 0 (bu yuzden
    // SITL'de --home ...,0 ile GORUNMEZ), 584 m rakimli bir golde ise -10 m'lik bir
    // RTL "594 m derinlige in" komutuna doner. Tum ArduSub bu yanlis etiketle yasar
    // ve sayiyi donusturmez, yeniden etiketler; buranin da oyle yapmasi sart.
    Location target_loc(ahrs.get_home());
    target_loc.set_alt_cm(current_loc.alt, Location::AltFrame::ABOVE_HOME);

    mode_auto.auto_wp_start(target_loc);

    // verify_RTL'in de bir kacis kapisi olsun (verify_nav_wp ile ayni gerekce):
    // reached_wp_destination() tek yonlu bir mandal, ulasilamayan bir eve giden RTL
    // gorevi kalici olarak park ediyordu.
    aura_nav_guard_kur(0);
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

    // NAV_LOITER_UNLIM'in verify'i HIC true donmez - komutun anlami bu. Ama bu,
    // bu dosyadaki tek kacissiz nav komutu demek: diger her sey ya tamamlanir ya
    // da guard tavaniyla dusar. GCS'siz bir gorevde (FS_GCS_ENABLE=0) modu
    // degistirecek operator de yoktur; arac batarya failsafe'ine kadar bekler.
    // Davranis DEGISTIRILMEDI (komutun tanimi bu), ama sessiz kalmasi yanlis:
    // QGC'nin "Loiter" deseni bunu operator planina kolayca sokuyor.
    gcs().send_text(MAV_SEVERITY_WARNING,
                    "LoiterUnlim: mission holds here until the mode is changed");
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
    // x10 olcegi NAV_LOITER_TURNS'e ozgu bir depolama numarasi (AP_Mission.cpp:1140).
    // type_specific_bits baska komutlarda baska anlama gelebilir, o yuzden Copter gibi
    // komut kimligi de kontrol edilir.
    if (cmd.id == MAV_CMD_NAV_LOITER_TURNS && (cmd.type_specific_bits & (1U << 0))) {
        circle_radius_m *= 10;
    }


    // true if circle should be ccw
    const bool circle_direction_ccw = cmd.content.location.loiter_ccw;

    // NAV_LOITER_TURNS item basina hiz TASIYAMAZ (p1 dolu, bkz. MAV_CMD_AURA_CIRCLE
    // aciklamasi), o yuzden buyukluk parametreden gelir; item yalnizca yonu soyler.
    const float rate_degs = circle_direction_ccw ? -fabsf(circle_nav.get_rate_degs())
                                                 :  fabsf(circle_nav.get_rate_degs());

    // Bu komut yaw kipi tasimaz: klasik davranis "merkeze bak".
    daire_yaw_kip = 0;
    daire_tur_hedefi = cmd.get_loiter_turns();

    // move to edge of circle (verify_circle) will ensure we begin circling once we reach the edge
    mode_auto.auto_circle_movetoedge_start(circle_center, circle_radius_m, rate_degs);

    // Kenara gitme bacagi icin guard (AURA_CIRCLE'da da ayni).
    aura_nav_guard_kur(0);
}

// AURA: bir index'ten itibaren, konum TASIYAN ilk nav komutunu bul.
//
// do_position_fix'teki yuruteci ile ayni: konum tasimayan nav komutlarinin (demir,
// NAV_DELAY, daire) ustunden atlar ve DO_JUMP'in geriye zincirlemesine karsi hop
// sayisiyla sinirlidir.
bool Sub::aura_sonraki_konumlu_wp(uint16_t index, Location &konum)
{
    AP_Mission::Mission_Command next;
    uint16_t search_index = index;
    for (uint8_t hop = 0; hop < 8; hop++) {
        if (!mission.get_next_nav_cmd(search_index, next)) {
            return false;
        }
        if (AP_Mission::stored_in_location(next.id)) {
            konum = next.content.location;
            return true;
        }
        if (next.index < search_index) {
            return false;   // a jump sent the search backwards - do not chase it
        }
        search_index = next.index + 1;
    }
    return false;
}

// AURA: MAV_CMD_AURA_CIRCLE - daire, CIRCLE modunun kendi parametreleriyle.
void Sub::do_aura_circle(const AP_Mission::Mission_Command& cmd)
{
    const AP_Mission::Aura_Circle_Command &d = cmd.content.aura_circle;

    // ---- merkez ----
    // Komut bir Location TASIMAZ (16 bitlik id -> 10 bayt, PackedLocation 12 bayt
    // ister). Varsayilan merkez, item basladigindaki arac konumudur: normal kullanim
    // "hedefe git, sonra etrafinda don" oldugu icin bu zaten dogru noktadir.
    // centre_from_wp ile merkez, plandaki bir sonraki konumlu nav komutundan okunur
    // (AURA_POSITION_FIX'in kullandigi yontem).
    Location merkez(current_loc);
    if (d.centre_from_wp) {
        Location wp_konum;
        if (aura_sonraki_konumlu_wp(cmd.index + 1, wp_konum)) {
            merkez.lat = wp_konum.lat;
            merkez.lng = wp_konum.lng;
        } else {
            gcs().send_text(MAV_SEVERITY_WARNING,
                            "Circle: no waypoint after it, centring here");
        }
    }

    // ---- derinlik ----
    // 0 = "irtifa verilmedi, mevcut derinligi koru" (do_nav_wp ile ayni gelenek).
    //
    // POZITIF derinlik REDDEDILIR. Bu komut bir Location tasimadigi icin
    // start_command'in basindaki irtifa-cercevesi denetiminden GECMEZ
    // (stored_in_location degil), yani "Alt above home must be negative" korumasi ona
    // ulasmaz. Korumasiz birakilamaz: satih tavani komut edilen hedefi "bilerek
    // istenmis satha cikis" sayip kendini ona ACAR, dolayisiyla plan ureticisindeki
    // bir isaret hatasi (z=+3 yerine -3) araci satihta, surekli yukari itkiyle bir tam
    // tur dondururdu - AURA'nin acikca yasakladigi sey.
    if (d.depth_cm > 0) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Circle: alt above home must be negative");
        return;
    }
    merkez.set_alt_cm(d.depth_cm != 0 ? d.depth_cm : current_loc.alt,
                      Location::AltFrame::ABOVE_HOME);

    // ---- yaw kipi ----
    // Kip 2 sabit bir yon ister; yon verilmemisse (x < 0, demirle ayni gelenek)
    // "merkeze bak"a duselim. Aksi halde -1 sessizce 0'a, yani KUZEY'e donerdi.
    daire_yaw_kip = (d.yaw_mode == 2 && !d.yaw_valid) ? 0 : d.yaw_mode;
    daire_yaw_cd = d.yaw_deg * 100.0f;
    // AC_Circle'in kendi yaw'i (merkeze bakma) yalniz kip 0 ve 3'te kullanilir;
    // digerlerinde auto_circle_run bu degeri kullanir.

    // ---- tur sayisi ----
    daire_tur_hedefi = (d.turns_centi > 0) ? d.turns_centi * 0.01f : 1.0f;

    const float yaricap_m = d.radius_cm * 0.01f;    // 0 -> CIRCLE_RADIUS_M
    const float rate_degs = d.rate_cdegs * 0.01f;   // 0 -> CIRCLE_RATE (isaretiyle)

    // NOT: istenen acisal hiz bir UST SINIRDIR. AC_Circle::calc_velocities onu ayrica
    // pozisyon kontrolcusunun hiz/ivme limitleriyle kirpar: ulasilabilir en yuksek hiz
    // kabaca WP_SPD / yaricap'tir. WP_SPD=0.6 m/s ile 0.5 m yaricapta ~69 deg/s
    // mumkunken 10 m yaricapta ~3.4 deg/s'de doyar. Buyuk yaricapli bir dairede
    // istenen hizin tutmamasi hata degil, bu sinirdir.
    gcs().send_text(MAV_SEVERITY_INFO, "Circle: r=%.2fm %.2fdeg/s %.2f turns",
                    (double)(is_zero(yaricap_m) ? circle_nav.get_radius_parm_m() : yaricap_m),
                    (double)(is_zero(rate_degs) ? circle_nav.get_rate_degs() : rate_degs),
                    (double)daire_tur_hedefi);

    mode_auto.auto_circle_movetoedge_start(merkez, yaricap_m, rate_degs);

    // Kenara gitme bacagi olusabilir; guard butcesi o bacak icin.
    aura_nav_guard_kur(0);
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

    // verify_nav_wp ile ayni gerekce: reached_wp_destination() tek yonlu bir
    // mandal, ulasilamayan bir noktada gorev kalici olarak park ederdi.
    aura_nav_guard_kur(loiter_time_max);
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

    // Everything above restarts, but the shutter must not. Leaving AUTO and coming
    // back re-runs the current NAV command (AP_Mission::resume -> set_current_cmd ->
    // start_command), so a plain reset here fired a second frame of the same target:
    // measured in SITL, a two-photo plan produced three files and the wpNNN -> target
    // mapping in gorev_eslesme.csv stopped being one-to-one. Only clear the flag when
    // this is a different anchor than the one already photographed.
    // (A mission that DO_JUMPs back to the same index on purpose would be treated as
    // already shot; no generator emits one.)
    if (cmd.index != anchor_photo_cmd_index) {
        anchor_photo_done = false;
    }

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

// AURA: do_position_fix - snap the navigation solution onto a known point.
//   param1 -> dwell_ms    : wait after the reset before advancing (0 = 1 s)
//   param2 -> accuracy_cm : 1-sigma position uncertainty given to the EKF (0 = default)
// The coordinate is deliberately NOT carried by this command: it is taken from the
// next nav command that stores a Location. Dropped in front of the waypoint the
// vehicle is physically sitting on, the item means "you are standing on that point" -
// the same operation as SonarView's Set Location. A DVL solution drifts, and without
// this correction every leg after the drift inherits the error.
// "Next nav command that stores a Location" is meant literally: nav commands that
// carry no Location (AURA_ANCHOR, NAV_DELAY, ...) are stepped over rather than ending
// the search.
void Sub::do_position_fix(const AP_Mission::Mission_Command& cmd)
{
    posfix_start_ms = AP_HAL::millis();
    posfix_skipped = false;
    posfix_relocked = false;

    // Walk forward to the first nav command that actually stores a Location. It is not
    // always the immediate next one: AURA_ANCHOR is a nav command with no Location, and
    // the start gate deliberately puts one in between - the mission opens with
    // fix -> anchor -> fix -> dive in place, so the hold happens on an already
    // corrected solution and the second fix clears whatever that hold drifted.
    // Stopping at the first location-less nav command skipped the leading fix outright
    // ("no waypoint after it"), which is why hand-written plans grew a dummy waypoint
    // between the two - given a positive altitude so ArduSub would reject it - purely
    // to carry the coordinate. Stepping over them is safe: nothing between here and
    // that waypoint moves the vehicle, an anchor holds station and a delay waits.
    AP_Mission::Mission_Command next;
    bool found = false;
    uint16_t search_index = cmd.index + 1;
    // Bounded rather than "to the end of the mission": get_next_nav_cmd() follows
    // DO_JUMPs, so a mission that jumps backwards could hand back the same command for
    // ever. Eight hops is far more than any generated pattern puts between a fix and
    // its waypoint.
    for (uint8_t hop = 0; hop < 8; hop++) {
        if (!mission.get_next_nav_cmd(search_index, next)) {
            break;
        }
        if (AP_Mission::stored_in_location(next.id)) {
            found = true;
            break;
        }
        if (next.index < search_index) {
            break;      // a jump sent the search backwards - do not chase it
        }
        search_index = next.index + 1;
    }
    if (!found) {
        // Nothing to read the coordinate from. Skipping keeps a mission that was
        // edited down to nothing from stalling on an item that can never complete.
        gcs().send_text(MAV_SEVERITY_WARNING, "Position fix: no waypoint after it, skipped");
        posfix_skipped = true;
        return;
    }

#if AP_AHRS_POSITION_RESET_ENABLED
    // Horizontal only - NavEKF3_core::setLatLng ignores the altitude element, which is
    // right: depth comes from the barometer and is never the thing that drifted.
    const Location &loc = next.content.location;
    // NaN means "accuracy unknown", and the EKF then falls back to its own position
    // noise parameter. That is the honest default for an operator-placed waypoint.
    const float accuracy_m = cmd.content.aura_position_fix.accuracy_cm > 0
                             ? cmd.content.aura_position_fix.accuracy_cm * 0.01f
                             : nanf("");
    if (!ahrs.handle_external_position_estimate(loc, accuracy_m, AP_HAL::millis())) {
        // NavEKF3_core::setLatLng refuses in two cases, and both are worth shouting
        // about rather than silently continuing on the drifted solution:
        //   - a GPS-class source is still passing its position checks (UGPS, GPS_INPUT,
        //     and the fake GPS in SITL). The reset is only for a dead-reckoning
        //     solution, i.e. the DVL/velocity-aided case this command exists for.
        //   - there is no aiding at all, or no EKF origin yet.
        gcs().send_text(MAV_SEVERITY_ERROR, "Position fix: REJECTED by EKF (GPS aiding?)");
        posfix_skipped = true;
        return;
    }
    gcs().send_text(MAV_SEVERITY_INFO, "Position fix: %.7f %.7f",
                    (double)loc.lat * 1.0e-7, (double)loc.lng * 1.0e-7);
#else
    gcs().send_text(MAV_SEVERITY_ERROR, "Position fix: not built into this firmware");
    posfix_skipped = true;
#endif
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
    // Guided overlay devredeyken bu bacak ILERLEMEZ.
    //
    // Bu bir suslemek degil, ZORUNLULUK: overlay Guided_WP alt-modunu kullaniyor ve
    // o alt-mod wp_nav'in hedefini GUIDED noktasiyla degistiriyor. Dolayisiyla
    // reached_wp_destination() guided hedefine gore mandallanir ve asagidaki kontrol
    // "gercek waypoint'e varildi" sanip gorevi ilerletirdi - arac oraya hic
    // gitmemisken. mission.update() overlay sirasinda da kosmaya devam ettigi icin
    // bu kacinilmazdi.
    if (guided_overlay_etkin) {
        return false;
    }

    // AURA: guard ceiling first - the mission must never park on a waypoint the
    // vehicle cannot reach (see do_nav_wp for the budget). reached_wp_destination()
    // is a one-way latch on a 3D WPNAV_RADIUS test, so without this there is no
    // escape at all. Report how far short we gave up: that number is the whole
    // diagnosis when it fires.
    if (aura_nav_guard_doldu()) {
        gcs().send_text(MAV_SEVERITY_WARNING, "WP #%i: guard time expired, %.2fm short",
                        cmd.index,
                        (double)(wp_nav.get_wp_distance_to_destination_cm() * 0.01f));
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
    if (aura_nav_guard_doldu()) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Surface: guard time expired, %.2fm depth",
                        (double)(sub.current_loc.alt * 0.01f));
        return true;
    }

    bool retval = false;

    switch (auto_surface_state) {
        case AUTO_SURFACE_STATE_GO_TO_LOCATION:
            // check if we've reached the location
            if (wp_nav.reached_wp_destination()) {
                // Set target to current xy and zero depth
                // TODO get xy target from current wp destination, because current location may be acceptance-radius away from original destination
                Location target_location(cmd.content.location.lat, cmd.content.location.lng, 0, Location::AltFrame::ABOVE_HOME);

                mode_auto.auto_wp_start(target_location);

                // Guard'i IKINCI bacak icin yeniden kur. do_surface'te kurulan
                // butce yalniz ilk bacagin (konuma gitme) uzunlugundan hesaplanmisti;
                // derin bir tirmanis onu asabilir ve gorev sebepsiz dusrdu.
                aura_nav_guard_kur(0);

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

// AURA: guided verisi son esik_ms icinde geldi mi?
bool Sub::guided_verisi_taze(uint32_t esik_ms) const
{
    if (guided_veri_ms == 0) {
        return false;       // hic gelmedi
    }
    return (AP_HAL::millis() - guided_veri_ms) <= esik_ms;
}

// AURA: MAV_CMD_AURA_GUIDED_MISSION (31025)
//
// Bacagi dis bilgisayara devreder ve o SUSUNCA gorevi surdurur.
//
// NAV_GUIDED_ENABLE (92) de kontrolu devrediyor ama TEK cikisi bir
// DO_GUIDED_LIMITS ihlali: onune limit konmazsa gorev o item'da SONSUZA KADAR
// park eder ve veri kesilmesi diye bir cikis hic yoktur. Dalis ortasinda yeniden
// baslayabilen bir yardimci bilgisayar icin bu yanlis basarisizlik bicimi.
void Sub::do_aura_guided_mission(const AP_Mission::Mission_Command& cmd)
{
    // Auto_NavGuided alt-modu: MOD DEGISMEZ, arac AUTO'da kalir. Mod degistirmek
    // gorevi dondururdu - verify_command_callback AUTO disinda daima false doner.
    mode_auto.auto_nav_guided_start();

    // Sayaci bu andan baslat. Damgayi SIFIRLAMIYORUZ: item'dan hemen once gelmis
    // bir setpoint de gecerli sayilmali.
    nav_wp_start_ms = AP_HAL::millis();

    gcs().send_text(MAV_SEVERITY_INFO, "GuidedMission: waiting for setpoints");
}

bool Sub::verify_aura_guided_mission(const AP_Mission::Mission_Command& cmd)
{
    const uint32_t simdi = AP_HAL::millis();
    const uint32_t esik_ms = cmd.content.aura_guided_mission.timeout_ms > 0
                             ? cmd.content.aura_guided_mission.timeout_ms
                             : 3000;

    // Azami sure tavani (0 = tavan yok).
    const uint16_t azami_s = cmd.content.aura_guided_mission.max_time_s;
    if (azami_s > 0 && (simdi - nav_wp_start_ms) > (uint32_t)azami_s * 1000UL) {
        gcs().send_text(MAV_SEVERITY_INFO, "GuidedMission: max time, moving on");
        return true;
    }

    // Veri sessizligi. Sayac ILK VERIDEN degil ITEM GIRISINDEN isler: bu bir gorev
    // item'i, dis bilgisayar hic konusmazsa gorevin orada asili kalmasi kabul
    // edilemez. Operator esigi buna gore secmeli (yardimci bilgisayarin konusmaya
    // baslamasi icin gereken sureden buyuk).
    const uint32_t referans = (guided_veri_ms > nav_wp_start_ms) ? guided_veri_ms : nav_wp_start_ms;
    if ((simdi - referans) > esik_ms) {
        gcs().send_text(MAV_SEVERITY_INFO, "GuidedMission: setpoints stopped, moving on");
        return true;
    }
    return false;
}

// AURA: MAV_CMD_AURA_GUIDED_SETUP (31030) - guided overlay'i ac/kapa.
//
// Bir DO komutu: gorevi bloklamaz, yalnizca bayrak kurar. Gorev listesinde
// sonradan kapali bir tanesi konursa o andan itibaren devre disi kalir, tekrar
// acik konursa yeniden calisir - yani dinamik.
void Sub::do_aura_guided_setup(const AP_Mission::Mission_Command& cmd)
{
    guided_overlay_acik = (cmd.content.aura_guided_setup.enable != 0);
    guided_overlay_zaman_ms = cmd.content.aura_guided_setup.timeout_ms > 0
                              ? cmd.content.aura_guided_setup.timeout_ms
                              : 3000;

    if (!guided_overlay_acik && guided_overlay_etkin) {
        // Kapatildi ama su an devredeydi: bacagi derhal geri ver.
        mode_auto.guided_overlay_birak();
    }

    gcs().send_text(MAV_SEVERITY_INFO, "GuidedSetup: overlay %s",
                    guided_overlay_acik ? "on" : "off");
}

// NAV_ATTITUDE_TIME: sure dolunca tamamlanir.
bool Sub::verify_nav_attitude_time(const AP_Mission::Mission_Command& cmd)
{
    return (AP_HAL::millis() - mode_auto.nav_attitude_time_start_ms())
           > (cmd.content.nav_attitude_time.time_sec * 1000UL);
}

bool Sub::verify_RTL() {
    if (aura_nav_guard_doldu()) {
        gcs().send_text(MAV_SEVERITY_WARNING, "RTL: guard time expired, %.2fm short",
                        (double)(wp_nav.get_wp_distance_to_destination_cm() * 0.01f));
        return true;
    }
    return wp_nav.reached_wp_destination();
}

bool Sub::verify_loiter_unlimited()
{
    return false;
}

// verify_loiter_time - check if we have loitered long enough
bool Sub::verify_loiter_time()
{
    if (aura_nav_guard_doldu()) {
        gcs().send_text(MAV_SEVERITY_WARNING, "LoiterTime: guard time expired, %.2fm short",
                        (double)(wp_nav.get_wp_distance_to_destination_cm() * 0.01f));
        return true;
    }

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
        if (aura_nav_guard_doldu()) {
            gcs().send_text(MAV_SEVERITY_WARNING, "Circle #%i: guard expired, starting anyway",
                            cmd.index);
            mode_auto.auto_circle_start();
            return false;
        }
        if (wp_nav.reached_wp_destination()) {
            // Buradaki circle_center hesabi OLU KODDU: degisken kuruluyor, sartli olarak
            // .xy()'si eziliyor ve hicbir yerde kullanilmadan kapsam disina cikiyordu.
            // Merkez zaten do_circle -> auto_circle_movetoedge_start -> set_center()
            // yolunda circle_nav'a yazildi; auto_circle_start() argumansizdir ve merkezi
            // oradan okur. Kaldirildi.
            mode_auto.auto_circle_start();
        }
        return false;
    }
    // check if we have completed circling
    return fabsf(sub.circle_nav.get_angle_total_rad()/M_2PI) >= daire_tur_hedefi;
}

// AURA: MAV_CMD_AURA_CIRCLE dogrulamasi. Tur sayimi verify_circle ile ayni; ayri
// durmasinin sebebi kenar bacagindaki guard kapisi.
bool Sub::verify_aura_circle(const AP_Mission::Mission_Command& cmd)
{
    if (auto_mode == Auto_CircleMoveToEdge) {
        if (aura_nav_guard_doldu()) {
            gcs().send_text(MAV_SEVERITY_WARNING, "Circle #%i: guard expired, starting anyway",
                            cmd.index);
            mode_auto.auto_circle_start();
            return false;
        }
        if (wp_nav.reached_wp_destination()) {
            mode_auto.auto_circle_start();
        }
        return false;
    }

    return fabsf(sub.circle_nav.get_angle_total_rad()/M_2PI) >= daire_tur_hedefi;
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
    // The settle gate is the first thing that has to pass and it wants an
    // UNINTERRUPTED settle_time, so it belongs in the budget. Leaving it out meant a
    // long settle could not fit under its own ceiling: with the default start anchor
    // (duration 0, no photo, no turn) the budget is 30 s, so an anchorSettle of 30 s
    // or more made the command mathematically unable to finish - it always ended on
    // "guard time expired" and silently degraded to a plain wait.
    uint32_t guard_ms = anchor.guard_time_s * 1000UL;
    if (guard_ms == 0) {
        // Auto ceiling. The turn and the pre-shutter wait happen inside the anchor,
        // so they are part of the budget: a plain duration*3 + 30 could expire
        // mid-turn and the shutter would never fire.
        guard_ms = (anchor.duration_s + anchor.photo_delay_s + anchor.settle_time_s) * 3000UL
                   + 30000UL;
        if (anchor.yaw_valid) {
            // a 180 deg turn at the default slew rate takes the better part of 20 s
            guard_ms += 30000UL;
        }
    } else {
        // An operator-supplied ceiling is not validated anywhere else. Below the sum
        // of the stages it can never be met: guard 2 s with settle 3 s was measured
        // in SITL to skip "Anchor: settled" entirely and go straight to an off-aim
        // frame. Keep the operator's value as a floor-checked ceiling instead.
        const uint32_t asgari = (anchor.duration_s + anchor.photo_delay_s +
                                 anchor.settle_time_s) * 1000UL + 5000UL;
        guard_ms = MAX(guard_ms, asgari);
    }
    if (now - anchor_start_ms >= guard_ms) {
        // Last resort: the heading gate below is enforced continuously, so a vehicle
        // that can never hold the aim would otherwise reach this point having taken
        // no photo at all. A frame off-aim still beats an empty slot in the survey,
        // and the distinct text says which one it was.
        // Only while armed. In AUTO the mission runs regardless of arm state
        // (ModeAuto::run calls mission.update() unconditionally) and a disarmed
        // vehicle cannot turn, so the heading gate never opens and this branch used
        // to fire at every anchor - a disarmed sub sitting in AUTO quietly worked its
        // way through the whole mission taking garbage frames.
        if (anchor.take_photo && !anchor_photo_done && motors.armed()) {
            anchor_photo_done = true;
            anchor_photo_cmd_index = cmd.index;
#if AP_CAMERA_ENABLED
            const bool cekildi = camera.take_picture();
#else
            const bool cekildi = false;
#endif
            gcs().send_text(cekildi ? MAV_SEVERITY_WARNING : MAV_SEVERITY_ERROR,
                            cekildi ? "Anchor: photo off-aim (guard)"
                                    : "Anchor: PHOTO FAILED (guard)");
        }
        gcs().send_text(MAV_SEVERITY_WARNING, "Anchor: guard time expired");
        return true;
    }

    // 2) settle gate: horizontal distance to the locked target must stay inside
    // the radius CONTINUOUSLY (get_wp_distance_to_destination is horizontal only)
    const uint16_t radius_cm = anchor.settle_radius_cm;
    if (radius_cm > 0) {
        if (wp_nav.get_wp_distance_to_destination_cm() > (float)radius_cm) {
            anchor_settle_ms = 0;      // left the radius -> reset the counter
            // The pre-shutter wait has to restart too. Without this, drifting out
            // and back in "paid" for the swing-settling time with time spent outside
            // the radius: the shutter fired the instant the vehicle re-settled.
            if (anchor.take_photo && !anchor_photo_done) {
                anchor_ready_ms = 0;
            }
            return false;
        }
        if (anchor_settle_ms == 0) {
            anchor_settle_ms = now;
        }
        if (now - anchor_settle_ms < anchor.settle_time_s * 1000UL) {
            return false;
        }
    } else if (anchor_settle_ms == 0) {
        // Gate disabled. settle_time still has to mean something, otherwise a
        // radius of 0 collapsed the whole command: with duration 0 the anchor
        // finished in a single 400 Hz tick while still printing "settled", so a
        // departure gate could vanish and the log read as if it had worked.
        anchor_settle_ms = now;
    }
    if (radius_cm == 0 && now - anchor_settle_ms < anchor.settle_time_s * 1000UL) {
        return false;
    }
    if (!anchor_settled) {
        anchor_settled = true;
        gcs().send_text(MAV_SEVERITY_INFO,
                        radius_cm > 0 ? "Anchor: settled" : "Anchor: waited (no gate)");
    }

    // The heading was set once in do_anchor() and only read from here on, so anything
    // that overwrote auto_yaw_mode in between - a DO_SET_ROI in the same mission, an
    // operator CONDITION_YAW from the GCS, a CONDITION_YAW queued just before this
    // anchor - left the gate measuring a target the controller was no longer flying,
    // which locks it until the guard fires. verify_yaw() re-asserts for exactly this
    // reason; do it here too, value included.
    if (anchor.yaw_valid && auto_yaw_mode != AUTO_YAW_LOOK_AT_HEADING) {
        mode_auto.set_auto_yaw_look_at_heading(anchor.yaw_deg, 0.0f, 0, 0);
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
    // A turn always needs a moment to stop swinging, even when photo_delay is 0:
    // with 0 the shutter fired on the very tick the nose first crossed into the
    // window, i.e. at maximum angular rate, right before the overshoot. mavui lets
    // the operator set photoBefore to 0, so this is reachable from the UI.
    uint32_t bekleme_ms = anchor.photo_delay_s * 1000UL;
    if (anchor.yaw_valid) {
        bekleme_ms = MAX(bekleme_ms, 1000UL);
    }
    if (anchor.take_photo && now - anchor_ready_ms < bekleme_ms) {
        return false;
    }

    // 4) shutter, exactly once. This is the same code path the DO_DIGICAM_CONTROL
    // mission item took (AP_Mission::start_command_camera -> camera.take_picture),
    // so CAM1_TYPE = MAVLink is still what makes the command reach the network.
    if (anchor.take_photo && !anchor_photo_done) {
        anchor_photo_done = true;
        anchor_photo_cmd_index = cmd.index;
#if AP_CAMERA_ENABLED
        // The return value matters: take_picture() fails when no camera backend
        // exists (CAM1_TYPE = 0, or the parameter was changed without a reboot -
        // AP_Camera::init only runs at boot and the parameter carries no
        // @RebootRequired). Announcing "Anchor: photo" regardless meant the mission
        // reported success while nothing reached the network - confirmed in SITL
        // with CAM1_TYPE=0: four anchors, four "Anchor: photo", zero 203 on the wire.
        const bool cekildi = camera.take_picture();
#else
        const bool cekildi = false;
#endif
        gcs().send_text(cekildi ? MAV_SEVERITY_INFO : MAV_SEVERITY_ERROR,
                        cekildi ? "Anchor: photo" : "Anchor: PHOTO FAILED (no camera)");
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

// AURA: verify_position_fix - wait out the dwell after the solution was moved.
// The reset moves the vehicle's ESTIMATE, not the vehicle. Whatever the previous leg
// left in wp_nav is a point in the old frame, so holding it would make the sub fly the
// reset delta to reach a place it is already sitting on. auto_loiter_start() re-locks
// onto the post-reset stopping point, and it runs here rather than in do_position_fix()
// because of loop ordering: read_inertia() has already run for this iteration when
// mission.update() is reached, so AP_InertialNav still reports the pre-reset position
// until the next loop. The dwell then gives the estimator and the controller a moment
// to settle before the next leg is computed from the corrected solution.
bool Sub::verify_position_fix(const AP_Mission::Mission_Command& cmd)
{
    if (posfix_skipped) {
        return true;
    }

    if (!posfix_relocked) {
        posfix_relocked = true;
        if (!mode_auto.auto_loiter_start()) {
            // Only fails without a position source, and do_position_fix() already
            // needed one to get this far. Say it rather than hold the mission: the
            // previous leg's controller stays in charge and the dwell still expires.
            gcs().send_text(MAV_SEVERITY_WARNING, "Position fix: could not re-lock hold");
        }
    }

    const uint32_t dwell_ms = cmd.content.aura_position_fix.dwell_ms > 0
                              ? cmd.content.aura_position_fix.dwell_ms
                              : AURA_POSITION_FIX_DEFAULT_DWELL_MS;
    return (AP_HAL::millis() - posfix_start_ms) >= dwell_ms;
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
    // AURA: keep condition_value in centimetres.  verify_within_distance() compares it
    // against get_wp_distance_to_destination_cm(), so upstream dropping the *100 during
    // the 4.7 unit rework (4.5.3 had "meters * 100") made every threshold 100x too small
    // -- CONDITION_DISTANCE 5 became "5 cm to go", which never satisfies and hangs the
    // mission on that item.  condition_value is int32_t, so cm also keeps sub-metre precision.
    condition_value  = cmd.content.distance.meters * 100;
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
