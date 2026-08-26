#include "Sub.h"

/*
 * control_auto.cpp
 *  Contains the mission, waypoint navigation and NAV_CMD item implementation
 *
 *  While in the auto flight mode, navigation or do/now commands can be run.
 *  Code in this file implements the navigation commands
 */

// AURA sig-su satih tavani (GOREV_ALGORITMASI.md §7): SEYIRDE dikey hedef
// satihtan en az 0.3 m derinlikte tutulur — arac su altinda ilerlerken
// istemeden satha cikmasin. Sig suda terrain hedefi ("tabandan +1 m") su
// seviyesinin USTUNE dusebilir -> arac satihta durmadan yukari iterdi.
//
// ISTISNA: gorevin ACIKCA komutladigi satih waypoint'i (ornegin alt = -0.1 m)
// tavandan sig ise ona izin verilir; tavan yalnizca ISTEMEDEN satha cikmayi
// engeller, satha cikma emrini bogmaz. (Satih WP'si -0.1 m'dir: govde ~35 cm,
// derinlik sensoru dikey ortada -> sensor 10 cm su altinda kalirken kamera
// suyun disinda olur.) Istisna yalnizca terrain-frame OLMAYAN bacaklar icin:
// terrain bacaginda hedef z tabandan olcudur, satih tavaniyla kiyaslanamaz.
//
// 4.5'teki get/set_pos_target_z_cm ikilisi 4.7'de yok: nihai hedef yalniz
// D_update_controller() icinde kuruluyor (AC_PosControl.cpp:1094,
// _pos_target = desired + offset + terrain) ve disariya sadece desired yazilabilir.
// DIKKAT: bu fonksiyon update_wpnav() ile D_update_controller() ARASINDA calisir,
// yani get_pos_target_NED_m() bir dongu BAYAT deger dondurur. Bayat hedeften
// "fark" cikarmak periyot-2 salinim uretir (tavan bir dongu tutar, bir dongu
// kacar; sig suda ortalama hedef satihin ustunde kalir ve tavan tamamen bosa
// duser). Bu yuzden bu dongunun hedefi yerel hesaplanir ve desired MUTLAK
// yazilir -> islem idempotent, her dongude yeniden degerlendirilir
// (rangefinder/terrain degisimini izler). Birimler metre, U = yukari pozitif.
//
// Cekirdek, komut edilen hedefi disaridan alir: wp_nav'in hedefi yalniz seyir
// bacaklarinda anlamli, daire bacaginda wp_nav bayat/alakasiz bir hedef tutuyor.
// Daire de ayni tavana tabi olmali (bkz. aura_daire_satih_tavani_uygula).
void aura_satih_tavani_cekirdek(AC_PosControl *position_control,
                                float komut_u_m, bool terrain_mi)
{
    constexpr float SATIH_TAVANI_U_M = -0.30f;   // U (yukari, m; 0 = satih)
    float tavan_u_m = SATIH_TAVANI_U_M;
    if (!terrain_mi) {
        // komut edilen hedef daha sigsa (satha cikis WP'si) tavani ona ac
        tavan_u_m = MAX(tavan_u_m, komut_u_m);
    }
    // hedef_u = desired_u + ofset_u ; ofset_u = -(pos_offset_D + pos_terrain_D)
    const float ofset_u_m = -(float)(position_control->get_pos_offset_NED_m().z
                                     + position_control->get_pos_terrain_D_m());
    const float hedef_u_m = position_control->get_pos_desired_U_m() + ofset_u_m;
    if (hedef_u_m > tavan_u_m) {
        position_control->set_pos_desired_U_m(tavan_u_m - ofset_u_m);

        // Konum hedefini kirpmak TEK BASINA yetmiyordu: AC_WPNav yorungeyi konum,
        // HIZ ve IVME olarak birlikte yaziyor (set_pos_vel_accel_NED_m) ve
        // D_update_controller ikisini de hedefe EKLIYOR. Tavan devredeyken P terimi
        // hedefi -0.30 m'de tutarken ileri-besleme hala YUKARI komut ediyor, arac da
        // konum hatasi ileri-beslemeyi yenene kadar tavanin USTUNE tasiyordu.
        // Kirpilmis yorungenin yukari bileseni artik gecerli degil, sifirlanir.
        // (Asagi yonlu ileri-beslemeye dokunulmaz: o zaten tavandan uzaklastiriyor.)
        if (is_negative(position_control->get_vel_desired_NED_ms().z)) {
            position_control->set_vel_desired_D_ms(0.0f);
        }
        if (is_negative(position_control->get_accel_desired_D_mss())) {
            position_control->set_accel_desired_D_mss(0.0f);
        }
    }
}

// Seyir bacaklari: komut edilen hedef wp_nav'dan gelir.
static void aura_satih_tavani_uygula(AC_PosControl *position_control, const AC_WPNav &wp_nav)
{
    aura_satih_tavani_cekirdek(position_control,
                               wp_nav.get_wp_destination_NEU_cm().z * 0.01f,
                               wp_nav.origin_and_destination_are_terrain_alt());
}

// Daire bacagi: komut edilen hedef dairenin merkez irtifasidir.
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

    // Guided overlay'i AUTO'ya her girişte KAPAT. Gerekcesi guided_limit_clear ile
    // ayni: kontrol otoritesini devreden bir ayar, operatorun yaptigi bir mod
    // degisikliginden sagligiyla cikmamali. Plan overlay'i istiyorsa
    // MAV_CMD_AURA_GUIDED_SETUP item'i onu yeniden acar.
    sub.guided_overlay_acik = false;
    sub.guided_overlay_etkin = false;

    // Gorev BURADA baslatilmaz - arac arm olana kadar beklenir (run() icinde).
    //
    // Eskiden burada dogrudan mission.start_or_resume() vardi ve run() de
    // mission.update()'i arm durumundan bagimsiz cagiriyordu. Sonuc: AUTO'ya disarm
    // halde girildigi anda gorev kosmaya basliyor, auto_wp_run'in disarm dali her
    // dongude wp_and_spline_init_m() ile hedefi anlik konuma sifirladigi icin
    // reached_wp_destination() derhal true donuyor ve TUM plan yerinde tukeniyordu.
    // SITL'de olculdu: 5 waypoint'lik plan, arac hic kimildamadan 0.01 saniyede
    // "gorev tamamlandi". Operator sonra arm ettiginde yapilacak is kalmiyor.
    // Bu, aracin normal akisi (once AUTO, sonra ARM - gorev_yukle.py --auto --arm
    // tam olarak bunu yapar) oldugu icin teorik degil.
    //
    // Copter ayni isi waiting_to_start bayragiyla yapiyor (ArduCopter/mode_auto.cpp).
    gorev_arm_bekliyor = true;
    return true;
}

// auto_run - runs the appropriate auto controller
// according to the current auto_mode
void ModeAuto::run()
{
    // Gorev yalnizca arac ARM iken ilerler.
    //
    // Disarm halde mission.update() cagirmak iki ayri sekilde zarar veriyordu:
    //   1) auto_wp_run'in disarm dali hedefi her dongude sifirladigi icin waypoint'ler
    //      aninda "ulasildi" sayiliyor, plan yerinde tukeniyordu (bkz. init()).
    //   2) AURA guard sayaclari (nav_wp_guard_ms, demir guard'i) arm durumuna
    //      bakmadan isliyor, yani sureli item'lar da kendiliginden ilerliyordu.
    // Demir deklansorunun arm kontrolu (commands_logic.cpp, "In AUTO the mission runs
    // regardless of arm state") bu sorunun BELIRTISINE konmus bir yamaydi; sebep
    // buydu ve artik burada kapatiliyor.
    if (motors.armed()) {
        if (gorev_arm_bekliyor) {
            // start/resume the mission (based on MIS_RESTART parameter)
            sub.mission.start_or_resume();
            gorev_arm_bekliyor = false;
        }
        // Guided overlay'i mission.update()'ten ONCE degerlendir: overlay
        // verify_nav_wp'yi bloklar, dolayisiyla karar bu dongunun verify'inden once
        // verilmis olmali.
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

    // BILEREK default: YOK. Tum AutoSubMode degerleri yukarida kapsanmis durumda;
    // default: eklemek -Wswitch'i susturur ve ileride eklenecek bir alt-modun
    // "hicbir cikis uretmeyen durum" olmasi DERLEME ZAMANINDA yakalanamaz hale gelir.
    // Koruma, korumasi gereken seyi gizlemis olurdu.
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
    aura_satih_tavani_uygula(position_control, sub.wp_nav);   // seyirde hedef satihtan >= 0.3 m derinde (satih WP haric)

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
// radius_m : 0 = CIRCLE_RADIUS_M parametresini kullan
// rate_degs: ISARETLI acisal hiz (+ saat yonu, - ters); 0 = CIRCLE_RATE parametresini
//            isaretiyle birlikte kullan
void ModeAuto::auto_circle_movetoedge_start(const Location &circle_center, float radius_m, float rate_degs)
{
    // set circle center
    sub.circle_nav.set_center(circle_center);

    // Yaricap HER ZAMAN yazilir. Eskiden sifir yaricap "dokunma" demekti ve
    // AC_Circle::update_ms ham _radius_m'i kullandigi icin (get_radius_m()'in
    // parametre geri dusumu ORADA calismaz) geride kalan deger neyse o cizilirdi:
    // ilk acilista 0 -> yerinde donme, bir onceki daireden sonra 0 -> o dairenin
    // yaricapi. Ikisi de gorev item'inin soyledigi sey degil. AURA'da 0 = "CIRCLE_RADIUS_M
    // parametresini kullan". get_radius_m() BU IS ICIN YANLIS: geri dusumu yalniz hic
    // calisma-ani yaricap yazilmamisken yapar, yazilmissa o bayat degeri dondurur -
    // yani duzeltmek istedigimiz hatanin ta kendisi. Parametre dogrudan okunur.
    sub.circle_nav.set_radius_m(is_zero(radius_m) ? sub.circle_nav.get_radius_parm_m() : radius_m);

    // Acisal hiz da item basina verilebilir. Eskiden buraya yalnizca bir "ccw" bayragi
    // geliyor, buyukluk her zaman CIRCLE_RATE'ten okunuyordu; yani gorevdeki her daire
    // ayni hizda donmek zorundaydi. Simdi isaretli hiz dogrudan geliyor, 0 ise
    // parametreye (isareti dahil) dusuluyor.
    sub.circle_nav.set_rate_degs(is_zero(rate_degs) ? sub.circle_nav.get_rate_degs() : rate_degs);

    // check our distance from edge of circle
    Vector3f circle_edge_neu_cm;
    float dist_to_edge;
    sub.circle_nav.get_closest_point_on_circle_NEU_cm(circle_edge_neu_cm, dist_to_edge);

    // if more than 3m then fly to edge
    if (dist_to_edge > 300.0f) {
        // Durum, set_wp_destination_loc'tan ONCE yazilir ve boyle kalmali.
        //
        // Bir ara sona tasinmisti ("araya giren bir run() bir onceki hedefe surer"
        // gerekcesiyle) ama o gerekce YANLISTI: mission.update() zaten ModeAuto::run()
        // icinden, kontrolcu switch'inden ONCE cagriliyor, yani do_* ile run() araya
        // giremez. Dahasi tasima gercek bir hata uretiyordu: asagidaki
        // set_wp_destination_loc basarisiz olursa failsafe_terrain_on_event() calisir,
        // o da auto_mode'u Auto_TerrainRecover yapip gorevi durdurur - sondaki atama
        // bunu EZIP terrain failsafe'ini sessizce iptal ederdi.
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
        // Aktif bir ROI varsa ona dokunma: DO_SET_ROI kamerayi bir noktaya kilitler ve
        // daireye giden bacak onu sessizce eziyordu. auto_wp_start ayni korumayi zaten
        // yapiyor, Copter da (mode_auto.cpp circle_movetoedge_start) yapiyor.
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
    // Yaw kipi 1 = "daireye GIRERKEN bakilan yonu koru". Bu, item'in basladigi an
    // degil, dairenin basladigi andir: arada kenara gitme bacagi varsa arac orada
    // LOOK_AT_NEXT_WP ile doner, o yuzden aci burada yakalanir.
    if (sub.daire_yaw_kip == 1) {
        sub.daire_yaw_cd = sub.ahrs.yaw_sensor;
    }

    // init_NEU_cm'e get_rate_degs() vermek DONUS YONUNU CÖPE ATIYORDU.
    // get_rate_degs() CIRCLE_RATE PARAMETRESINI okur; auto_circle_movetoedge_start'in
    // set_rate_degs() ile yazdigi isareti degil (AC_Circle.h: ikisi ayri uyeler,
    // _rate_parm_degs vs _rotation_rate_max_rads). Sonuc: gorevdeki param3<0 (CCW)
    // istegi her seferinde sessizce dusuyor, arac CIRCLE_RATE'in isaretine gore
    // (varsayilan +2 deg/s -> saat yonunde) donuyordu. get_rate_max_degs() az once
    // yazilan isaretli degeri geri verir.
    // NOT: Copter'da da ayni hata var (ArduCopter/mode_auto.cpp circle_start).
    sub.circle_nav.init_NEU_cm(sub.circle_nav.get_center_NEU_cm(),
                               sub.circle_nav.center_is_terrain_alt(),
                               sub.circle_nav.get_rate_max_degs());

    // Durum en sona: init_NEU_cm pozisyon kontrolcusunu sifirliyor.
    sub.auto_mode = Auto_Circle;
}

// auto_circle_run - circle in AUTO flight mode
//      called by auto_run at 100hz or more
// Dairenin dikey ekseni: merkez irtifasina dogru sekillendirilmis tirmanma hizi (m/s, U).
//
// AC_Circle::update_ms terrain OLMAYAN dalda dikey hedefi kendisi kurmaz - hedefi
// "-get_pos_desired_U_m()", yani mevcut hedefin kendisi yapar (AC_Circle.cpp:241) ve
// dikey ekseni yalniz verilen climb_rate ile surer. Climb rate verilmezse (eski hal)
// bu "bulundugun derinligi koru" demektir ve gorev item'inin irtifasi HIC uygulanmaz.
// Bu, arac daire kenarina 3 m'den yakin basladiginda goze batar: kenara gitme bacagi
// atlanir (o bacak derinligi copy_alt_from ile tasiyordu), daire de derinligi hic
// komut etmez - item -12 m dese bile arac bulundugu derinlikte doner.
float ModeAuto::aura_daire_dikey_hiz_ms() const
{
    if (sub.circle_nav.center_is_terrain_alt()) {
        // terrain dalinda kutuphane hedefi zaten input_pos_vel_accel_D_m ile suruyor
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
    // Arac disarm ise: eskiden bu fonksiyonun arm korumasi HIC yoktu; motor cikislari
    // ve dikey kontrolcu disarm halde de suruluyordu. auto_wp_run'daki koruma ornek
    // alindi - AMA oradaki gibi hedefi SIFIRLAMIYORUZ. auto_wp_run disarm dalinda
    // wp_and_spline_init_m() cagirir, bu da hedefi anlik konuma tasir ve
    // reached_wp_destination() aninda true olur; AUTO'ya disarm halde girilirse gorev
    // bu yuzden yerinde kosarak tukeniyor. Daire icin ayni tuzaga dusmemek adina
    // circle_nav durumuna dokunulmaz: acilar birikmez, verify_circle tamamlanmaz.
    if (!motors.armed()) {
        motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        // Sub vehicles do not stabilize roll/pitch/yaw when disarmed
        attitude_control->set_throttle_out(NEUTRAL_THROTTLE, true, g.throttle_filt);
        attitude_control->relax_attitude_controllers();
        return;
    }

    // DIKKAT: burada check_param_change() CAGRILMAZ.
    //
    // ModeCircle onu cagirir ve orada dogrudur - o modda yaricap zaten parametrenin
    // kendisidir. AUTO'da ise yaricabi GOREV ITEM'I belirler ve check_param_change()
    // onu eziyor: _last_radius_param_m yalnizca AC_Circle::init() icinde kuruluyor,
    // AUTO ise init_NEU_cm() yolunu kullandigi icin o alan 0'da kaliyor. Sonucta ilk
    // cagri her zaman "parametre degismis" sanip _radius_m'e CIRCLE_RADIUS_M'i
    // yaziyordu. SITL'de olculdu: item 2.50 m isterken arac 10.00 m (parametre
    // degeri) yaricapinda dondu. Item'i parametre ezmemeli.
    // Motorlari tam aralia al. Bu satir yoktu: daire gorevin ILK nav komutuysa
    // (kenara gitme bacagi da atlanmissa) spool durumu hicbir zaman
    // THROTTLE_UNLIMITED'a cekilmiyor, arac GROUND_IDLE'da kalip hic donmuyordu.
    motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // call circle controller
    sub.failsafe_terrain_set_status(sub.circle_nav.update_ms(aura_daire_dikey_hiz_ms()));

    // Satih tavani daire bacaginda da gecerli (seyirde hedef satihtan >= 0.3 m derinde).
    // Eskiden yalniz seyir bacaklarinda uygulaniyordu; dairenin istemeden satha
    // cikmasina karsi hicbir koruma yoktu.
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

// Dairede burnun nereye bakacagi. MAV_CMD_AURA_CIRCLE item basina secer;
// NAV_LOITER_TURNS her zaman kip 0 kurar, yani eski davranis birebir korunur.
float ModeAuto::aura_daire_yaw_cd()
{
    // Gorev ACIKCA bir yon soylediyse (CONDITION_YAW ya da DO_SET_ROI) o kazanir.
    //
    // Eskiden dairede auto_yaw_mode tamamen yok sayiliyordu ve bu yalniz "ROI calismaz"
    // demek degildi: verify_yaw(), bas aci hedefe 2 derece yaklasana kadar false doner.
    // Daire o hedefi hic uygulamadigi icin kosul asla saglanmiyor, _flags.do_cmd_loaded
    // true'da takiliyor ve o nav bloğundaki SONRAKI TUM do-komutlari (deklansor dahil)
    // hic calismiyordu.
    if (sub.auto_yaw_mode == AUTO_YAW_ROI || sub.auto_yaw_mode == AUTO_YAW_LOOK_AT_HEADING) {
        return get_auto_heading();
    }

    // AC_Circle::update_ms, CIRCLE_OPTIONS bit 1 acikken get_yaw_cd()'ye +/-90 dereceyi
    // KENDISI ekliyor. Kendi tegetini hesaplayan bir cagirici bunu bilmezse acıyı iki
    // kez uygular: kip 3 tam ters yone (180 hata), kip 0 ise 90 derece yana bakar.
    const bool kutuphane_teget = sub.circle_nav.face_direction_of_travel();
    // Isaret AC_Circle ile birebir ayni: pozitif hiz -> -90, negatif hiz -> +90.
    const float taraf_cd = is_negative(sub.circle_nav.get_rate_max_degs()) ? 9000.0f : -9000.0f;

    switch (sub.daire_yaw_kip) {
    case 1:     // basi sabit tut (daireye girerken bakilan yon)
    case 2:     // sabit bas acisi
        return sub.daire_yaw_cd;

    case 3:     // teget: yorunge yonune bak
        return kutuphane_teget ? sub.circle_nav.get_yaw_cd()
                               : wrap_360_cd(sub.circle_nav.get_yaw_cd() + taraf_cd);

    case 0:     // merkeze bak (varsayilan)
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
    aura_satih_tavani_uygula(position_control, sub.wp_nav);   // seyirde hedef satihtan >= 0.3 m derinde (satih WP haric)

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
    // NOT (4.7): get_auto_heading() santiderece dondurur -> _cd varyanti sart,
    // eksiz isim 4.7'de yok, _rad varyanti radyan bekler.
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
// AURA: guided overlay (MAV_CMD_AURA_GUIDED_SETUP) durum makinesi.
//
// ACIK iken ve CANLI bir guided setpoint varken, arac AUTO bacaginin yerine o
// setpoint'e gider; veri susunca bacagina KALDIGI YERDEN devam eder.
//
// Yalniz NAV_WAYPOINT bacaklarinda gecerlidir. Demir, daire, satha cikis ve konum
// sabitleme item'larinin her biri operatorun tam olarak istedigi bir seydir; bir
// setpoint'in onlari ezmesi plani okunamaz hale getirirdi. Waypoint ise sadece
// "su tarafa git" demek - yardimci bilgisayarin inceltmek isteyebilecegi sey tam
// olarak budur.
void ModeAuto::guided_overlay_degerlendir()
{
    const bool uygun = sub.guided_overlay_acik
                       && sub.mission.get_current_nav_id() == MAV_CMD_NAV_WAYPOINT
                       && sub.guided_verisi_taze(sub.guided_overlay_zaman_ms);

    if (uygun && !sub.guided_overlay_etkin) {
        // GIR: once bacagi sakla, sonra guided'a devret.
        //
        // Hedefi wp_nav'dan aliyoruz, gorev item'indan degil: do_nav_wp lat/lon==0
        // olan bir item icin anlik konumu koyuyor, yani item'in kendisi her zaman
        // uculuan noktayi vermiyor. wp_nav'daki cozulmus hedef dogru olan.
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

// Overlay'den cikip kesilen NAV_WAYPOINT bacagini geri ver.
void ModeAuto::guided_overlay_birak()
{
    if (!sub.guided_overlay_etkin) {
        return;
    }
    sub.guided_overlay_etkin = false;

    // Bacagi yeniden kur. auto_wp_start, wp_nav'i aracin BULUNDUGU yerden ayni
    // hedefe dogru yeniden baslatir - guided nereye goturmus olursa olsun.
    // AP_Mission durumuna DOKUNMUYORUZ: set_current_cmd(ayni index) o bacakla
    // paralel kosan do-komut kuyrugunu dusururdu.
    auto_wp_start(sub.guided_overlay_wp_neu_cm);

    // Guard saatini overlay suresi kadar ILERI KAYDIR. Saat overlay boyunca
    // isliyordu; kaydirmazsak uzun bir overlay, geri donuste bacagi hemen
    // "guard time expired" ile atlatirdi.
    const uint32_t gecen = AP_HAL::millis() - sub.guided_overlay_giris_ms;
    sub.nav_wp_start_ms += gecen;

    // Bekleme sayaci da overlay boyunca isliyordu; bacak yeniden basliyor.
    sub.loiter_time = 0;

    gcs().send_text(MAV_SEVERITY_INFO, "GuidedSetup: leg resumed");
}

// NAV_ATTITUDE_TIME (42703): verilen tutumu N saniye koru.
//
// AUV'de bunun karsiligi "kamerayi su yone doğrult ve orada bekle": bir duvara,
// ayaga ya da tekne govdesine bakarken foto/video almak icin. Konum TUTULMAZ -
// yalniz tutum ve dikey hiz komut edilir; akinti varsa arac suruklenir. Nokta
// tutulmasi gerekiyorsa MAV_CMD_AURA_ANCHOR kullanilir.
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

    // Konum kontrolu YOK: yatay iticiler bosta birakilir. Copter'da bu mod
    // egim acisiyla yatay ivme urettigi icin ayrica bir sey gerekmiyor; 6 serbestlik
    // dereceli bir AUV'de yanal/ileri iticiler ayri surulur, sifirlanmazsa bir onceki
    // bacaktan kalan deger donmeye devam ederdi.
    motors.set_lateral(0.0f);
    motors.set_forward(0.0f);

    // Copter'daki lean_angle_max kisiti BURADA UYGULANMAZ: o kisit coklu rotorda
    // egimin ayni zamanda itki vektoru olmasindan gelir. AUV'de roll/pitch gercek
    // tutum komutudur, kirpilmasi item'in soyledigi seyi bozar.
    attitude_control->input_euler_angle_roll_pitch_yaw_cd(nav_attitude_time.roll_deg * 100.0f,
                                                          nav_attitude_time.pitch_deg * 100.0f,
                                                          nav_attitude_time.yaw_deg * 100.0f,
                                                          true);

    const float climb_rate_ms = constrain_float(nav_attitude_time.climb_rate_ms,
                                                -position_control->get_max_speed_down_ms(),
                                                 position_control->get_max_speed_up_ms());
    position_control->D_set_pos_target_from_climb_rate_ms(climb_rate_ms);
    // Satih tavani: bu komut dikey hizi DOGRUDAN suruyor ve komut edilen bir hedef
    // derinlik yok, o yuzden terrain kurtarmasindaki gibi sabit -0.3 m tavan.
    // Bilerek satha cikmak isteyen plan NAV_LAND kullanir.
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

    // Motorlari tam aralia AL. Bu satir yoktu ve spool durumu YAPISKANDIR:
    // AP_Motors::armed() ona dokunmaz. Zemin takipli bir gorevde terrain failsafe
    // tetiklendikten sonra arac disarm edilip tekrar arm edilirse (yukaridaki dal
    // GROUND_IDLE yazmis olur) _spool_desired GROUND_IDLE'da kaliyor; AP_Motors6DOF
    // o durumda tum iticilere 1500 PWM basiyor. Sonuc: arac ARM, AUTO'da, pozisyon
    // kontrolcusu kendini komutta saniyor ama itki SIFIR - serbest suruklenme.
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
    // failsafe_terrain_act() MOD DEGISTIRIR (POSHOLD/ALT_HOLD/SURFACE) ya da disarm
    // eder. return olmadan bu fonksiyon devam edip yeni modun init()'inin az once
    // kurdugu pozisyon/attitude hedeflerini ustune yaziyordu. Yukaridaki
    // "No Rangefinder" dali return'u zaten dogru yapiyor; eksik olan bu ikisiydi.
    return;
#endif

    // exit on failure (timeout)
    if (AP_HAL::millis() > sub.fs_terrain_recover_start_ms + FS_TERRAIN_RECOVER_TIMEOUT_MS) {
        // Recovery has failed, revert to failsafe action
        gcs().send_text(MAV_SEVERITY_CRITICAL, "Terrain failsafe recovery timeout!");
        sub.failsafe_terrain_act();
        return;     // yukaridaki gerekce
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
    // Satih tavani BURADA en cok gerekiyor: OutOfRangeLow dali WPNAV_SPEED_UP ile
    // YUKARI tirmaniyor ve sig suda tabana yakin bir arac tam olarak OutOfRangeLow
    // bildirir. Tavan yoksa kurtarma, FS_TERRAIN_RECOVER_TIMEOUT_MS boyunca tirmanip
    // araci satha cikarir - hem de tavanin var olma sebebi olan (terrain-frame /
    // SURFTRAK) konfigurasyonda. Komut edilen hedef yok, o yuzden sabit -0.3 m.
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
