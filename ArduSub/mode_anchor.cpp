#include "Sub.h"

/*
 * mode_anchor.cpp - AURA: ANCHOR ucus modu
 *
 * Disaridan (GCS / Jetson) SUREKLI beslenen bir noktayi tutar.
 *
 * Gorevdeki demir item'i (MAV_CMD_AURA_ANCHOR 31010) YERINDE DURUYOR ve silinmedi.
 * Ikisi ayni istasyon-tutma fizigini kullanir ama farkli seyler icindir:
 *   - item  : gorev adimi, parametreleri plandan gelir, tamamlanip siradakine gecer
 *   - mod   : canli beslenir, sure ya da veri kesilmesiyle ANCHOR_MDSW'e gecer
 *
 * Foto/deklansor BILEREK YOK: gorev demiri onu icinde tasiyor cunku orada
 * do-komut kuyrugu sirasi garanti degildi (CLAUDE.md 10). Burada operator
 * deklansoru ne zaman isterse kendisi gonderir.
 */

// Demir noktasina "vardik" sayilma yaricapi hesaplanirken kullanilan taban.
// wp_nav'in kendi WP_RADIUS_M'i 3B'dir ve zaten dogru olcu; ayrica bir kapi
// koymuyoruz - bu mod bir gorev adimi degil, tutmanin kendisi.

bool ModeAnchor::init(bool ignore_checks)
{
    if (!sub.position_ok()) {
        // Konum cozumu olmadan nokta tutulamaz. requires_GPS() zaten set_mode
        // seviyesinde reddettiriyor; bu ikinci kapi optflow gibi yollar icin.
        return false;
    }

    // Dikey ve yatay limitleri kur, sonra mevcut durus noktasini kilitle.
    position_control->NE_set_max_speed_accel_cm(sub.wp_nav.get_default_speed_NE_cms(),
                                                sub.wp_nav.get_wp_acceleration_cmss());
    position_control->D_set_max_speed_accel_cm(sub.get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

    Vector3p durus_ned_m;
    sub.wp_nav.get_wp_stopping_point_NED_m(durus_ned_m);
    sub.wp_nav.wp_and_spline_init_m(0.0f, durus_ned_m);
    if (!sub.wp_nav.set_wp_destination_NED_m(durus_ned_m)) {
        return false;
    }

    giris_ms = AP_HAL::millis();
    son_veri_ms = 0;            // henuz demir verisi gelmedi
    mod_degistirildi = false;

    // Baslangicta bulundugumuz derinlik hedeftir; satih karari ona gore.
    hedef_d_m = -(float)sub.current_loc.alt * 0.01f;
    satihta_tut = (hedef_d_m < -(float)g.surface_depth * 0.01f);

    // Yaw: moda girerken bir sey ZORLAMIYORUZ.
    //
    // sub.auto_yaw_mode global bir uyedir ve mod degisiminde sifirlanmaz, yani
    // ANCHOR'a girmeden once verilmis bir CONDITION_YAW ya da DO_SET_ROI burada
    // gecerli kalir - istenen de bu. Demir verisi bir yaw tasiyorsa onu ezer
    // (demir_verisi_al), tasimazsa disaridan verilen yaw yasamaya devam eder.
    // Hicbiri yoksa WP_YAW_BEHAVIOR ne diyorsa o.

    gcs().send_text(MAV_SEVERITY_INFO, "Anchor: holding, %s",
                    g.anchor_time > 0 ? "timed" : "indefinite");
    return true;
}

// Demir verisi girisi. GCS_MAVLink_Sub.cpp, msg 86'yi cozup buraya verir.
// Donus: kabul edildi mi (false = konum donusturulemedi).
bool ModeAnchor::demir_verisi_al(const Vector3f &konum_neu_cm, bool yaw_var, float yaw_cd)
{
    nokta_kilitle(konum_neu_cm);
    son_veri_ms = AP_HAL::millis();

    if (yaw_var) {
        // Demir verisindeki yaw EN YUKSEK onceliklidir: disaridan verilmis bir
        // CONDITION_YAW/ROI varsa onu ezer.
        sub.mode_auto.set_auto_yaw_look_at_heading(yaw_cd * 0.01f, 0.0f, 0, 0);
    }
    // yaw_var false ise auto_yaw_mode'a DOKUNULMAZ: o an neyse (disaridan verilmis
    // bir aci, bir ROI, ya da varsayilan davranis) oyle kalir. Kullanicinin
    // istedigi oncelik sirasi budur: demir verisi > dis yaw komutu > varsayilan.
    return true;
}

void ModeAnchor::nokta_kilitle(const Vector3f &konum_neu_cm)
{
    const Vector3p hedef_ned_m {
        konum_neu_cm.x * 0.01f,
        konum_neu_cm.y * 0.01f,
       -konum_neu_cm.z * 0.01f,     // NEU(yukari) -> NED(asagi)
    };

    if (!sub.wp_nav.set_wp_destination_NED_m(hedef_ned_m)) {
        sub.failsafe_terrain_on_event();
        return;
    }

    hedef_d_m = (float)hedef_ned_m.z;

    // Satih karari KOMUT EDILEN derinlige gore verilir, aracin o anki olcumune
    // gore degil: niyete bakariz, dalga gurultusune degil. SURFACE_DEPTH (cm,
    // negatif) yuzey sayilma esigi; D asagi pozitif oldugu icin isaret cevrilir.
    satihta_tut = (hedef_d_m < -(float)g.surface_depth * 0.01f);
}

void ModeAnchor::run()
{
    // Arac disarm ise: ArduSub disarm halde tutum stabilize etmez.
    //
    // DIKKAT: burada wp_and_spline_init_m() CAGRILMAZ. auto_loiter_run'in disarm
    // dali onu cagiriyor ve hedefi her dongude anlik konuma sifirliyor - yani
    // kilidi yok ediyor. Bir demir modunda bu, disarm/arm arasinda demir
    // noktasinin kaymasi demek olurdu.
    if (!sub.motors.armed()) {
        sub.motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        attitude_control->set_throttle_out(NEUTRAL_THROTTLE, true, g.throttle_filt);
        attitude_control->relax_attitude_controllers();
        return;
    }

    sub.motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    cikis_kosulu_denetle();
    kontrolcuyu_sur();
}

// ANCHOR_TIME doldu mu, demir verisi kesildi mi?
void ModeAnchor::cikis_kosulu_denetle()
{
    const Mode::Number hedef_mod = (Mode::Number)g.anchor_mode_switch.get();

    // ANCHOR_MDSW kendisini gosteriyorsa hicbir cikis kosulu yok: sure de,
    // veri kesilmesi de onemsiz, arac son komut edilen noktayi SURESIZ tutar.
    // Kullanicinin istedigi davranis birebir budur.
    if (hedef_mod == Mode::Number::ANCHOR) {
        return;
    }
    if (mod_degistirildi) {
        return;     // bir kez denedik ve reddedildi; tutmaya devam
    }

    const uint32_t simdi = AP_HAL::millis();
    const char *sebep = nullptr;

    // 1) Demir verisi KESILDI mi? ANCHOR_DTIM = 0 -> zaman asimi kapali.
    //
    //    Zaman asimi ILK VERI GELDIKTEN SONRA islemeye baslar. Once "hic veri
    //    gelmediyse sayac moda giristen baslasin" diye yazilmisti ve SITL'de
    //    hemen patladi: operator moda gecip ilk mesaji gonderene kadar gecen
    //    sure zaman asimini doldurup modu birakiyordu. "Kesilmek" akan bir seyin
    //    durmasidir; hic baslamamis bir akis kesilmis sayilmaz.
    //
    //    Veri hic gelmezse arac giris noktasini tutmaya devam eder - bu, modun
    //    elle "burada dur" dugmesi olarak da kullanilabilmesi demek, ve guvenli
    //    olan da budur. Sureli birakma isteniyorsa ANCHOR_TIME zaten var.
    const float dtim_s = g.anchor_data_timeout;
    if (is_positive(dtim_s) && son_veri_ms != 0) {
        if (simdi - son_veri_ms > (uint32_t)(dtim_s * 1000.0f)) {
            sebep = "data lost";
        }
    }

    // 2) ANCHOR_TIME doldu mu? 0 = suresiz. Sayac MODA GIRISTEN baslar.
    if (sebep == nullptr && g.anchor_time > 0) {
        if (simdi - giris_ms > (uint32_t)g.anchor_time * 1000UL) {
            sebep = "time expired";
        }
    }

    if (sebep == nullptr) {
        return;
    }

    mod_degistirildi = true;
    if (!set_mode(hedef_mod, ModeReason::MISSION_END)) {
        // Istenen mod baslamayi reddetti (ornegin konum cozumu olmadan PosHold).
        // Tutmaya DEVAM ederiz: konum kontrolunden dusmek, tutmaya devam etmekten
        // her zaman daha kotudur. SURFMDSW'de de ayni karar verildi.
        gcs().send_text(MAV_SEVERITY_WARNING, "Anchor: %s, mode %d refused, holding",
                        sebep, (int)g.anchor_mode_switch.get());
    } else {
        gcs().send_text(MAV_SEVERITY_INFO, "Anchor: %s", sebep);
    }
}

// Ortak cikis yolu. Govdesi ModeAuto::auto_wp_run'dan alindi; Copter'in
// input_thrust_vector_heading() cagrisinin ArduSub'da karsiligi yoktur.
void ModeAnchor::kontrolcuyu_sur()
{
    sub.failsafe_terrain_set_status(sub.wp_nav.update_wpnav());

    ///////////////////////
    // dikey eksen
    //
    // YUZEYDE demir atarken derinlik hedefi SURFACE_DEPTH'e KAPALI CEVRIM
    // kilitlenir; SURFACE modundaki tutma dalinin aynisi.
    //
    // Sebep olculdu: onceki testlerde arac demirdeyken dikey iticileri surekli
    // calistiriyordu. Satih tavani, komut edilen hedef sigsa kendini ONA ACAR
    // (mode_auto.cpp) - yani yuzey demirinde tavan ~0'a acilir ve dikey
    // kontrolcu dalga/barometre gurultusunu kovalamaya baslar. Hedefi
    // SURFACE_DEPTH'e sabitlemek bu kovalamayi keser: kontrolcu yalniz aracin
    // yuzerliginin gerektirdigi kadar itki uretir (negatif yuzerlikte sabit
    // hafif yukari, notrde neredeyse sifir).
    if (satihta_tut) {
        float hedef = -(float)g.surface_depth * 0.01f;   // D, asagi pozitif
        float hiz_d_ms = 0.0f;
        position_control->input_pos_vel_accel_D_m(hedef, hiz_d_ms, 0.0f);
    } else {
        // Derinlikte: wp_nav dikey hedefi zaten kurdu. Satih tavani yine de
        // gecerli - komut edilen hedef derinlige acilir, yani bilerek istenen
        // sig bir demir bogulmaz ama istemeden satha cikis engellenir.
        aura_satih_tavani_cekirdek(position_control, -hedef_d_m, false);
    }

    ///////////////////////
    // yatay eksen
    float lateral_out, forward_out;
    sub.translate_wpnav_rp(lateral_out, forward_out);
    sub.motors.set_lateral(lateral_out);
    sub.motors.set_forward(forward_out);

    position_control->D_update_controller();

    ///////////////////////
    // tutum
    float target_roll, target_pitch;
    sub.get_pilot_desired_lean_angles(channel_roll->get_control_in(),
                                      channel_pitch->get_control_in(),
                                      target_roll, target_pitch,
                                      attitude_control->lean_angle_max_cd());

    if (sub.auto_yaw_mode == AUTO_YAW_HOLD) {
        // Otomatik yaw yok: bas pilotun yaw cubugundan surulur.
        float target_yaw_rate = 0;
        if (!sub.failsafe.pilot_input) {
            target_yaw_rate = sub.get_pilot_desired_yaw_rate(channel_yaw->get_control_in());
        }
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_cd(target_roll, target_pitch, target_yaw_rate);
    } else {
        // Demir verisinin yaw'i, ya da disaridan verilmis bir CONDITION_YAW / ROI.
        attitude_control->input_euler_angle_roll_pitch_yaw_cd(target_roll, target_pitch, get_auto_heading(), true);
    }
}
