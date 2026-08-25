#include "Sub.h"

/*
 * mode_smart_rtl.cpp - AURA: gelinen izi geri surerek eve donus
 *
 * AP_SmartRTL, arac ilerledikce 3B kirinti noktalari biriktirir (NED metre, EKF
 * orijinine gore) ve arka planda hem sadelestirme (Ramer-Douglas-Peucker) hem de
 * dongu budama uygular. Bu mod o noktalari TERSTEN tuketir.
 *
 * Duz hat RTL'den farki, tam da bir AUV icin onemli olan sey: buz altinda, enkaz
 * icinde, iskele ayaklari arasinda ya da bir kablo/halat izlerken eve giden duz
 * cizgi ENGELDEN GECER. Gelinen yol ise tanim geregi gecilebilir.
 *
 * Copter'in SmartRTL'i son noktanin 2 m ustune cikip INER; Rover'inki son noktada
 * durur. AUV icin dogru olan Rover'in sekli: burada inis/satha cikis YOKTUR.
 */

// Cok fazla ardisik pop hatasindan sonra pes et (Copter'daki ile ayni butce).
#define SRTL_POP_HATA_ZAMAN_MS  10000

bool ModeSmartRtl::init(bool ignore_checks)
{
    if (!sub.g2.smart_rtl.is_active()) {
        // Kutuphane devre disi: ya SRTL_POINTS=0, ya tampon/konum 15 sn boyunca
        // bozuk kaldi, ya da hic arm olunmadigi icin ev noktasi kaydedilmedi.
        // Sessizce reddetmek yerine sebebi soyle - operator bunu ucus sirasinda
        // ogrenmek zorunda.
        gcs().send_text(MAV_SEVERITY_WARNING, "SmartRTL not active");
        return false;
    }

    // wp_nav'i mevcut duruş noktasindan baslat. Hiz argumani verilmez -> WP_SPD.
    Vector3p durus_ned_m;
    sub.wp_nav.get_wp_stopping_point_NED_m(durus_ned_m);
    sub.wp_nav.wp_and_spline_init_m(0.0f, durus_ned_m);
    if (!sub.wp_nav.set_wp_destination_NED_m(durus_ned_m)) {
        return false;
    }

    tuketilmis_gecerli = false;
    son_basarili_pop_ms = AP_HAL::millis();
    durum = Durum::YOL_TEMIZLIGI;

    // Seyirde burun hat kerterizine donsun; kamera yonu isteniyorsa operator
    // moddan cikip elle surer (SmartRTL bir kacis manevrasidir, foto bacagi degil).
    set_auto_yaw_mode(get_default_auto_yaw_mode(true));

    return true;
}

// 3 Hz zamanlayici gorevi. AP_SmartRTL basligi "3 Hz ya da daha hizli, ARAC HANGI
// MODDA OLURSA OLSUN cagirin" diyor - kirinti yolu her modda birikmeli, yoksa
// operator MANUAL'de dolasip sonra SmartRTL'e bastiginda ortada yol olmaz.
void ModeSmartRtl::save_position()
{
    // SmartRTL'in KENDISI calisirken nokta EKLENMEZ; yoksa geri surdugumuz yol
    // kendini yeniden yazar ve arac hic eve varmaz.
    const bool kaydet = sub.motors.armed() && (sub.control_mode != Mode::Number::SMART_RTL);
    sub.g2.smart_rtl.update(sub.position_ok(), kaydet);
}

void ModeSmartRtl::cikis_temizligi()
{
    // Pop edilmis ama henuz varilmamis noktayi yola geri koy: aksi halde moddan
    // her cikis izde bir delik acar.
    if (tuketilmis_gecerli) {
        sub.g2.smart_rtl.add_point(tuketilmis_nokta_ned_m);
        tuketilmis_gecerli = false;
    }
    // Bekleyen kapsamli temizlik istegini iptal et; birakilirsa arka plan
    // temizligi kalici olarak bloke kalir.
    sub.g2.smart_rtl.cancel_request_for_thorough_cleanup();
}

void ModeSmartRtl::run()
{
    // Arac disarm ise: ArduSub disarm halde tutum stabilize etmez.
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

// Kutuphanenin kapsamli temizligi bitene kadar mevcut noktada bekle.
// request_thorough_cleanup() bir MANDAL: true donene kadar tekrar tekrar cagrilir.
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

    // Varilan noktanin yedegi artik gerekmiyor.
    tuketilmis_gecerli = false;

    Vector3p nokta_ned_m;
    if (!sub.g2.smart_rtl.pop_point(nokta_ned_m)) {
        // Semafor mesgul olabilir - bir sonraki dongude tekrar denenir.
        if (sub.g2.smart_rtl.get_num_points() == 0) {
            // Yol tukendi: eve varildi.
            gcs().send_text(MAV_SEVERITY_INFO, "SmartRTL: path complete");
            durum = Durum::DERINLIKTE_TUT;
        } else if (AP_HAL::millis() - son_basarili_pop_ms > SRTL_POP_HATA_ZAMAN_MS) {
            // Nokta var ama alinamiyor: burada durup tutmak, korlemesine
            // ilerlemekten ya da satha firlamaktan iyidir.
            gcs().send_text(MAV_SEVERITY_ERROR, "SmartRTL: path stalled, holding");
            durum = Durum::DERINLIKTE_TUT;
        }
        return;
    }

    son_basarili_pop_ms = AP_HAL::millis();
    tuketilmis_nokta_ned_m = nokta_ned_m;
    tuketilmis_gecerli = true;

    // DIKKAT: Copter burada hedefe 2 m EKLER (son noktanin ustune inecegi icin).
    // Burada oyle bir sey YOK - kirinti noktasi neredeyse oraya gidilir; satha
    // cikmaya karsi koruma wp_cikislarini_sur() icindeki SERT satih tavanidir.
    if (!sub.wp_nav.set_wp_destination_NED_m(nokta_ned_m)) {
        sub.failsafe_terrain_on_event();
        return;
    }

    // Bir sonraki noktayi da bildir ki S-egrisi her kirintida durmasin.
    Vector3p sonraki_ned_m;
    if (sub.g2.smart_rtl.peek_point(sonraki_ned_m)) {
        sub.wp_nav.set_wp_destination_next_NED_m(sonraki_ned_m);
    }
}

// Son noktada istasyon tut. Hedef degistirilmez; wp_nav zaten oraya kilitli.
void ModeSmartRtl::tut_kos()
{
}

// Ortak cikis yolu - govdesi ModeAuto::auto_wp_run'dan alindi. Copter'in
// input_thrust_vector_heading() cagrisinin ArduSub'da karsiligi yoktur: 6 serbestlik
// dereceli bir araçta yanal/ileri iticiler ayri surulur.
void ModeSmartRtl::wp_cikislarini_sur()
{
    sub.failsafe_terrain_set_status(sub.wp_nav.update_wpnav());

    // SERT satih tavani (-0.30 m), komut edilen hedefe BAKILMADAN.
    //
    // Bu, bu moddaki en onemli AUV'e ozgu karardir. AP_SmartRTL'in ev noktasi
    // set_home() ile ARM ANINDA kaydedilir; ROV'lar satihta arm edildigi icin son
    // kirinti noktasi ~0 m derinliktedir. Tavansiz bir SmartRTL, gorevi tam da
    // AURA'nin yasakladigi seyle bitirirdi: satha dikey cikis ve orada yatay seyir.
    // Tavan aracin 0.3 m'den sigda kalmasini engeller; satha cikmak isteniyorsa
    // operator moddan cikip SURFACE'a gecer.
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
