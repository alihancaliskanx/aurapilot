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

// ANCHOR_MDSW'e gecis reddedilirse ne kadar sonra yeniden denenecek.
#define ANCHOR_MOD_DENEME_ARALIK_MS  5000

// Reddedilme uyarisi bu araliktan sik basilmaz. Deneme 5 saniyede bir surer ama her
// denemede mesaj basmak GCS'i doldururdu: SITL'de 45 saniyelik bir kosuda 53 satir.
#define ANCHOR_UYARI_ARALIK_MS       30000

// Demir noktasi bu kadardan az degistiyse wp_nav'a YENI hedef yazilmaz (bkz. hedefi_uygula).
#define ANCHOR_HEDEF_ESIK_M          0.05f

bool ModeAnchor::init(bool ignore_checks)
{
    if (!sub.position_ok()) {
        // Konum cozumu olmadan nokta tutulamaz. requires_GPS() zaten set_mode
        // seviyesinde reddettiriyor; bu ikinci kapi optflow gibi yollar icin.
        return false;
    }

    // Dikey ve yatay limitleri kur.
    position_control->NE_set_max_speed_accel_cm(sub.wp_nav.get_default_speed_NE_cms(),
                                                sub.wp_nav.get_wp_acceleration_cmss());
    position_control->D_set_max_speed_accel_cm(sub.get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

    Vector3p durus_ned_m;
    sub.wp_nav.get_wp_stopping_point_NED_m(durus_ned_m);
    sub.wp_nav.wp_and_spline_init_m(0.0f, durus_ned_m);

    // Durus noktasi da AYNI kirpmadan gecer.
    //
    // Kirpma bir sure yalniz nokta_kilitle() icindeydi, yani ancak ILK demir verisi
    // mesaji gelince calisiyordu. Arac satihta yuzerken ANCHOR'a alinip hic veri
    // gonderilmezse (modun elle "burada dur" dugmesi olarak kullanilmasi - asagida
    // acikca destekleniyor) hedef, aracin o an yuzdugu ~2 cm derinlikte kaliyordu;
    // satih tavani da komut edilen hedefe acildigi icin tavan su yuzeyine aciliyor ve
    // dikey kontrolcu dalga/barometre gurultusunu kovalamaya basliyordu - yani
    // duzeltmeye calistigimiz davranisin ta kendisi geri geliyordu.
    hedefi_uygula(durus_ned_m, true);

    giris_ms = AP_HAL::millis();
    son_veri_ms = 0;            // henuz demir verisi gelmedi
    son_deneme_ms = 0;
    son_uyari_ms = 0;
    yeniden_kilitle = false;

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

// Hedefi kirp, sakla ve (gerekiyorsa) wp_nav'a yaz.
void ModeAnchor::hedefi_uygula(const Vector3p &istenen_ned_m, bool zorla)
{
    Vector3p yeni_ned_m = istenen_ned_m;

    // Satih karari KOMUT EDILEN derinlige gore verilir, aracin o anki olcumune gore
    // degil: niyete bakariz, dalga gurultusune degil. SURFACE_DEPTH (cm, negatif)
    // yuzey sayilma esigi; D asagi pozitif oldugu icin isaret cevrilir.
    const float satih_d_m = -(float)g.surface_depth * 0.01f;
    satihta_tut = ((float)yeni_ned_m.z < satih_d_m);

    if (satihta_tut) {
        // YUZEY DEMIRI: hedef derinlik SURFACE_DEPTH'e cekilir ve dikey ekseni
        // BURADAN SONRA YALNIZ wp_nav surer.
        //
        // Ilk surumde hedef oldugu gibi birakiliyor, dikey eksen ise
        // kontrolcuyu_sur() icinde AYRICA input_pos_vel_accel_D_m ile suruluyordu.
        // Bu, ModeCircle'da duzeltilen CIFT INTEGRASYONUN aynisiydi. SITL'de
        // olculdu: yuzey demirinde dikey itici degiskenligi PWM_std=36.8 ciktu -
        // derin demirle (37.0) ve POSHOLD ile (34.6) ayni, yani yuzey dali hicbir
        // sey yapmiyordu. Hedefi SURFACE_DEPTH'e cekince arac SURFACE modunun
        // dengelendigi derinligin aynisinda (-0.21 m) durdu ve gorev demirine gore
        // dikey efor 224 -> 30 PWM, degiskenlik 100 -> 34 dustu.
        yeni_ned_m.z = satih_d_m;
    }

    // Hedef gercekten degismediyse wp_nav'a DOKUNMA.
    //
    // set_wp_destination_NED_m, hedefe henuz varilmamisken cagrilirsa AC_WPNav'in
    // S-egrisi bacagini bastan kurar. Demir verisi 2 Hz akarken bu, yaklasma
    // yorungesinin saniyede iki kez sifirlanmasi demek: hiz/ivme ileri-beslemesi
    // her seferinde sifirdan basliyor, arac sarsintili ilerliyor. Ayni nokta
    // tekrar tekrar geldiginde (olagan durum) hicbir sey yapmamak dogru.
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

// Demir verisi girisi. GCS_MAVLink_Sub.cpp, msg 86'yi cozup buraya verir.
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
    const Vector3p istenen_ned_m {
        konum_neu_cm.x * 0.01f,
        konum_neu_cm.y * 0.01f,
       -konum_neu_cm.z * 0.01f,     // NEU(yukari) -> NED(asagi)
    };
    hedefi_uygula(istenen_ned_m, false);
}

void ModeAnchor::run()
{
    // Arac disarm ise: ArduSub disarm halde tutum stabilize etmez.
    if (!sub.motors.armed()) {
        sub.motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        attitude_control->set_throttle_out(NEUTRAL_THROTTLE, true, g.throttle_filt);
        attitude_control->relax_attitude_controllers();

        // Konum kontrolcusunu CANLI TUT. Bu satirlar yoktu ve sonucu agirdi:
        //
        // AC_PosControl'un NE/D denetleyicileri "gecen dongu de cagrildim mi" diye
        // tick sayaciyla bakar (dt_ticks <= 1). Zamanlayici arm durumundan bagimsiz
        // dondugu icin, disarm halde beklenen her dongude sayac buyur. Ilk armli
        // dongude update_wpnav() -> NE_update_controller() bunu gorup kendini
        // yeniden ilklendirir VE INTERNAL_ERROR(flow_of_control) atar; hemen
        // ardindan D_update_controller() ikincisini atar. Bu bayrak KALICIDIR ve
        // AP_Arming onu gorunce "Internal errors 0x..." diyerek arm'i reddeder -
        // ustelik o kontrol ARMING_CHECK ile KAPATILAMAZ. Yani arac bir daha
        // BOOT EDILMEDEN ARM OLMAZ. Modun ilan edilen kullanimi ("once ANCHOR'a
        // al, sonra arm et") tam olarak bu yolu tetikliyordu.
        //
        // ArduSub'un konum/derinlik tutan diger TUM modlari disarm dalinda bunu
        // yapar (ModePoshold: NE_init_controller_stopping_point + D_relax_controller).
        position_control->NE_init_controller_stopping_point();
        position_control->D_relax_controller(sub.motors.get_throttle_hover());

        // Sayaclar ARM ile birlikte baslar. Yoksa disarm halde ANCHOR'da beklemek
        // ANCHOR_TIME'i ve ANCHOR_DTIM'i doldurur ve arm'in ILK dongusunde mod
        // aninda degisirdi.
        giris_ms = AP_HAL::millis();
        son_veri_ms = 0;

        // Yukaridaki init'ler hedefi anlik konuma sifirladi; arm olunca demir
        // noktasi geri yazilmali.
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

    // Mod degistiyse BU DONGUDE baska hicbir sey yapma: aksi halde ANCHOR'in
    // kontrolcusu, yeni modun init()'inin az once kurdugu hedefleri ustune yazar.
    if (cikis_kosulu_denetle()) {
        return;
    }

    kontrolcuyu_sur();
}

// ANCHOR_TIME doldu mu, demir verisi kesildi mi?
// Donus: mod DEGISTI mi (true ise cagiran derhal donmeli).
bool ModeAnchor::cikis_kosulu_denetle()
{
    const Mode::Number hedef_mod = (Mode::Number)g.anchor_mode_switch.get();

    // ANCHOR_MDSW kendisini gosteriyorsa hicbir cikis kosulu yok: sure de,
    // veri kesilmesi de onemsiz, arac son komut edilen noktayi SURESIZ tutar.
    // Kullanicinin istedigi davranis birebir budur.
    if (hedef_mod == Mode::Number::ANCHOR) {
        return false;
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
    //    elle "burada dur" dugmesi olarak da kullanilabilmesi demek.
    const float dtim_s = g.anchor_data_timeout;
    if (is_positive(dtim_s) && son_veri_ms != 0) {
        if (simdi - son_veri_ms > (uint32_t)(dtim_s * 1000.0f)) {
            sebep = "data lost";
        }
    }

    // 2) ANCHOR_TIME doldu mu? 0 = suresiz.
    if (sebep == nullptr && g.anchor_time > 0) {
        if (simdi - giris_ms > (uint32_t)g.anchor_time * 1000UL) {
            sebep = "time expired";
        }
    }

    if (sebep == nullptr) {
        return false;
    }

    // Gecis reddedilirse KALICI olarak vazgecme, geri cekilip yeniden dene.
    // Bir bayrakla "bir kez denedim" demek, tek bir reddedilmis gecisin
    // ANCHOR_TIME'i de ANCHOR_DTIM'i de ucusun geri kalani icin devre disi
    // birakmasi demekti - oysa reddin sebebi (orn. konum cozumunun o an kayip
    // olmasi) genellikle gecicidir.
    if (son_deneme_ms != 0 && simdi - son_deneme_ms < ANCHOR_MOD_DENEME_ARALIK_MS) {
        return false;
    }
    son_deneme_ms = simdi;

    if (!set_mode(hedef_mod, ModeReason::MISSION_END)) {
        // Istenen mod baslamayi reddetti (ornegin konum cozumu olmadan PosHold).
        // Tutmaya DEVAM ederiz: konum kontrolunden dusmek, tutmaya devam etmekten
        // her zaman daha kotudur. SURFMDSW'de de ayni karar verildi.
        // Denemeyi surduruyoruz ama uyariyi seyrekletiyoruz.
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

// Ortak cikis yolu. Govdesi ModeAuto::auto_wp_run'dan alindi; Copter'in
// input_thrust_vector_heading() cagrisinin ArduSub'da karsiligi yoktur.
void ModeAnchor::kontrolcuyu_sur()
{
    sub.failsafe_terrain_set_status(sub.wp_nav.update_wpnav());

    ///////////////////////
    // dikey eksen
    //
    // Dikey ekseni YALNIZ wp_nav surer; burada IKINCI bir cagri YOKTUR.
    // Yuzey demirinde hedefin kendisi hedefi_uygula() icinde SURFACE_DEPTH'e
    // cekilir - gerekcesi orada.
    //
    // Satih tavani her iki durumda da gecerli: komut edilen hedef derinlige
    // acilir, yani bilerek istenen sig bir demir bogulmaz ama istemeden satha
    // cikis engellenir.
    aura_satih_tavani_cekirdek(position_control, -hedef_d_m, false);

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
