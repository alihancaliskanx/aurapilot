# AuraTorpedo — Yeni Araç Sınıfı Yol Haritası

Torpido tipi UUV: tek pervane + X-Kuyruk (4 fin, ±45° çapraz). Yeni araç sınıfı
olarak (ArduSub/ArduPlane/ArduCopter gibi) `AuraTorpedo/` dizininde yaşar.
Taban: **ArduSub kopyası** (derinlik/EKF/failsafe/joystick + aura yamaları hazır gelir).

- Branch: `torpedo-4.5.3-aura` (taban `sub-4.5.3-aura`)
- ALTIN KURAL: ArduSub'a dokunulmaz. Ortak kütüphane değişiklikleri yalnız
  `APM_BUILD_AuraTorpedo` koşuluyla eklenir; her ortak dosya değişikliğinden
  sonra ArduSub SITL derlenip mevcut sim ile duman testi yapılır (Faz 5).

## Faz 0 — Hazırlık
- ✅ Branch açıldı: `torpedo-4.5.3-aura`
- ✅ Taban kararı: ArduSub kopyası; isim: **AuraTorpedo** (binary: `auratorpedo`)

## Faz 1 — İskelet: derlenebilir AuraTorpedo — ✅ TAMAM (2026-07-24)
- ✅ 1.1 `ArduSub/` → `AuraTorpedo/` kopyalandı; wscript: `auratorpedo` (grup: bin+auratorpedo);
  kök wscript build komut listesine 'auratorpedo' eklendi (dizin recursion otomatikti)
- ✅ 1.2 `APM_BUILD_AuraTorpedo 14` (AP_Vehicle_Type.h) — wscript ap_vehicle=dizin adı
  olduğundan tanım kendiliğinden devreye girdi
- ✅ 1.3 Sub→Torpedo adlandırma (GCS_Torpedo, AP_Arming_Torpedo, Torpedo.h/.cpp,
  AuraTorpedo.cpp; ReleaseNotes hariç; Makefile.waf hedefi elle düzeltildi)
- ✅ 1.4 16 kütüphane dosyasında `APM_BUILD_TYPE(APM_BUILD_ArduSub)` →
  `APM_BUILD_SUB_OR_TORPEDO` (yeni makro; Sub için birebir aynı önişlemci sonucu).
  NOT: AP_Scripting bindings.desc (Sub singleton) torpidoya TAŞINMADI — Lua'da
  sub: API'si torpidoda yok, gerekirse sonra
- ✅ 1.5 vehicleinfo.py: AuraTorpedo (vectored + vectored_6dof frame'leri, sub parm'ları)
- ✅ 1.6 MAV_TYPE_SUBMARINE doğrulandı (boot testinde HEARTBEAT type=12)
- 🎯 KİLOMETRE TAŞI GEÇİLDİ: `waf auratorpedo` temiz derleniyor (1242 görev, sıfır
  hata); standalone boot (SIM_Submarine, --model vectored) → "ArduPilot Ready",
  MANUAL mod; `waf sub` regresyonu temiz (16 sn, artımlı)
- Derleme tuzakları (gelecek için):
  - Konteynerde derle: `docker run --user $(id -u) -e HOME=/tmp -v aurapilot:/ap
    auraauv/aurasimauv:latest bash -c 'cd /ap && ./waf auratorpedo'`
  - İmaja future+pexpect kuruldu (commit; yedek tag :pre-pydeps) — mavgen/dronecan ister
  - Tools/AP_Periph/wscript: em/pexpect import'ları build() içine taşındı (modül-yükleme
    aninda sitl build'ini kırıyordu)
  - Tools/ardupilotwaf/ap_library.py `_vehicle_macros`: APM_BUILD_SUB_OR_TORPEDO eklendi
    (yoksa makroyu kullanan kütüphaneler araç-bağımsız sınıflanıp derleme kırılıyor)
  - build/c4che'de bayat CubeOrangePlus varyantı (host configure kalıntısı) build'i
    kilitliyordu → silindi; CubeOrange gerekince yeniden configure edilir

## Faz 2 — Aktüatör katmanı: X-Kuyruk + tek pervane — ✅ ÇEKİRDEK TAMAM (2026-07-24)
- ✅ 2.1+2.2 KARAR DEĞİŞİKLİĞİ: ayrı AP_MotorsTorpedo/k_xtail SRV fonksiyonları yerine
  AP_Motors6DOF'a yeni frame: **SUB_FRAME_TORPEDO_XTAIL (FRAME_CONFIG=8, enum sonu)**.
  M1=pervane (forward), M2-5=fin (M2 sancak-üst, M3 sancak-alt, M4 iskele-alt,
  M5 iskele-üst — Gazebo fin0..3 sırası). Daha az kod, QGC motor testi bedava,
  ArduSub'a salt-ekleme. AuraTorpedo FRAME_CONFIG default=8.
  DOĞRULANDI (SITL, RC override → SERVO_OUTPUT_RAW):
  nötr=hepsi 1500; forward→yalnız M1 1900; pitch→(+,+,-,-); yaw→(+,-,-,+);
  roll→(+,+,+,+) — Gazebo'da doğrulanan X matrisiyle birebir.
- ⬜ 2.3 QGC motor testi GUI doğrulaması (motor fonksiyonları kullanıldığından
  çalışması beklenir; Gazebo entegrasyonunda test edilecek)
- ⬜ 2.4 Parametre temizliği: XTAIL karışım gain paramları; anlamsız MOT_ ayıklama
- NOT: standalone SIM_Submarine X-kuyruğu SİMÜLE ETMEZ (bilinçli, risk notlarında);
  torpido_sitl_dene.sh bu yüzden FRAME_CONFIG=1'e sabitler. X-kuyruk fiziği
  yalnız Gazebo yolunda (Faz 4.2 ArduPilotPlugin kanal haritası SIRADAKİ İŞ).

## Faz 3 — Kontrol mimarisi (torpidoya özgü asıl iş)
- ⬜ 3.1 Attitude: AC_AttitudeControl_Sub rate PID çıkışları → set_roll/pitch/yaw
  → X-Tail mixer. İlk sürüm sabit gain; sonra hızla gain ölçekleme
  (fin otoritesi ∝ hız²; EKF hızı ya da pervane komutu vekil)
- ✅ 3.2 TAMAM (2026-07-24): ALT_HOLD torpido hali — derinlik hatası → pitch hedefi
  (P=12°/m, D=9°/(m/s), sınır ±30°, 0.8 m/s altında otorite lineer kısılır;
  update_z_controller çağrılmaz). Satıh tavanı (≥0.3 m) döngüde. Pilot throttle
  çubuğu = dal/çık hızı. GAZEBO'DA DOĞRULANDI: 2.3-2.6 m/s seyirde −11 m dalış
  (pitch −29°), +26° ile çıkış, hedef derinlik tutma. Kazanç parametreleştirme kaldı.
  + MANUAL/STABILIZE lateral→pitch remap (QGC toggle'sız pitch; lateral çıkışı 0)
- 🟡 3.3 Modlar: MANUAL ✓, STABILIZE ✓, ALT_HOLD ✓, **AUTO ✓ (2026-07-24)**;
  SURFACE/GUIDED torpido uyarlaması bekliyor
- ✅ 3.4 KISMEN — AUTO torpido "pure pursuit" (mode_auto.cpp): translate_wpnav_rp
  (hover-varsayımlı, dur-kalk kilitleniyordu) yerine: ileri = mesafe-orantılı
  (15 m'den iniş) × burun-hedef hizası (alt sınır 0.25 — DÖNÜŞ İÇİN YOL ŞART,
  hizasız+hızsız kalıcı kilitlenme!), 10 m'den uzakta taban sürüş 0.35;
  yaw = hedefe-bearing (track-bearing WP kaçırınca geri döndürmüyordu);
  derinlik = ortak pitch döngüsü (update_z_controller ölü). Loiter/terrain-recover
  de lateral'siz + pitch döngülü. GAZEBO'DA GÖREV UÇTU: 3 WP (dal −5 ✓ kabul,
  −10 ✓ kabul, −3'e çıkış ✓), WPNAV_RADIUS 500, WP_YAW_BEHAVIOR 4.
  KALAN İNCELİKLER: son-WP kabulü sınırda; yaklaşmada hız dalgalanması
  (hiza/mesafe etkileşimi); AUTO'ya geçiş EKF konumu İSTER (test protokolü:
  "using GPS" bekle + geçişi tekrar dene); circle bacağı eski yolda
- ⬜ 3.4 Holonomiklik: torpidoda lateral yok — `motors.cpp` translate_* yardımcıları
  ve WPNav çevirisi uyarlanır (lateral talep → yaw'a yönlendir);
  sıfır-bacak yaw yaması (aura) korunur
- ⬜ 3.5 Failsafe gözden geçirme: "dur" davranışı torpidoda "itkiyi kes →
  pozitif yüzerlikle satha süzül" olmalı; leak/batarya failsafe'leri aynen

## Faz 4 — Gazebo simülasyonu
- ✅ 4.0 Sim deposu branch'i: `auv_simulation` → `torpedo`; boş dünya
  `aura_worlds/deniz_torpido.sdf` (deniz_sitl altyapısı, araç/görev öğesi yok,
  600x600 m alan)
- ✅ 4.1 Model: `auv_simulation/aura_models/aura_torpedo/` — ECA A9 referanslı
  (mesh+katsayılar uuvsimulator/eca_a9, Apache-2.0): tek pervane
  (gz-sim-thruster force mode), 4 X-fin (LiftDrag + JointPositionController).
  Boş dünyada TEST EDİLDİ: 40 N → 2.2 m/s, pitch/yaw/roll tepkileri doğrulandı.
  Detay + tuzaklar: `auv_simulation/TORPIDO.md`; elle sürüş: `docker/torpido_surus.sh`
- ✅ 4.2 TAMAM (2026-07-24): `aura_models/aura_torpedo_sitl/` (IMU FLU→FRD +
  ArduPilotPlugin: ch0 pervane COMMAND→cmd_thrust ±50 N, ch1-4 fin COMMAND→cmd_pos
  ±0.35 rad; işaret çözümü FIRMWARE matrisinde — pitch/roll sütunları FRD
  konvansiyonuna çevrildi). deniz_torpido.sdf artık SITL varyantını içerir
  (ad "aura_torpedo" → elle sürüş de çalışır). UÇTAN UCA DOĞRULANDI:
  arm → 42.9 m seyir, STABILIZE'da pitch −0.6°/roll 0.1° düz uçuş, 2.51 m/s
  (gz yer-gerçeği). MANUAL feedback'siz savruk (normal; his ayarı Faz 3).
- ⬜ 4.3 Hidrodinamik: torpido formuna göre katsayılar (eksenel düşük / lateral
  yüksek direnç), added mass, pozitif yüzerlik + CB üstte (statik stabilite)
- ✅ 4.4 TAMAM (2026-07-24): `torpedo-aura.parm` (FRAME_CONFIG 8, SERVO1-5=Motor1-5,
  yumuşak ATC kazançları) + vehicleinfo.py `torpedo-aura` frame'i (model JSON)
- ✅ 4.5a kullan.sh (torpedo branch): torpidoya özel sade sürüm — AUV haritaları/
  SITL/QGC çıkarıldı; tek komutla konteyner + boş deniz + torpido, elle sürüş
  ipuçları; AURA_HEADLESS=1 ile başsız. Uçtan uca test edildi (2.09 m/s @ 40 N)
- ✅ 4.5b KISMEN (2026-07-24): kullan.sh adım [6/6] = AuraTorpedo SITL + QGC
  köprüsü (torpido_sitl_dene.sh, varsayılan JSON/Gazebo modu; "standalone" argümanı
  Gazebo'suz mod). Tether/durdur aura_torpedo_fw'yi de kapsar. Tam akış test
  edildi: kullan.sh → Gazebo+SITL+köprü, MAVLink type=12 akıyor.
  KALAN: `simulasyon` branch kullan.sh'ı ile birleşme (WORLDS menüsüne dönüş)
- ⬜ 4.6 Uçtan uca: arm → MANUAL sürüş → ALT_HOLD derinlik tutma → küçük AUTO görevi
- 🎯 Kilometre taşı: QGC'den torpido sim'de görev uçurulabiliyor

## Faz 5 — Koruma ve test
- ⬜ 5.1 Her ortak-kütüphane commit'inden sonra: `waf sub` derle + vectored_6dof-aura
  sim duman testi (arm, ALT_HOLD, kısa AUTO) — ArduSub davranışı değişmemeli
- ⬜ 5.2 AuraTorpedo duman testleri betikleştir (arm/mod/derinlik)
- ⬜ 5.3 Dokümantasyon: bu dosyada işaretler güncel tutulur;
  auv_simulation/TORPIDO.md (sim kullanım notları)

## Bilinen risk/karar notları
- QGC, firmware'i ArduSub sanacak (1.6 kararı) — mod adları/arayüz Sub gibi
  görünür; torpidoya özel modlar QGC'de "Unknown" listelenebilir, kabul edildi.
- Fin otoritesi hıza bağlı: düşük hızda ALT_HOLD derinlik tutamaz (fiziksel
  gerçek); mod tasarımında minimum yol hızı kavramı gerekecek.
- SIM_Submarine'e fin fiziği YAZILMAYACAK — fizik Gazebo'dan (JSON backend)
  geliyor; salt-SITL çalıştırma torpidoda desteklenmeyecek.
