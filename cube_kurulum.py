#!/usr/bin/env python3
"""
AURA CubeOrange+ tam kurulum — tek komut (2026-07-15 bench kurulumunun scripti).

Akis: (1) uploader.py ile firmware flash (boot-loop'taki kartta da bootloader'i yakalar)
      (2) FORMAT_VERSION=0 + reboot -> parametre deposu SIFIRLANIR
      (3) sub-aura.parm (4.8 adlari, kalibrasyonlar dahil) + 4.5.3->4.8 isim/birim
          eslemeleri + gorev parametreleri (CAM1_TYPE=5, FS_GCS_ENABLE=0)
      (4) reboot + kritik parametre dogrulamasi + sessizlik taramasi

Kullanim (aurapilot kokunden, QGC KAPALI olmali):
    python3 cube_kurulum.py [--port /dev/ttyACM0] [--apj fw_cubeorangeplus_4.8dev_9c2803e7/ardusub.apj]
Parm kaynagi: master:Tools/autotest/default_params/sub-aura.parm (git show ile cekilir).
"""
import argparse
import json
import subprocess
import sys
import time
from pathlib import Path

from pymavlink import mavutil

KOK = Path(__file__).resolve().parent
# Firmware secenekleri: 1 = karttaki son durum (standart), 2 = sig-su satih tavanli
# (AUTO'da dikey hedef satihtan >= 0.3 m derinlikte tutulur; commit 6621332).
# DIKKAT: secenek 2'nin SITL testi henuz yapilmadi — araca yuklemeden once test onerilir.
FW_SECENEKLERI = [
    ("standart 4.8-dev (9c2803e7) — karttaki son durum",
     KOK / 'fw_cubeorangeplus_4.8dev_9c2803e7/ardusub.apj'),
    ("sig-su satih tavanli 4.8-dev (6621332) — AUTO hedef >= 0.3 m derinlik [SITL testi bekliyor]",
     KOK / 'fw_cubeorangeplus_4.8dev_sigsu_6621332/ardusub.apj'),
]
VARSAYILAN_APJ = FW_SECENEKLERI[0][1]


def firmware_sec(secim):
    """--fw N verildiyse onu, verilmediyse interaktif sor (varsayilan 1)."""
    if secim is not None:
        return str(FW_SECENEKLERI[secim - 1][1])
    if not sys.stdin.isatty():
        return str(VARSAYILAN_APJ)
    print("Firmware secenekleri:")
    for i, (ad, yol) in enumerate(FW_SECENEKLERI, 1):
        print(f"  {i}) {ad}")
    while True:
        s = input(f"Secim [1]: ").strip()
        if not s:
            return str(VARSAYILAN_APJ)
        if s.isdigit() and 1 <= int(s) <= len(FW_SECENEKLERI):
            return str(FW_SECENEKLERI[int(s) - 1][1])
        print("  ! 1-%d arasi bir sayi gir" % len(FW_SECENEKLERI))

SKIP_EXACT = {'FORMAT_VERSION', 'SYSID_SW_MREV', 'MIS_TOTAL', 'FENCE_TOTAL'}
SKIP_PRE = ('STAT_', 'SYS_NUM', 'BARO1_GND_PRESS', 'BARO2_GND_PRESS', 'BARO3_GND_PRESS')
# 4.5.3 dokumundeki eski adlarin 4.8 karsiliklari (birim donusumleri dahil)
ESLEME = {
    'ARMING_SKIPCHK': 785982,   # ARMING_CHECK 448 (yalniz RC+volt+batarya) tersine cevrilmis hali
    'ATC_ANGLE_MAX': 45,        # ANGLE_MAX 4500 cdeg -> deg
    'ATC_ACC_P_MAX': 1100, 'ATC_ACC_R_MAX': 1100, 'ATC_ACC_Y_MAX': 1100,  # ATC_ACCEL_* /100
    'ATC_RATE_WPY_MAX': 60,     # ATC_SLEW_YAW 6000 /100
    'CIRCLE_RADIUS_M': 10,      # CIRCLE_RADIUS 1000 cm -> m
    'RNGFND1_MIN': 0.5, 'RNGFND1_MAX': 50.0, 'RNGFND1_GNDCLR': 0.1,       # *_CM /100
}
GOREV = {'CAM1_TYPE': 5, 'FS_GCS_ENABLE': 0}   # CLAUDE.md §4 gorev gereksinimleri
KONTROL = {'ARMING_SKIPCHK': 785982, 'ATC_ANGLE_MAX': 45, 'ATC_ACC_Y_MAX': 1100,
           'ATC_RATE_WPY_MAX': 60, 'RNGFND1_TYPE': 10, 'RNGFND1_ORIENT': 25,
           'RNGFND1_MAX': 50, 'RNGFND1_MIN': 0.5, 'WP_YAW_BEHAVIOR': 4,
           'AHRS_EKF_TYPE': 3, 'EK3_SRC1_POSXY': 6, 'EK3_SRC1_POSZ': 1,
           'EK3_SRC1_YAW': 1, 'VISO_TYPE': 1, 'MOT_1_DIRECTION': -1,
           'CAM1_TYPE': 5, 'FS_GCS_ENABLE': 0}


def bagla(port, timeout=25):
    c = mavutil.mavlink_connection(port, baud=115200, source_system=254)
    if c.wait_heartbeat(timeout=timeout) is None:
        sys.exit(f"HATA: {port} heartbeat yok")
    return c


def p_yaz(c, ad, deger, deneme=3):
    for _ in range(deneme):
        c.mav.param_set_send(c.target_system, c.target_component, ad.encode(),
                             float(deger), mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        m = c.recv_match(type='PARAM_VALUE', blocking=True, timeout=1.5)
        if m and m.param_id == ad and abs(m.param_value - float(deger)) < max(1e-3, abs(deger) * 1e-5):
            return True
    return False


def p_oku(c, ad):
    c.mav.param_request_read_send(c.target_system, c.target_component, ad.encode(), -1)
    m = c.recv_match(type='PARAM_VALUE', blocking=True, timeout=3)
    return m.param_value if m and m.param_id == ad else None


def reboot(c, bekle=14):
    c.mav.command_long_send(c.target_system, c.target_component,
                            mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, 0, 1, 0, 0, 0, 0, 0, 0)
    c.close()
    time.sleep(bekle)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--port', default='/dev/ttyACM0')
    ap.add_argument('--apj', default=None, help='dogrudan apj yolu (secim menusunu atlar)')
    ap.add_argument('--fw', type=int, choices=range(1, len(FW_SECENEKLERI) + 1),
                    help='firmware secenegi (1=standart, 2=sig-su tavanli)')
    ap.add_argument('--flashsiz', action='store_true', help='flash adimini atla (yalniz parametre)')
    a = ap.parse_args()
    if a.apj is None:
        a.apj = firmware_sec(a.fw)

    # parm setini master'dan cek
    parm = subprocess.run(['git', '-C', str(KOK), 'show',
                           'master:Tools/autotest/default_params/sub-aura.parm'],
                          capture_output=True, text=True, check=True).stdout
    hedef = {}
    for ln in parm.splitlines():
        ln = ln.strip()
        if not ln or ln.startswith('#'):
            continue
        p = ln.split()
        if len(p) < 2 or p[0] in SKIP_EXACT or p[0].startswith(SKIP_PRE):
            continue
        hedef[p[0]] = float(p[1])
    hedef.update(ESLEME)
    hedef.update(GOREV)
    print(f"[1] hedef parametre seti: {len(hedef)}")

    if not a.flashsiz:
        print(f"[2] flash: {a.apj} (bootloader bekleniyor; kart boot-loop'taysa da yakalar)")
        r = subprocess.run([sys.executable, '-u', str(KOK / 'Tools/scripts/uploader.py'),
                            '--port', a.port, a.apj], timeout=240)
        if r.returncode != 0:
            sys.exit("HATA: flash basarisiz")
        time.sleep(10)

    print("[3] parametre deposu sifirlaniyor (FORMAT_VERSION=0 + reboot)...")
    c = bagla(a.port)
    c.mav.param_set_send(c.target_system, c.target_component, b'FORMAT_VERSION', 0,
                         mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
    c.recv_match(type='PARAM_VALUE', blocking=True, timeout=3)
    reboot(c)

    print("[4] parametreler yukleniyor (iki gecis)...")
    c = bagla(a.port)
    for n, v in hedef.items():
        c.mav.param_set_send(c.target_system, c.target_component, n.encode(), v,
                             mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        c.recv_match(type='PARAM_VALUE', blocking=True, timeout=0.7)
    c.mav.param_request_list_send(c.target_system, c.target_component)
    cihaz, son = {}, time.time()
    while time.time() - son < 3:
        m = c.recv_match(type='PARAM_VALUE', blocking=True, timeout=3)
        if m:
            cihaz[m.param_id] = m.param_value
            son = time.time()
    fark = {n: v for n, v in hedef.items()
            if n in cihaz and abs(cihaz[n] - v) > max(1e-5, abs(v) * 1e-5)}
    inatci = [n for n, v in fark.items() if not p_yaz(c, n, v)]
    yok = [n for n in hedef if n not in cihaz]
    print(f"    cihaz={len(cihaz)} fark={len(fark)} inatci={inatci} eski-ad(beklenen)={len(yok)}")

    print("[5] reboot + dogrulama...")
    reboot(c)
    c = bagla(a.port)
    hata = 0
    for n, v in KONTROL.items():
        val = p_oku(c, n)
        if val is None or abs(val - v) > max(1e-3, abs(v) * 1e-5):
            hata += 1
            print(f"    BOZUK {n} = {val} (beklenen {v})")
    print(f"    kritik dogrulama: {len(KONTROL)-hata}/{len(KONTROL)} OK")
    t0, gorulen = time.time(), set()
    while time.time() - t0 < 10:
        s = c.recv_match(type='STATUSTEXT', blocking=True, timeout=2)
        if s and s.text not in gorulen:
            gorulen.add(s.text)
            print(f"    MSG: {s.text}")
    print("KURULUM TAMAM" if hata == 0 else f"KURULUM BITTI ({hata} sorunlu parametre!)")


if __name__ == '__main__':
    main()
