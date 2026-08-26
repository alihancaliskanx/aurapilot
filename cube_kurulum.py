#!/usr/bin/env python3
"""
AURA CubeOrange+ full setup - one command (the script of the 2026-07-15 bench setup).

Flow: (1) flash the firmware with uploader.py (it catches the bootloader even on a
          board stuck in a boot loop)
      (2) FORMAT_VERSION=0 + reboot -> the parameter store is WIPED
      (3) sub-aura.parm (4.8 names, calibrations included) + 4.5.3->4.8 name/unit
          mappings + mission parameters (CAM1_TYPE=5, FS_GCS_ENABLE=0)
      (4) reboot + critical-parameter verification + a quiet-period scan

Usage (from the aurapilot root, with QGC CLOSED):
    python3 cube_kurulum.py [--port /dev/ttyACM0] [--apj fw_cubeorangeplus_4.8dev_9c2803e7/ardusub.apj]
Parm source: master:Tools/autotest/default_params/sub-aura.parm (fetched with git show).
"""
import argparse
import json
import subprocess
import sys
import time
from pathlib import Path

from pymavlink import mavutil

KOK = Path(__file__).resolve().parent
# Firmware options: 1 = what is currently on the board (standard), 2 = shallow-water
# surface ceiling (in AUTO the vertical target is held >= 0.3 m below the surface;
# commit 6621332).
# CAUTION: option 2 has NOT been SITL-tested yet - testing before flashing is advised.
FW_SECENEKLERI = [
    ("standard 4.8-dev (9c2803e7) - what is currently on the board",
     KOK / 'fw_cubeorangeplus_4.8dev_9c2803e7/ardusub.apj'),
    ("shallow-water surface ceiling 4.8-dev (6621332) - AUTO target >= 0.3 m deep [SITL test pending]",
     KOK / 'fw_cubeorangeplus_4.8dev_sigsu_6621332/ardusub.apj'),
]
VARSAYILAN_APJ = FW_SECENEKLERI[0][1]


def firmware_sec(secim):
    """Use --fw N if it was given, otherwise ask interactively (default 1)."""
    if secim is not None:
        return str(FW_SECENEKLERI[secim - 1][1])
    if not sys.stdin.isatty():
        return str(VARSAYILAN_APJ)
    print("Firmware options:")
    for i, (ad, yol) in enumerate(FW_SECENEKLERI, 1):
        print(f"  {i}) {ad}")
    while True:
        s = input(f"Choice [1]: ").strip()
        if not s:
            return str(VARSAYILAN_APJ)
        if s.isdigit() and 1 <= int(s) <= len(FW_SECENEKLERI):
            return str(FW_SECENEKLERI[int(s) - 1][1])
        print("  ! enter a number between 1 and %d" % len(FW_SECENEKLERI))

SKIP_EXACT = {'FORMAT_VERSION', 'SYSID_SW_MREV', 'MIS_TOTAL', 'FENCE_TOTAL'}
SKIP_PRE = ('STAT_', 'SYS_NUM', 'BARO1_GND_PRESS', 'BARO2_GND_PRESS', 'BARO3_GND_PRESS')
# 4.8 equivalents of the old names in the 4.5.3 dump (unit conversions included)
ESLEME = {
    'ARMING_SKIPCHK': 785982,   # the inverted form of ARMING_CHECK 448 (RC+volt+battery only)
    'ATC_ANGLE_MAX': 45,        # ANGLE_MAX 4500 cdeg -> deg
    'ATC_ACC_P_MAX': 1100, 'ATC_ACC_R_MAX': 1100, 'ATC_ACC_Y_MAX': 1100,  # ATC_ACCEL_* /100
    'ATC_RATE_WPY_MAX': 60,     # ATC_SLEW_YAW 6000 /100
    'CIRCLE_RADIUS_M': 10,      # CIRCLE_RADIUS 1000 cm -> m
    'RNGFND1_MIN': 0.5, 'RNGFND1_MAX': 50.0, 'RNGFND1_GNDCLR': 0.1,       # *_CM /100
}
GOREV = {'CAM1_TYPE': 5, 'FS_GCS_ENABLE': 0}   # CLAUDE.md 4 mission requirements
KONTROL = {'ARMING_SKIPCHK': 785982, 'ATC_ANGLE_MAX': 45, 'ATC_ACC_Y_MAX': 1100,
           'ATC_RATE_WPY_MAX': 60, 'RNGFND1_TYPE': 10, 'RNGFND1_ORIENT': 25,
           'RNGFND1_MAX': 50, 'RNGFND1_MIN': 0.5, 'WP_YAW_BEHAVIOR': 4,
           'AHRS_EKF_TYPE': 3, 'EK3_SRC1_POSXY': 6, 'EK3_SRC1_POSZ': 1,
           'EK3_SRC1_YAW': 1, 'VISO_TYPE': 1, 'MOT_1_DIRECTION': -1,
           'CAM1_TYPE': 5, 'FS_GCS_ENABLE': 0}


def bagla(port, timeout=25):
    c = mavutil.mavlink_connection(port, baud=115200, source_system=254)
    if c.wait_heartbeat(timeout=timeout) is None:
        sys.exit(f"ERROR: no heartbeat on {port}")
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
    ap.add_argument('--apj', default=None, help='apj path directly (skips the selection menu)')
    ap.add_argument('--fw', type=int, choices=range(1, len(FW_SECENEKLERI) + 1),
                    help='firmware option (1=standard, 2=shallow-water ceiling)')
    ap.add_argument('--flashsiz', action='store_true', help='skip the flash step (parameters only)')
    a = ap.parse_args()
    if a.apj is None:
        a.apj = firmware_sec(a.fw)

    # fetch the parm set from master
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
    print(f"[1] target parameter set: {len(hedef)}")

    if not a.flashsiz:
        print(f"[2] flash: {a.apj} (waiting for the bootloader; catches it even in a boot loop)")
        r = subprocess.run([sys.executable, '-u', str(KOK / 'Tools/scripts/uploader.py'),
                            '--port', a.port, a.apj], timeout=240)
        if r.returncode != 0:
            sys.exit("ERROR: flash failed")
        time.sleep(10)

    print("[3] wiping the parameter store (FORMAT_VERSION=0 + reboot)...")
    c = bagla(a.port)
    c.mav.param_set_send(c.target_system, c.target_component, b'FORMAT_VERSION', 0,
                         mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
    c.recv_match(type='PARAM_VALUE', blocking=True, timeout=3)
    reboot(c)

    print("[4] loading the parameters (two passes)...")
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
    print(f"    device={len(cihaz)} differing={len(fark)} stubborn={inatci} old-name(expected)={len(yok)}")

    print("[5] reboot + verification...")
    reboot(c)
    c = bagla(a.port)
    hata = 0
    for n, v in KONTROL.items():
        val = p_oku(c, n)
        if val is None or abs(val - v) > max(1e-3, abs(v) * 1e-5):
            hata += 1
            print(f"    WRONG {n} = {val} (expected {v})")
    print(f"    critical verification: {len(KONTROL)-hata}/{len(KONTROL)} OK")
    t0, gorulen = time.time(), set()
    while time.time() - t0 < 10:
        s = c.recv_match(type='STATUSTEXT', blocking=True, timeout=2)
        if s and s.text not in gorulen:
            gorulen.add(s.text)
            print(f"    MSG: {s.text}")
    print("SETUP COMPLETE" if hata == 0 else f"SETUP FINISHED ({hata} bad parameter(s)!)")


if __name__ == '__main__':
    main()
