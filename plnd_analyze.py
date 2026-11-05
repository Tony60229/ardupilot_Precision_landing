#!/usr/bin/env python3
"""
Precision landing 離線分析（Gazebo 真值）

1. 影像延遲 / PLND_LAG：ap_precland 的 *_raw.npz（姿態、SIM_STATE 真值、每幀相機量測）
   對不同延遲 d 計算「用 t-d 的姿態旋轉的量測」與「t-d 的真值」的水平 RMS，取最小者
2. 飛控估測品質：ArduPilot DataFlash（.BIN）的 PL（AC_PrecLand 目標相對位置估測）
   與 SIM2 / SIM（SITL 真值）比較，依高度分段統計
3. 降落穩定性：觸地垂直 / 水平速度、最後 2 m 最大傾角、最後 1 m 的水平偏移

  python3 plnd_analyze.py --npz logs/plnd_xxx_raw.npz
  python3 plnd_analyze.py --bin run_01/logs/00000001.BIN --marker N E D
"""

from __future__ import annotations

import argparse
import glob
import json
import math
import os
from typing import Optional, Sequence

import numpy as np

IMU_ABOVE_ORIGIN_M = 0.194923       # SITL 位置點（iris_demo 原點，腳架底）→ IMU（base_link）


def rot_zyx_batch(r, p, y):
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    R = np.empty((len(r), 3, 3))
    R[:, 0, 0] = cp * cy; R[:, 0, 1] = sr * sp * cy - cr * sy; R[:, 0, 2] = cr * sp * cy + sr * sy
    R[:, 1, 0] = cp * sy; R[:, 1, 1] = sr * sp * sy + cr * cy; R[:, 1, 2] = cr * sp * sy - sr * cy
    R[:, 2, 0] = -sp;     R[:, 2, 1] = sr * cp;                R[:, 2, 2] = cr * cp
    return R


def _interp(t_src, v_src, t, angle=False):
    if angle:
        u = np.unwrap(v_src)
        return (np.interp(t, t_src, u) + np.pi) % (2 * np.pi) - np.pi
    return np.interp(t, t_src, v_src)


# ---------------------------------------------------------------------------
# 1. 影像延遲
# ---------------------------------------------------------------------------

def latency_from_npz(path: str, max_ms: int = 300, step_ms: int = 5) -> Optional[dict]:
    z = np.load(path)
    att_t, att_v = z["att_t"], z["att_v"]
    tr_t, tr_v = z["truth_t"], z["truth_v"]
    cam_t, cam_p = z["cam_t"], z["cam_p"]
    if len(cam_t) < 50 or len(att_t) < 50 or len(tr_t) < 50:
        return None
    marker = z["marker_rel_home"]
    cam_off = z["cam_offset"]
    C = z["C_bc"]
    p_body = cam_p @ C.T
    curve = []
    for ms in range(0, max_ms + 1, step_ms):
        tc = cam_t - ms / 1000.0
        ok = (tc > max(att_t[0], tr_t[0])) & (tc < min(att_t[-1], tr_t[-1]))
        if ok.sum() < 30:
            continue
        # 排除附近沒有姿態 / 真值資料的樣本（避免跨資料缺口內插）
        ia = np.clip(np.searchsorted(att_t, tc), 1, len(att_t) - 1)
        it = np.clip(np.searchsorted(tr_t, tc), 1, len(tr_t) - 1)
        ok &= (att_t[ia] - att_t[ia - 1] < 0.1) & (tr_t[it] - tr_t[it - 1] < 0.1)
        if ok.sum() < 30:
            continue
        t = tc[ok]
        R = rot_zyx_batch(_interp(att_t, att_v[:, 0], t), _interp(att_t, att_v[:, 1], t),
                          _interp(att_t, att_v[:, 2], t, True))
        meas = np.einsum("nij,nj->ni", R, p_body[ok])
        Rt = rot_zyx_batch(_interp(tr_t, tr_v[:, 3], t), _interp(tr_t, tr_v[:, 4], t),
                           _interp(tr_t, tr_v[:, 5], t, True))
        pos = np.stack([_interp(tr_t, tr_v[:, k], t) for k in range(3)], 1)
        cam = pos + Rt @ cam_off
        truth_rel = marker - cam
        e = np.hypot(meas[:, 0] - truth_rel[:, 0], meas[:, 1] - truth_rel[:, 1])
        e = np.sort(e)[: max(1, int(len(e) * 0.95))]       # 去除最差 5%（偵測剛出現 / 影像停頓）
        curve.append((ms, float(np.sqrt(np.mean(e ** 2))), int(ok.sum())))
    if not curve:
        return None
    best = min(curve, key=lambda c: c[1])
    return {"best_latency_ms": best[0], "rms_at_best_m": best[1], "rms_at_0_m": curve[0][1],
            "curve": curve}


# ---------------------------------------------------------------------------
# 2/3. DataFlash
# ---------------------------------------------------------------------------

def _read_bin(path: str, types: Sequence[str]):
    from pymavlink import mavutil
    mlog = mavutil.mavlink_connection(path)
    out = {t: [] for t in types}
    while True:
        m = mlog.recv_match(type=list(types))
        if m is None:
            break
        out[m.get_type()].append(m.to_dict())
    return out


def analyze_bin(path: str, marker_rel_home: Sequence[float]) -> dict:
    d = _read_bin(path, ["PL", "SIM2", "SIM", "MODE"])
    if not d["PL"] or not d["SIM2"] or not d["SIM"]:
        return {"error": "BIN 缺少 PL / SIM2 / SIM"}
    mk = np.asarray(marker_rel_home, float)
    s2 = d["SIM2"]
    s2_t = np.array([m["TimeUS"] for m in s2]) * 1e-6
    s2_p = np.array([[m["PN"], m["PE"], m["PD"]] for m in s2])
    s2_v = np.array([[m["VN"], m["VE"], m["VD"]] for m in s2])
    s1 = d["SIM"]
    s1_t = np.array([m["TimeUS"] for m in s1]) * 1e-6
    s1_a = np.radians(np.array([[m["Roll"], m["Pitch"], m["Yaw"]] for m in s1]))

    def truth_imu(t):
        R = rot_zyx_batch(_interp(s1_t, s1_a[:, 0], t), _interp(s1_t, s1_a[:, 1], t),
                          _interp(s1_t, s1_a[:, 2], t, True))
        pos = np.stack([_interp(s2_t, s2_p[:, k], t) for k in range(3)], 1)
        return pos + R @ np.array([0.0, 0.0, -IMU_ABOVE_ORIGIN_M])

    res: dict = {}
    # ---- PL 估測誤差 ----
    pl = [m for m in d["PL"] if m["TAcq"]]
    if pl:
        t = np.array([m["TimeUS"] for m in pl]) * 1e-6
        est = np.array([[m["pX"], m["pY"]] for m in pl])
        imu = truth_imu(t)
        rel = mk - imu
        h = rel[:, 2]
        e = np.hypot(est[:, 0] - rel[:, 0], est[:, 1] - rel[:, 1])
        bands = {}
        for lo, hi in ((3, 99), (2, 3), (1, 2), (0.5, 1), (0, 0.5)):
            sel = (h >= lo) & (h < hi)
            if sel.sum() >= 5:
                bands[f"{lo}-{hi}m"] = {"n": int(sel.sum()), "mean_mm": float(e[sel].mean() * 1000),
                                        "p95_mm": float(np.percentile(e[sel], 95) * 1000),
                                        "max_mm": float(e[sel].max() * 1000)}
        res["pl_est_err"] = bands
        res["pl_outliers"] = int(max(m["EKFOutl"] for m in d["PL"]))
    # ---- 降落過程（真值） ----
    land_t = None
    for m in d["MODE"]:
        if int(m.get("ModeNum", m.get("Mode", -1))) == 9:
            land_t = m["TimeUS"] * 1e-6
    if land_t is not None:
        sel = s2_t >= land_t
        t = s2_t[sel]
        imu = truth_imu(t)
        h = mk[2] - imu[:, 2]                          # IMU 相對 tag 平面的高度
        horiz = np.hypot(imu[:, 0] - mk[0], imu[:, 1] - mk[1])
        vel = s2_v[sel]
        # 觸地：IMU 高度第一次低於 0.21 m（停在平台上約 0.195）
        td = np.nonzero(h < IMU_ABOVE_ORIGIN_M + 0.015)[0]
        if len(td):
            i = td[0]
            res["touchdown"] = {"t_after_land_s": float(t[i] - land_t), "vd_mps": float(vel[i, 2]),
                                "vh_mps": float(math.hypot(vel[i, 0], vel[i, 1])),
                                "horiz_err_mm": float(horiz[i] * 1000)}
            fin = horiz[-1]
            res["final_horiz_err_mm"] = float(fin * 1000)
            res["slide_after_touchdown_mm"] = float(abs(fin - horiz[i]) * 1000)
        for hh in (3.0, 2.0, 1.0, 0.5):
            j = np.nonzero(h < hh + IMU_ABOVE_ORIGIN_M)[0]
            if len(j):
                res[f"horiz_err_at_{hh:g}m_mm"] = float(horiz[j[0]] * 1000)
        a_sel = s1_t >= land_t
        ta = s1_t[a_sel]
        ha = mk[2] - truth_imu(ta)[:, 2]
        low = ha < 2.0 + IMU_ABOVE_ORIGIN_M
        if len(td):
            low &= ta < t[td[0]]                    # 只看觸地前（觸地 / 上鎖瞬間的姿態變化不計）
        if low.any():
            tilt = np.degrees(np.hypot(s1_a[a_sel][low, 0], s1_a[a_sel][low, 1]))
            res["max_tilt_below_2m_deg"] = float(tilt.max())
        below1 = h < 1.0 + IMU_ABOVE_ORIGIN_M
        if below1.any() and len(td):
            k = below1 & (np.arange(len(h)) < td[0])
            if k.any():
                res["max_horiz_err_below_1m_mm"] = float(horiz[k].max() * 1000)
    return res


def latest_bin(run_dir: str) -> Optional[str]:
    bins = sorted(glob.glob(os.path.join(run_dir, "logs", "*.BIN")), key=os.path.getmtime)
    return bins[-1] if bins else None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--npz")
    ap.add_argument("--bin")
    ap.add_argument("--marker", type=float, nargs=3, help="平台中心相對 home 的 NED")
    a = ap.parse_args()
    if a.npz:
        r = latency_from_npz(a.npz)
        if r:
            r.pop("curve")
        print(json.dumps(r, indent=1))
    if a.bin:
        print(json.dumps(analyze_bin(a.bin, a.marker), indent=1, ensure_ascii=False))


if __name__ == "__main__":
    main()
