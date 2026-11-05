#!/usr/bin/env python3
"""
USB 相機實測工具：相機固定對著螢幕，由螢幕顯示測試圖案，量測延遲 / 幀率 / 內參 / 偵測品質

  python3 cam_test.py latency --sweep                     # 螢幕翻轉 ArUco → 相機解出的時間（解析度 / 曝光比較）
  python3 cam_test.py drift                               # 相機 / 支架是否在動
  python3 cam_test.py calib --size 1280x720               # 相機固定：畸變 + 主點（f 準確度有限，見下方說明）
  python3 cam_test.py calib --size 1280x720 --multi       # 相機可動：多姿態完整校正
  python3 cam_test.py tag --profile camera_profiles/uhd4k_1280x720.json     # tag 大小 vs 偵測率 / 抖動
  python3 cam_test.py grid --profile camera_profiles/uhd4k_1280x720.json    # 已知位移 vs 量測（水平定位誤差）
  python3 cam_test.py e2e --profile camera_profiles/uhd4k_1280x720.json     # 螢幕 → ap_precland → 飛控收到

結果存在 logs/camtest/*.json。全螢幕測試中按 Esc 或 q 中止。
"""

from __future__ import annotations

import argparse
import datetime
import json
import os
import random
import re
import subprocess
import threading
import time
from typing import List, Optional, Tuple

import cv2
import numpy as np

from ap_precland import AppConfig, HERE, NestedTagDetector, UsbCamera, find_v4l2, undistort_maps

OUT_DIR = os.path.join(HERE, "logs", "camtest")
WIN = "cam_test"
MARKER_IDS = (10, 23)


def find_camera(name: str = "UHD4K"):
    return find_v4l2(name) or None


def parse_size(s: str) -> Tuple[int, int]:
    w, h = s.lower().split("x")
    return int(w), int(h)


def save_json(name: str, data: dict) -> str:
    os.makedirs(OUT_DIR, exist_ok=True)
    p = os.path.join(OUT_DIR, f"{name}_{datetime.datetime.now():%Y%m%d_%H%M%S}.json")
    with open(p, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=1, ensure_ascii=False)
    return p


class Screen:
    """全螢幕顯示；尺寸取自實際視窗"""

    def __init__(self):
        out = subprocess.run(["xrandr"], capture_output=True, text=True).stdout
        m = re.search(r"current (\d+) x (\d+)", out)
        self.w, self.h = int(m.group(1)), int(m.group(2))
        cv2.namedWindow(WIN, cv2.WINDOW_NORMAL | cv2.WINDOW_FREERATIO | cv2.WINDOW_GUI_NORMAL)
        cv2.setWindowProperty(WIN, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
        self.show(np.zeros((self.h, self.w), np.uint8))
        for _ in range(30):
            cv2.waitKey(10)
        rect = cv2.getWindowImageRect(WIN)
        if (rect[2], rect[3]) != (self.w, self.h):
            print(f"警告：視窗 {rect} 不是螢幕大小 {self.w}x{self.h}，顯示可能被縮放")

    def show(self, img) -> bool:
        cv2.imshow(WIN, img)
        return cv2.waitKey(1) & 0xFF not in (27, ord("q"))

    def pump(self, dt: float) -> bool:
        t_end = time.monotonic() + dt
        while time.monotonic() < t_end:
            if cv2.waitKey(2) & 0xFF in (27, ord("q")):
                return False
        return True

    def close(self):
        cv2.destroyWindow(WIN)
        cv2.waitKey(50)


def aruco_detector():
    d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    return d, cv2.aruco.ArucoDetector(d, cv2.aruco.DetectorParameters())


# ---------------------------------------------------------------------------
# 延遲
# ---------------------------------------------------------------------------

def marker_page(d, mid: int, w: int, h: int) -> np.ndarray:
    side = int(h * 0.8)
    page = np.full((h, w), 255, np.uint8)
    y, x = (h - side) // 2, (w - side) // 2
    page[y:y + side, x:x + side] = cv2.aruco.generateImageMarker(d, mid, side)
    return page


class FrameWatcher(threading.Thread):
    """每幀記錄：取得時間、驅動時間戳、偵測到的 ID、與上一幀的差異量"""

    def __init__(self, cam: UsbCamera):
        super().__init__(daemon=True)
        self.cam = cam
        self.rows: List[Tuple[float, float, int, float]] = []
        self.stop_ev = threading.Event()
        self.scan_ms: List[float] = []

    def run(self):
        _, det = aruco_detector()
        last, prev = -1, None
        while not self.stop_ev.is_set():
            f, t, seq = self.cam.latest()
            if f is None or seq == last:
                time.sleep(0.001)
                continue
            if last >= 0 and seq != last + 1:
                self.rows.append((t, 0.0, -2, 0.0))     # 偵測跟不上而漏掉的幀
            last = seq
            t_buf = self.cam.latest_buf_time()
            t0 = time.perf_counter()
            small = cv2.resize(f, (480, int(480 * f.shape[0] / f.shape[1])), interpolation=cv2.INTER_AREA)
            _, ids, _ = det.detectMarkers(small)
            mid = -1
            if ids is not None:
                hit = [int(i) for i in ids.ravel() if int(i) in MARKER_IDS]
                mid = hit[0] if len(hit) == 1 else -1
            s = small.astype(np.int16)
            diff = 0.0 if prev is None else float(np.mean(np.abs(s - prev)))
            prev = s
            self.scan_ms.append((time.perf_counter() - t0) * 1000)
            self.rows.append((t, t_buf, mid, diff))


def summarize_latency(flips, rows, fps) -> dict:
    t = np.array([r[0] for r in rows])
    ids = np.array([r[2] for r in rows])
    diff = np.array([r[3] for r in rows])
    tb = np.array([r[1] for r in rows])
    noise = float(np.median(diff[diff > 0])) if (diff > 0).any() else 0.0
    dec, chg, dec_buf = [], [], []
    for i, (t_show, mid) in enumerate(flips):
        t_next = flips[i + 1][0] if i + 1 < len(flips) else t_show + 0.5
        sel = np.nonzero((t > t_show) & (t < t_next))[0]
        hit = sel[ids[sel] == mid]
        if len(hit) == 0:
            continue
        dec.append(t[hit[0]] - t_show)
        dec_buf.append(tb[hit[0]] - t_show)
        big = sel[diff[sel] > max(4 * noise, 2.0)]
        if len(big):
            chg.append(t[big[0]] - t_show)

    def stats(a):
        a = np.array(a) * 1000
        return {"n": len(a), "min": float(a.min()), "p5": float(np.percentile(a, 5)), "median": float(np.median(a)),
                "mean": float(a.mean()), "p95": float(np.percentile(a, 95)), "max": float(a.max()),
                "std": float(a.std())} if len(a) else None

    good = t[ids != -2]
    dt = np.diff(good) * 1000
    return {"flips": len(flips), "seen": len(dec), "decoded_ms": stats(dec), "first_change_ms": stats(chg),
            "decoded_buf_ts_ms": stats(dec_buf), "frames": len(rows), "fps": float(1000 / dt.mean()),
            "dt_p99_ms": float(np.percentile(dt, 99)), "dt_max_ms": float(dt.max()),
            "missed_by_watcher": int((ids == -2).sum()), "half_period_ms": 500.0 / fps}


def run_latency(dev, size, fps, flips, exposure, screen: Screen) -> Optional[dict]:
    d, _ = aruco_detector()
    pages = {m: marker_page(d, m, screen.w, screen.h) for m in MARKER_IDS}
    cam = UsbCamera(dev, size[0], size[1], fps, exposure)
    cam.start()
    screen.show(pages[MARKER_IDS[0]])
    if not screen.pump(2.5):                            # 等自動曝光穩定
        cam.stop()
        return None
    w = FrameWatcher(cam)
    w.start()
    screen.pump(0.5)
    rec, show_cost, cur = [], [], 0
    aborted = False
    for _ in range(flips):
        if not screen.pump(random.uniform(0.35, 0.65)):  # 隨機間隔：翻轉時刻與相機幀相位無關
            aborted = True
            break
        cur ^= 1
        t0 = time.monotonic()
        if not screen.show(pages[MARKER_IDS[cur]]):
            aborted = True
            break
        t1 = time.monotonic()
        rec.append((t1, MARKER_IDS[cur]))
        show_cost.append((t1 - t0) * 1000)
    screen.pump(0.4)
    w.stop_ev.set()
    w.join(1.0)
    backlog = cam.backlog_s()
    skipped = cam.skipped
    cam.stop()
    if aborted or len(w.rows) < 30:
        return None
    r = summarize_latency(rec, w.rows, fps)
    r.update({"size": f"{size[0]}x{size[1]}", "req_fps": fps, "exposure": exposure, "show_ms_median": float(np.median(show_cost)),
              "scan_ms_median": float(np.median(w.scan_ms)), "skipped": skipped, "last_backlog_s": backlog})
    return r


def fmt_lat(r: dict) -> str:
    d = r["decoded_ms"]
    c = r["first_change_ms"] or {"median": float("nan")}
    return (f"{r['size']:>10} exp={str(r['exposure']):>5} {r['fps']:5.1f} fps | 看到 {r['seen']:3d}/{r['flips']:3d} | "
            f"解出 ID：min {d['min']:5.1f} 中位 {d['median']:5.1f} 平均 {d['mean']:5.1f} p95 {d['p95']:5.1f} max {d['max']:5.1f} ms | "
            f"畫面變化中位 {c['median']:5.1f} ms | 掉幀 {r['skipped']}")


def cmd_latency(a):
    dev = a.device or find_camera()
    screen = Screen()
    print(f"相機 {dev}，螢幕 {screen.w}x{screen.h}")
    if a.sweep:
        jobs = [(s, e) for s in ((1920, 1080), (1280, 720), (960, 540), (640, 480)) for e in (None,)]
        jobs += [((1280, 720), e) for e in (100, 50, 20)]
    else:
        jobs = [(parse_size(a.size), a.exposure)]
    out = []
    try:
        for rnd in range(a.rounds):
            for size, exp in jobs:
                r = run_latency(dev, size, a.fps, a.flips, exp, screen)
                if r is None:
                    print("中止")
                    return
                r["round"] = rnd
                out.append(r)
                print(fmt_lat(r), flush=True)
    finally:
        screen.close()
    print("結果：", save_json("latency", {"device": dev, "screen": [screen.w, screen.h], "runs": out}))



# ---------------------------------------------------------------------------
# 內參
#
# 相機和螢幕都不動時，所有點都在同一平面、同一姿態：
#   * 畸變與畸變中心：看得很清楚（點密且遍布全畫面）
#   * 焦距：只能靠螢幕相對相機的傾斜（透視）推得。實測單一姿態 f 可差 25%（1018 / 797），主點差 5 px，
#     遠大於統計誤差，所以單姿態的 f 不可信；校正檔由多個姿態聯合求解（fit_per_group）
#   * 在螢幕上畫「變形」的圖案沒有幫助：點實際上仍在同一平面
# 要完整內參請用 calib --multi：螢幕不動，手動把相機擺 10 個以上不同角度（Zhang 法）
# ---------------------------------------------------------------------------

SQ_PX, MK_PX = 80, 60
BOARD_DICT = cv2.aruco.DICT_4X4_250


def make_board(sw: int, sh: int):
    nx, ny = (sw - SQ_PX) // SQ_PX, (sh - SQ_PX) // SQ_PX
    d = cv2.aruco.getPredefinedDictionary(BOARD_DICT)
    board = cv2.aruco.CharucoBoard((nx, ny), float(SQ_PX), float(MK_PX), d)
    img = board.generateImage((nx * SQ_PX, ny * SQ_PX), marginSize=0, borderBits=1)
    return board, img, nx, ny


def board_page(img, sw, sh, ox, oy) -> np.ndarray:
    page = np.full((sh, sw), 255, np.uint8)
    page[oy:oy + img.shape[0], ox:ox + img.shape[1]] = img
    return page


def grab_frames(cam: UsbCamera, screen: Screen, n: int) -> list:
    out, last = [], cam.latest()[2]
    t_end = time.monotonic() + n / cam.fps * 3 + 1.0
    while len(out) < n and time.monotonic() < t_end:
        f, _, seq = cam.latest()
        if seq != last and f is not None:
            last = seq
            out.append(f)
        cv2.waitKey(1)
    return out


def detect_board(det, frames) -> dict:
    """同一畫面多幀平均：{角點 id: (x, y)}（只留穩定的角點）"""
    acc: dict = {}
    for f in frames:
        cc, ci, _, _ = det.detectBoard(f)
        if ci is None:
            continue
        cc = cv2.cornerSubPix(f, cc.astype(np.float32), (5, 5), (-1, -1),
                              (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 40, 0.005))
        for i, p in zip(ci.ravel(), cc.reshape(-1, 2)):
            acc.setdefault(int(i), []).append(p)
    need = max(2, int(len(frames) * 0.6))
    out = {}
    for i, v in acc.items():
        v = np.array(v)
        if len(v) >= need and v.std(0).max() < 0.35:
            out[i] = v.mean(0)
    return out


def collect_points(dev, size, fps, exposure, screen: Screen, shifts: int = 4, frames: int = 6):
    """全螢幕 ChArUco 依序平移 shifts x shifts 次；回傳 螢幕座標 (N,2)、影像座標 (N,2)、組別"""
    board, img, nx, ny = make_board(screen.w, screen.h)
    det = cv2.aruco.CharucoDetector(board)
    ch = board.getChessboardCorners()[:, :2]
    cam = UsbCamera(dev, size[0], size[1], fps, exposure)
    cam.start()
    mx, my = screen.w - img.shape[1], screen.h - img.shape[0]
    obj, pix, grp = [], [], []
    g = 0
    for iy in range(shifts):
        for ix in range(shifts):
            ox, oy = int(mx * ix / max(1, shifts - 1) * 0.98), int(my * iy / max(1, shifts - 1) * 0.98)
            screen.show(board_page(img, screen.w, screen.h, ox, oy))
            if not screen.pump(0.7 if g else 2.5):
                cam.stop()
                raise SystemExit("中止")
            pts = detect_board(det, grab_frames(cam, screen, frames))
            for i, p in pts.items():
                obj.append(ch[i] + (ox, oy))
                pix.append(p)
                grp.append(g)
            print(f"  位置 {g + 1:2d}/{shifts * shifts}：{len(pts)} 點", flush=True)
            g += 1
    cam.stop()
    return np.array(obj, float), np.array(pix, float), np.array(grp)


def _proj(x, obj3, f):
    K = np.array([[f, 0, x[0]], [0, f, x[1]], [0, 0, 1.0]])
    p, _ = cv2.projectPoints(obj3, x[7:10], x[10:13], K, x[2:7])
    return p.reshape(-1, 2)


def fit_fixed_f(f, obj3, pix, x0):
    """x = cx cy k1 k2 p1 p2 k3 r(3) t(3)；obj3 單位 = 千螢幕像素"""
    from scipy.optimize import least_squares
    fun = lambda x: (_proj(x, obj3, f) - pix).ravel()
    r = least_squares(fun, x0, method="lm", x_scale=np.maximum(np.abs(x0), 1e-2), max_nfev=400)
    return r.x, float(np.sqrt(np.mean(r.fun ** 2 * 2))), r


def initial_x(f, w, h, obj3, pix):
    K = np.array([[f, 0, w / 2], [0, f, h / 2], [0, 0, 1.0]])
    _, rv, tv = cv2.solvePnP(obj3, pix, K, None, flags=cv2.SOLVEPNP_ITERATIVE)
    return np.r_[w / 2, h / 2, np.zeros(5), rv.ravel(), tv.ravel()]


def fit_single_plane(obj, pix, w, h) -> dict:
    """掃描焦距 f：每個 f 都重新擬合其餘參數，RMS 曲線的尖銳程度 = f 的可觀測性"""
    obj3 = np.c_[obj / 1000.0, np.zeros(len(obj))]
    curve = []
    for f in np.geomspace(0.45 * w, 2.5 * w, 15):
        x, rms, _ = fit_fixed_f(f, obj3, pix, initial_x(f, w, h, obj3, pix))
        curve.append((float(f), rms, x))
    i = int(np.argmin([c[1] for c in curve]))
    lo, hi = curve[max(0, i - 1)][0], curve[min(len(curve) - 1, i + 1)][0]
    from scipy.optimize import minimize_scalar
    state = {"x": curve[i][2]}

    def cost(f):
        x, rms, _ = fit_fixed_f(f, obj3, pix, state["x"])
        state["x"] = x
        return rms

    b = minimize_scalar(cost, bounds=(lo, hi), method="bounded", options={"xatol": 0.5})
    f = float(b.x)
    x, rms, _ = fit_fixed_f(f, obj3, pix, state["x"])
    res = _proj(x, obj3, f) - pix
    Rm, _ = cv2.Rodrigues(x[7:10])
    tilt = float(np.degrees(np.arccos(np.clip(Rm[2, 2], -1, 1))))
    # 把「RMS 比最小值只差 2%」的 f 視為資料分不出來
    fine = []
    for ff in np.geomspace(max(0.3 * w, f * 0.5), f * 2.0, 25):
        _, r2, _ = fit_fixed_f(ff, obj3, pix, x)
        fine.append((float(ff), r2))
    okf = [c[0] for c in fine if c[1] <= rms * 1.02]
    return {"f": f, "cx": float(x[0]), "cy": float(x[1]), "dist": [float(v) for v in x[2:7]], "rms_px": rms,
            "tilt_deg": tilt, "f_range": [min(okf), max(okf)] if okf else [f, f], "curve": [(c[0], c[1]) for c in curve],
            "p95_px": float(np.percentile(np.hypot(*res.T), 95)), "max_px": float(np.hypot(*res.T).max()),
            "n": len(obj), "x": [float(v) for v in x]}


def rms_without_distortion(obj, pix, w, h, f) -> float:
    """只用單應矩陣（無畸變）擬合的殘差：與上面比較可看出畸變大小"""
    H, _ = cv2.findHomography(obj, pix, 0)
    p = cv2.perspectiveTransform(obj.reshape(-1, 1, 2), H).reshape(-1, 2)
    return float(np.sqrt(np.mean(np.sum((p - pix) ** 2, 1))))


def holdout_rms(obj, pix, grp, w, h, f, x0) -> float:
    """保留一半位置不參與擬合（固定 f），在這些點上算重投影誤差"""
    obj3 = np.c_[obj / 1000.0, np.zeros(len(obj))]
    test = grp % 2 == 1
    x, _, _ = fit_fixed_f(f, obj3[~test], pix[~test], x0)
    e = _proj(x, obj3[test], f) - pix[test]
    return float(np.sqrt(np.mean(np.sum(e ** 2, 1))))



def write_profile(size, fps, exposure, sets, path) -> dict:
    """每個顯示位置都當獨立視角聯合求解；相機姿態沒有變化時 f 不可靠，檔案內會註明"""
    w, h = size
    r = fit_per_group(sets, size)
    spread = r["tilt_range_deg"][1] - r["tilt_range_deg"][0]
    prof = {"device_name": "UHD4K", "width": w, "height": h, "fps": fps, "exposure": exposure,
            "fx": r["fx"], "fy": r["fy"], "cx": r["cx"], "cy": r["cy"], "dist": r["dist"], "rms_px": r["rms_px"],
            "capture_to_host_s": 0.0, "method": "per-view joint", "views": r["views"],
            "tilt_range_deg": r["tilt_range_deg"],
            "f_reliable": spread >= 10.0,
            "f_note": f"視角傾斜差僅 {spread:.1f}°：f 不可靠（單姿態實測可差 25%）。水平定位不受影響，距離 Z 依比例偏差；用 calib --multi 重做"
                      if spread < 10.0 else "",
            "created": f"{datetime.datetime.now():%Y-%m-%d %H:%M}"}
    os.makedirs(os.path.dirname(path), exist_ok=True)
    if os.path.exists(path):
        old = load_prof(path)
        prof["capture_to_host_s"] = old.get("capture_to_host_s", 0.0)
        if "latency_note" in old:
            prof["latency_note"] = old["latency_note"]
    with open(path, "w", encoding="utf-8") as fp:
        json.dump(prof, fp, indent=1, ensure_ascii=False)
    return prof


# ---------------------------------------------------------------------------
# AprilTag：真實降落圖案顯示在螢幕上
# ---------------------------------------------------------------------------

TAG_PNG = os.path.join(HERE, "models", "apriltag_landing_pad", "materials", "textures", "tag48h12_nested.png")


def tag_page(sw, sh, side, cx, cy) -> np.ndarray:
    t = cv2.imread(TAG_PNG, cv2.IMREAD_GRAYSCALE)
    t = cv2.resize(t, (side, side), interpolation=cv2.INTER_NEAREST)
    page = np.full((sh, sw), 255, np.uint8)
    x, y = int(cx - side / 2), int(cy - side / 2)
    page[y:y + side, x:x + side] = t
    return page


def load_prof(path):
    with open(path, encoding="utf-8") as f:
        return json.load(f)


def cmd_tag(a):
    """縮放 tag：哪個大小解得出來、像素抖動、偵測時間"""
    dev = a.device or find_camera()
    size = parse_size(a.size)
    prof = load_prof(a.profile) if a.profile else None
    K = (prof["fx"], prof["fy"], prof["cx"], prof["cy"]) if prof else (1056.0 * size[0] / 1280, 1056.0 * size[0] / 1280,
                                                                       size[0] / 2, size[1] / 2)
    remap = undistort_maps(*K, prof["dist"], *size) if prof else None
    screen = Screen()
    cam = UsbCamera(dev, size[0], size[1], a.fps, a.exposure, remap)
    cam.start()
    cfg = AppConfig(quad_decimate=a.decimate, tag_min_margin=a.min_margin)
    det = NestedTagDetector(cfg)
    rows = []
    try:
        for side in a.sides:
            screen.show(tag_page(screen.w, screen.h, side, screen.w // 2, screen.h // 2))
            if not screen.pump(1.5):
                return
            fr = grab_frames(cam, screen, a.frames)
            res = [det.detect(f, K) for f in fr]
            ok = [m for m in res if m.ok]
            ids = {}
            for m in res:
                for i in m.ids.split():
                    ids[i] = ids.get(i, 0) + 1
            row = {"side_screen_px": side, "frames": len(fr), "ok": len(ok), "ids": ids,
                   "detect_ms": float(np.median([m.detect_ms for m in res]))}
            if ok:
                row.update({"edge_px": float(np.median([m.edge_px for m in ok])), "best_id": ok[-1].tag_id,
                            "uv_std_px": float(np.hypot(np.std([m.u for m in ok]), np.std([m.v for m in ok]))),
                            "margin": float(np.median([m.margin for m in ok]))})
            rows.append(row)
            print(f"tag {side:4d} px：解出 {len(ok):3d}/{len(fr)} ID 次數 {ids} 最大 tag 邊長 {row.get('edge_px', 0):6.1f} px "
                  f"抖動 {row.get('uv_std_px', 0):5.3f} px margin {row.get('margin', 0):5.1f} 偵測 {row['detect_ms']:5.1f} ms", flush=True)
    finally:
        cam.stop()
        screen.close()
    print("結果：", save_json(f"tag_{a.size}_d{a.decimate:g}", {"size": size, "decimate": a.decimate, "rows": rows}))


def kabsch_residual(P, Q):
    """Q ≈ R P + t（剛體，不含縮放）；回傳殘差 (N,3)"""
    Pc, Qc = P - P.mean(0), Q - Q.mean(0)
    U, _, Vt = np.linalg.svd(Pc.T @ Qc)
    D = np.diag([1, 1, np.sign(np.linalg.det(Vt.T @ U.T))])
    R = Vt.T @ D @ U.T
    return Q - (P @ R.T + (Q.mean(0) - R @ P.mean(0)))


def cmd_grid(a):
    """tag 移到螢幕已知位置：量到的 3D 位置與真實位移比較（單位都是螢幕像素，不需知道實體尺寸）"""
    dev = a.device or find_camera()
    size = parse_size(a.size)
    prof = load_prof(a.profile)
    screen = Screen()
    cam = UsbCamera(dev, size[0], size[1], a.fps, a.exposure)      # 原始影像，各種校正條件離線比較
    cam.start()
    side = a.side
    xs = np.linspace(side / 2 + 8, screen.w - side / 2 - 8, a.nx)
    ys = np.linspace(side / 2 + 8, screen.h - side / 2 - 8, a.ny)
    shots = []
    try:
        for rnd in range(a.rounds):
            for y in ys:
                for x in xs:
                    screen.show(tag_page(screen.w, screen.h, side, x, y))
                    if not screen.pump(0.7):
                        return
                    shots.append(((x, y), grab_frames(cam, screen, 4)))
            print(f"第 {rnd + 1} 輪完成", flush=True)
    finally:
        cam.stop()
        screen.close()
    w, h = size
    cfg = AppConfig(quad_decimate=1.0)
    det = NestedTagDetector(cfg)
    cfg.tag_sizes.update({0: 1.0})                                  # 先以 tag 邊長為單位，下面換算成螢幕像素
    f0 = prof["fx"]
    cases = [("無校正 (f 正確, 影像中心, 無畸變)", f0, w / 2, h / 2, None)]
    cases += [(f"校正 f={f:.0f}", f, prof["cx"], prof["cy"], prof["dist"]) for f in (f0 * 0.8, f0, f0 * 1.2)]
    # tag0 偵測邊長（螢幕像素）：由 png 內 ID0 的邊長比例決定
    edge_ratio = a.edge_ratio
    out = {}
    for name, f, cx, cy, dist in cases:
        maps = undistort_maps(f, f, cx, cy, dist, w, h) if dist else None
        P, Q = [], []
        for (x, y), frames in shots:
            ps = []
            for g in frames:
                g2 = cv2.remap(g, maps[0], maps[1], cv2.INTER_LINEAR) if maps else g
                m = det.detect(g2, (f, f, cx, cy))
                if m.ok and m.tag_id == 0:
                    ps.append(m.p_cam * side * edge_ratio)
            if ps:
                P.append((x, y, 0.0))
                Q.append(np.mean(ps, 0))
        P, Q = np.array(P), np.array(Q)
        res = kabsch_residual(P, Q)
        Z = float(Q[:, 2].mean())
        lat = np.hypot(res[:, 0], res[:, 1])
        out[name] = {"n": len(P), "Z_screen_px": Z, "lateral_rms_pct_of_Z": float(np.sqrt(np.mean(lat ** 2)) / Z * 100),
                     "lateral_max_pct_of_Z": float(lat.max() / Z * 100), "z_rms_pct": float(np.sqrt(np.mean(res[:, 2] ** 2)) / Z * 100)}
        o = out[name]
        print(f"{name:32s} 點 {o['n']:3d} Z {Z:7.0f} | 水平 RMS {o['lateral_rms_pct_of_Z']:.3f}% 高度（1 m 高 = {o['lateral_rms_pct_of_Z'] * 10:.1f} mm）"
              f" 最大 {o['lateral_max_pct_of_Z'] * 10:.1f} mm | 高度方向 {o['z_rms_pct']:.2f}%", flush=True)
    print("結果：", save_json(f"grid_{a.size}", out))



def remove_drift(obj, pix, grp, w, h, iters: int = 3) -> Tuple[np.ndarray, np.ndarray]:
    """相機緩慢漂移（支架 / 溫度）：每個顯示位置各估一個影像平移再扣掉，避免它污染主點與傾斜"""
    obj3 = np.c_[obj / 1000.0, np.zeros(len(obj))]
    f = 0.82 * w
    x = initial_x(f, w, h, obj3, pix)
    p = pix.copy()
    off = {}
    for _ in range(iters):
        x, _, _ = fit_fixed_f(f, obj3, p, x)
        e = _proj(x, obj3, f) - p
        for g in np.unique(grp):
            d = e[grp == g].mean(0)
            p[grp == g] += d
            off[int(g)] = off.get(int(g), 0) + d
    return p, np.array([off[k] for k in sorted(off)])


def cmd_drift(a):
    """全螢幕棋盤固定不動，看相機影像隨時間的平移 / 旋轉 / 縮放"""
    dev = a.device or find_camera()
    size = parse_size(a.size)
    screen = Screen()
    board, img, _, _ = make_board(screen.w, screen.h)
    det = cv2.aruco.CharucoDetector(board)
    cam = UsbCamera(dev, size[0], size[1], a.fps, a.exposure)
    cam.start()
    screen.show(board_page(img, screen.w, screen.h, 40, 40))
    rows, ref = [], None
    t0 = time.monotonic()
    try:
        screen.pump(2.0)
        while time.monotonic() - t0 < a.duration:
            pts = detect_board(det, grab_frames(cam, screen, 4))
            if ref is None:
                ref = pts
            k = [i for i in pts if i in ref]
            M, _ = cv2.estimateAffinePartial2D(np.array([ref[i] for i in k], np.float32),
                                               np.array([pts[i] for i in k], np.float32))
            sc = float(np.hypot(M[0, 0], M[1, 0]))
            c = np.array([size[0] / 2, size[1] / 2])
            d = M[:, :2] @ c + M[:, 2] - c
            rows.append((time.monotonic() - t0, float(d[0]), float(d[1]), float(np.degrees(np.arctan2(M[1, 0], M[0, 0]))),
                         (sc - 1) * 100))
            print(f"t={rows[-1][0]:5.0f} s  dx {d[0]:+6.2f} dy {d[1]:+6.2f} px  旋轉 {rows[-1][3]:+.4f}°  縮放 {rows[-1][4]:+.4f}%", flush=True)
            if not screen.pump(a.step):
                break
    finally:
        cam.stop()
        screen.close()
    print("結果：", save_json("drift", {"size": size, "rows": rows}))



def cmd_e2e(a):
    """完整流程：tag 在螢幕左右跳 → ap_precland（real 模式）→ 假飛控收到 LANDING_TARGET 的時間與內容"""
    os.environ.setdefault("MAVLINK20", "1")
    from pymavlink import mavutil
    port = a.port
    fc = mavutil.mavlink_connection(f"tcpin:127.0.0.1:{port}", source_system=1, source_component=1)
    log_dir = os.path.join(OUT_DIR, "e2e_logs")
    cmd = ["python3", os.path.join(HERE, "ap_precland.py"), "--mode", "real", "--conn", f"tcp:127.0.0.1:{port}",
           "--cam-profile", a.profile, "--log-dir", log_dir, "--tag-sizes", "0:0.75,1:0.15,2:0.03"]
    if a.exposure is not None:
        cmd += ["--exposure", str(a.exposure)]
    proc = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    rx: List[Tuple[float, float, float, float, int, int, int, float]] = []
    stop = threading.Event()

    def fc_loop():
        last = 0.0
        while not stop.is_set():
            now = time.monotonic()
            if now - last > 0.5:
                last = now
                fc.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_QUADROTOR, mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
                                      0, 0, 0)
            m = fc.recv_match(type="LANDING_TARGET", blocking=True, timeout=0.05)
            if m is not None:
                rx.append((time.monotonic(), m.x, m.y, m.z, m.frame, m.position_valid, m.type, m.distance))

    th = threading.Thread(target=fc_loop, daemon=True)
    th.start()
    screen = Screen()
    xs = (screen.w // 2 - a.step, screen.w // 2 + a.step)
    pages = [tag_page(screen.w, screen.h, a.side, x, screen.h // 2 + 40) for x in xs]
    flips, cur = [], 0
    try:
        screen.show(pages[0])
        t0 = time.monotonic()
        while not rx and time.monotonic() - t0 < 40:
            if not screen.pump(0.2):
                return
        if not rx:
            print("40 s 內沒有收到 LANDING_TARGET")
            return
        screen.pump(1.0)
        for _ in range(a.flips):
            if not screen.pump(random.uniform(0.35, 0.65)):
                return
            cur ^= 1
            screen.show(pages[cur])
            flips.append((time.monotonic(), cur))
        screen.pump(0.5)
    finally:
        stop.set()
        th.join(1.0)
        screen.close()
        proc.terminate()
        proc.wait(5)
    r = np.array([x[:4] for x in rx])
    t, y = r[:, 0], r[:, 2]
    mid = (np.percentile(y, 10) + np.percentile(y, 90)) / 2
    side_right = y > mid
    lat = []
    for i, (tf, c) in enumerate(flips):
        tn = flips[i + 1][0] if i + 1 < len(flips) else tf + 0.5
        k = np.nonzero((t > tf) & (t < tn) & (side_right == bool(c)))[0]
        if len(k):
            lat.append(t[k[0]] - tf)
    lat = np.array(lat) * 1000
    dt = np.diff(t[t > flips[0][0]])
    lo, hi = y[~side_right], y[side_right]
    out = {"flips": len(flips), "seen": len(lat), "min": float(lat.min()), "median": float(np.median(lat)),
           "mean": float(lat.mean()), "p95": float(np.percentile(lat, 95)), "max": float(lat.max()),
           "msg_rate_hz": float(1 / np.mean(dt)), "msg_gap_max_ms": float(dt.max() * 1000),
           "frame": int(rx[-1][4]), "position_valid": int(rx[-1][5]), "type": int(rx[-1][6]),
           "y_when_left": float(np.median(lo)), "y_when_right": float(np.median(hi)),
           "x_median": float(np.median(r[:, 1])), "z_median": float(np.median(r[:, 3])), "distance": float(rx[-1][7])}
    print(f"螢幕 → 飛控收到：看到 {out['seen']}/{out['flips']} min {out['min']:.1f} 中位 {out['median']:.1f} 平均 {out['mean']:.1f} "
          f"p95 {out['p95']:.1f} max {out['max']:.1f} ms | {out['msg_rate_hz']:.1f} Hz，最大間隔 {out['msg_gap_max_ms']:.0f} ms")
    print(f"訊息：frame {out['frame']} valid {out['position_valid']} type {out['type']} | tag 在左 y={out['y_when_left']:+.3f} "
          f"在右 y={out['y_when_right']:+.3f} x={out['x_median']:+.3f} z={out['z_median']:.3f} distance={out['distance']:.3f} m")
    print("結果：", save_json("e2e", out))



def fit_multi_pose(views, size) -> dict:
    """多姿態（Zhang）：views = [(螢幕座標, 影像座標)]，每組是相機擺在一個不同角度時的全部點"""
    w, h = size
    objs = [np.c_[o / 1000.0, np.zeros(len(o))].astype(np.float32) for o, _ in views]
    pixs = [p.astype(np.float32).reshape(-1, 1, 2) for _, p in views]
    K0 = np.array([[0.8 * w, 0, w / 2], [0, 0.8 * w, h / 2], [0, 0, 1.0]])
    crit = (cv2.TERM_CRITERIA_COUNT + cv2.TERM_CRITERIA_EPS, 3000, 1e-9)    # 預設 30 次迭代在此資料不會收斂
    rms, K, dist, rv, _, = cv2.calibrateCamera(objs, pixs, (w, h), K0, None, flags=cv2.CALIB_USE_INTRINSIC_GUESS,
                                               criteria=crit)
    tilts = [float(np.degrees(np.arccos(np.clip(cv2.Rodrigues(r)[0][2, 2], -1, 1)))) for r in rv]
    return {"fx": float(K[0, 0]), "fy": float(K[1, 1]), "cx": float(K[0, 2]), "cy": float(K[1, 2]),
            "dist": [float(v) for v in dist.ravel()[:5]], "rms_px": float(rms), "tilts_deg": tilts}



def fit_per_group(sets, size, min_pts: int = 80, f0: float = 0.8) -> dict:
    """每個顯示位置當成一個獨立視角（各有自己的姿態），不需假設相機靜止；相機姿態真的有變化時 f 才可觀測
    sets = [(obj, pix, grp), ...]"""
    w, h = size
    objs, pixs = [], []
    for obj, pix, grp in sets:
        for g in np.unique(grp):
            k = grp == g
            if k.sum() >= min_pts:
                objs.append(np.c_[obj[k] / 1000.0, np.zeros(k.sum())].astype(np.float32))
                pixs.append(pix[k].astype(np.float32).reshape(-1, 1, 2))
    K0 = np.array([[f0 * w, 0, w / 2], [0, f0 * w, h / 2], [0, 0, 1.0]])
    flags = cv2.CALIB_USE_INTRINSIC_GUESS | cv2.CALIB_FIX_ASPECT_RATIO
    crit = (cv2.TERM_CRITERIA_COUNT + cv2.TERM_CRITERIA_EPS, 3000, 1e-9)    # 預設 30 次迭代在此資料不會收斂
    rms, K, dist, rv, tv, sd, _, per = cv2.calibrateCameraExtended(objs, pixs, (w, h), K0, None, flags=flags, criteria=crit)
    tilts = [float(np.degrees(np.arccos(np.clip(cv2.Rodrigues(r)[0][2, 2], -1, 1)))) for r in rv]
    return {"fx": float(K[0, 0]), "fy": float(K[1, 1]), "cx": float(K[0, 2]), "cy": float(K[1, 2]),
            "dist": [float(v) for v in dist.ravel()[:5]], "rms_px": float(rms), "views": len(objs),
            "sd_f": float(sd.ravel()[1]), "sd_cx": float(sd.ravel()[2]), "sd_cy": float(sd.ravel()[3]),
            "tilt_range_deg": [min(tilts), max(tilts)]}


def cmd_calib_multi(a):
    """螢幕不動，相機每換一個角度按 Enter；傾斜要有 10~30° 且方向不同，全畫面都要看到足夠的棋盤"""
    dev = a.device or find_camera()
    size = parse_size(a.size)
    views = []
    for v in range(a.views):
        input(f"視角 {v + 1}/{a.views}：把相機轉到新角度並固定後按 Enter ")
        screen = Screen()
        try:
            o, p, _ = collect_points(dev, size, a.fps, a.exposure, screen, shifts=2)
        finally:
            screen.close()
        print(f"  {len(o)} 點")
        if len(o) >= 100:
            views.append((o, p))
    if len(views) < 4:
        raise SystemExit("有效視角不足 4 個")
    r = fit_multi_pose(views, size)
    print(f"f=({r['fx']:.1f},{r['fy']:.1f}) c=({r['cx']:.1f},{r['cy']:.1f}) RMS {r['rms_px']:.3f} px 傾斜 {np.round(r['tilts_deg'], 1).tolist()}")
    path = os.path.join(HERE, "camera_profiles", f"uhd4k_{a.size}.json")
    prof = load_prof(path) if os.path.exists(path) else {}
    prof.update({k: r[k] for k in ("fx", "fy", "cx", "cy", "dist", "rms_px")})
    prof.update({"device_name": "UHD4K", "width": size[0], "height": size[1], "fps": a.fps, "exposure": a.exposure,
                 "method": "multi-pose Zhang", "f_note": ""})
    with open(path, "w", encoding="utf-8") as f:
        json.dump(prof, f, indent=1, ensure_ascii=False)
    print("已更新", path)


def cmd_calib(a):
    if a.multi:
        return cmd_calib_multi(a)
    dev = a.device or find_camera()
    size = parse_size(a.size)
    w, h = size
    screen = Screen()
    runs = []
    try:
        for r in range(a.rounds):
            print(f"第 {r + 1}/{a.rounds} 輪 {a.size}", flush=True)
            runs.append(collect_points(dev, size, a.fps, a.exposure, screen, a.shifts))
    finally:
        screen.close()
    os.makedirs(OUT_DIR, exist_ok=True)
    fits = []
    raw_runs = list(runs)                       # 聯合求解用原始點（姿態各自估，不需扣漂移）
    for r, (obj, pix, grp) in enumerate(runs):
        np.savez(os.path.join(OUT_DIR, f"calib_pts_{a.size}_{datetime.datetime.now():%H%M%S}_r{r}.npz"), obj=obj, pix=pix, grp=grp)
        pix, off = remove_drift(obj, pix, grp, w, h)
        runs[r] = (obj, pix, grp)
        ft = fit_single_plane(obj, pix, w, h)
        ft["drift_px"] = float(np.ptp(off, axis=0).max())
        ft["no_dist_rms_px"] = rms_without_distortion(obj, pix, w, h, ft["f"])
        ft["holdout_rms_px"] = holdout_rms(obj, pix, grp, w, h, ft["f"], np.array(ft["x"]))
        fits.append(ft)
        print(f"第 {r + 1} 輪：f {ft['f']:.1f}（分不出的範圍 {ft['f_range'][0]:.0f}~{ft['f_range'][1]:.0f}）"
              f" c=({ft['cx']:.1f},{ft['cy']:.1f}) 傾斜 {ft['tilt_deg']:.2f}° 點數 {ft['n']} RMS {ft['rms_px']:.3f} px"
              f"（無畸變 {ft['no_dist_rms_px']:.3f}，保留組 {ft['holdout_rms_px']:.3f}）"
              f" k={np.round(ft['dist'], 4).tolist()}", flush=True)
    print("結果：", save_json(f"calib_{a.size}", {"size": size, "fits": fits}))
    path = os.path.join(HERE, "camera_profiles", f"uhd4k_{a.size}.json")
    prof = write_profile(size, a.fps, a.exposure, raw_runs, path)
    print(f"校正檔 {path}：f {prof['fx']:.1f} c=({prof['cx']:.1f},{prof['cy']:.1f}) RMS {prof['rms_px']:.3f} px")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--device", help="預設依名稱尋找 UHD4K")
    sub = ap.add_subparsers(dest="cmd", required=True)
    p = sub.add_parser("latency")
    p.add_argument("--size", default="1280x720")
    p.add_argument("--fps", type=int, default=30)
    p.add_argument("--flips", type=int, default=100)
    p.add_argument("--rounds", type=int, default=1)
    p.add_argument("--exposure", type=float)
    p.add_argument("--sweep", action="store_true")
    p.set_defaults(func=cmd_latency)
    p = sub.add_parser("calib")
    p.add_argument("--size", default="1280x720")
    p.add_argument("--fps", type=int, default=30)
    p.add_argument("--exposure", type=float)
    p.add_argument("--rounds", type=int, default=3)
    p.add_argument("--shifts", type=int, default=4)
    p.add_argument("--multi", action="store_true", help="相機可移動時用：多姿態完整校正")
    p.add_argument("--views", type=int, default=8)
    p.set_defaults(func=cmd_calib)
    p = sub.add_parser("drift")
    p.add_argument("--size", default="1280x720")
    p.add_argument("--fps", type=int, default=30)
    p.add_argument("--exposure", type=float)
    p.add_argument("--duration", type=float, default=180)
    p.add_argument("--step", type=float, default=10)
    p.set_defaults(func=cmd_drift)
    p = sub.add_parser("tag")
    p.add_argument("--size", default="1280x720")
    p.add_argument("--fps", type=int, default=30)
    p.add_argument("--exposure", type=float)
    p.add_argument("--profile")
    p.add_argument("--decimate", type=float, default=2.0)
    p.add_argument("--min-margin", type=float, default=30.0)
    p.add_argument("--frames", type=int, default=30)
    p.add_argument("--sides", type=int, nargs="+", default=[960, 700, 500, 350, 250, 180, 130, 100, 75, 55])
    p.set_defaults(func=cmd_tag)
    p = sub.add_parser("e2e")
    p.add_argument("--profile", required=True)
    p.add_argument("--exposure", type=float)
    p.add_argument("--flips", type=int, default=100)
    p.add_argument("--port", type=int, default=5790)
    p.add_argument("--side", type=int, default=400)
    p.add_argument("--step", type=int, default=300, help="tag 左右跳動距離（螢幕像素）")
    p.set_defaults(func=cmd_e2e)
    p = sub.add_parser("grid")
    p.add_argument("--size", default="1280x720")
    p.add_argument("--fps", type=int, default=30)
    p.add_argument("--exposure", type=float)
    p.add_argument("--profile", required=True)
    p.add_argument("--side", type=int, default=300)
    p.add_argument("--nx", type=int, default=7)
    p.add_argument("--ny", type=int, default=3)
    p.add_argument("--rounds", type=int, default=2)
    p.add_argument("--edge-ratio", type=float, default=0.5, help="ID0 偵測邊長 / 圖檔全寬")
    p.set_defaults(func=cmd_grid)
    a = ap.parse_args()
    a.func(a)


if __name__ == "__main__":
    main()
