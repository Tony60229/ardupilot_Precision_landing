#!/usr/bin/env python3
"""
ArduPilot 視覺導引 Precision Landing — companion 觀測節點（參考 PX4 vte_land.py 架構）

流程：
  影像 → AprilTag tagCustom48h12 nested（ID 0/1/2）偵測 → 解出「相機→平台中心」3D 向量
  → MAVLink LANDING_TARGET（frame = BODY_FRD，position_valid = 1，x/y/z + distance）
  → ArduPilot AC_PrecLand（PLND_TYPE = 1）以 Kalman filter 估測 → LAND / RTL 時精準降落

  * 送 3D 向量 + distance：飛控直接用視覺量到的距離，不需要測距儀，平台高度（0.4 m）不影響
  * BODY_FRD：飛控用「現在 - PLND_LAG」時的姿態旋轉到 NED，所以 PLND_LAG 要等於
    拍攝 → 飛控收到 的總延遲（sim 可用 --sim-auto 結束時的延遲估測結果設定）
  * --frame local：改送 LOCAL_FRD（程式先用拍攝當下的 roll/pitch 轉成水平座標，飛控只再轉 yaw）

模式（--mode）：
  sim   Gazebo Classic：UDP 5600 影像、直連 SITL SERIAL1 tcp:127.0.0.1:5762（或 --conn）、Gazebo 真值比對
        --sim-auto：自動 解鎖 → 起飛 → 飛到平台附近（可加入定位誤差）→ LAND → 量測降落誤差
  real  實機：USB 相機（--cam-profile）/ RTSP 影像、serial 飛控；只送 LANDING_TARGET，不切換模式、不解鎖

紀錄：logs/plnd_YYYYmmdd_HHMMSS.csv，每幀一列（同一時間點）：
  影像ID、偵測結果、送出的 LANDING_TARGET、位置/目標位置、速度/目標速度、姿態/目標姿態、
  遙控輸入、（sim）真值與量測誤差。結束時另存 .xlsx（若有 pandas + openpyxl）

使用：
  python3 ap_precland.py --mode sim                       # 只觀測 + 送目標（在 MAVProxy 下 mode LAND）
  python3 ap_precland.py --mode sim --sim-auto            # 自動起飛 + 精準降落 + 誤差報告
  python3 ap_precland.py --mode real --conn /dev/ttyACM0 --cam-profile camera_profiles/uhd4k_1280x720.json
  python3 ap_precland.py --mode real --conn /dev/ttyACM0 --rtsp rtsp://...
"""

from __future__ import annotations

import argparse
import bisect
import csv
import datetime
import json
import logging
import math
import os
import random
import signal
import subprocess
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass, field, replace
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

os.environ.setdefault("MAVLINK20", "1")
from pymavlink import mavutil  # noqa: E402

log = logging.getLogger("plnd")

HERE = os.path.dirname(os.path.abspath(__file__))
SIM_PARAM_FILE = os.path.join(HERE, "params", "precland_mavlink.parm")

# ---------------------------------------------------------------------------
# Config
# ---------------------------------------------------------------------------


@dataclass
class AppConfig:
    mode: str = "real"

    # 影像
    camera_source: str = "rtsp"            # udp | rtsp | device
    rtsp_url: str = ""
    device: str = ""                       # /dev/videoN；空白 = 依 cam_name 尋找
    cam_name: str = "UHD4K"
    cam_width: int = 1280
    cam_height: int = 720
    cam_fps: int = 30
    cam_exposure: Optional[float] = None   # 手動曝光（單位 100 µs）；None = 自動
    cam_dist: Tuple[float, ...] = ()       # 畸變係數（k1 k2 p1 p2 k3）；有值時偵測前先去畸變
    udp_port: int = 5600
    video_latency_s: float = 0.0           # 拍攝時間 = 取得時間 - 此值（sim 由真值估測）
    max_video_backlog_s: float = 0.15      # 影像年齡（積壓 + 處理）超過此值就停止送 LANDING_TARGET（不用舊影像導引）

    # 相機內參（未給 fx 時以 FOV 計算；影像尺寸以實際影像為準）
    fov_horizontal_deg: float = 80.0
    fx: float = 0.0
    fy: float = 0.0
    cx: float = 0.0
    cy: float = 0.0
    # 相機光軸座標（x 右、y 下、z 前）→ 機體 FRD；預設：下視、影像上方 = 機頭
    cam_to_body: Tuple[Tuple[float, ...], ...] = ((0, -1, 0), (1, 0, 0), (0, 0, 1))

    # AprilTag（tag_size = 偵測角點邊長 m；三個 tag 同中心）
    tag_family: str = "tagCustom48h12"
    tag_sizes: Dict[int, float] = field(default_factory=lambda: {0: 0.75, 1: 0.15, 2: 0.03})
    tag_min_margin: float = 30.0
    tag_min_px: float = 12.0               # 角點邊長小於此像素數不採用
    quad_decimate: float = 2.0
    nthreads: int = 2
    hamming_bits: int = 1                  # 48h12 用 2 bits 需 ~4.5 GB 記憶體；1 bit 約 150 MB

    # MAVLink
    conn: str = "/dev/ttyACM0"
    baud: int = 921600
    source_system: int = 255
    source_component: int = 191            # MAV_COMP_ID_ONBOARD_COMPUTER
    target_system: int = 1
    target_component: int = 1
    send_frame: str = "body"               # body = MAV_FRAME_BODY_FRD；local = MAV_FRAME_LOCAL_FRD
    max_send_hz: float = 40.0

    # Log / UI
    show_preview: bool = False
    log_dir: str = os.path.join(HERE, "logs")
    rc_channels_logged: int = 8
    run_timeout_s: float = 0.0             # >0：超過即結束

    # Sim
    sim_truth: bool = False
    sim_auto: bool = False
    sim_vehicle_model: str = "iris_demo"
    sim_pad_model: str = "landing_pad"
    sim_pad_top_z: float = 0.4025          # tag 平面相對平台模型原點高度
    # 相機相對 SITL 位置點（= Gazebo iris_demo 模型原點，在腳架底部）的 FRD 位置：
    # base_link（IMU）在原點上方 0.195 m，相機在 IMU 下方 0.03 m → z = -0.165
    sim_cam_offset_body: Tuple[float, float, float] = (0.0, 0.0, -0.165)
    sim_takeoff_alt_m: float = 6.0
    sim_approach_err_m: float = 1.0        # 飛到「平台真值 + 隨機水平誤差」模擬 GPS 導航誤差
    sim_approach_yaw_deg: Optional[float] = None   # None：隨機
    sim_hover_s: float = 3.0
    sim_land_timeout_s: float = 150.0
    sim_seed: Optional[int] = None
    result_json: str = ""


PRESETS = {
    "real": dict(camera_source="rtsp"),
    "sim": dict(camera_source="udp", udp_port=5600, fov_horizontal_deg=math.degrees(1.3963),
                conn="tcp:127.0.0.1:5762", sim_truth=True, video_latency_s=0.0),
}

MAV_FRAME_BODY_FRD = 12
MAV_FRAME_LOCAL_FRD = 20
LANDING_TARGET_TYPE_VISION_FIDUCIAL = 3
COPTER_MODES = {0: "STABILIZE", 2: "ALT_HOLD", 3: "AUTO", 4: "GUIDED", 5: "LOITER", 6: "RTL", 9: "LAND",
                16: "POSHOLD", 17: "BRAKE"}

# ---------------------------------------------------------------------------
# 數學
# ---------------------------------------------------------------------------


def rot_zyx(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """機體 → NED"""
    cr, sr, cp, sp, cy, sy = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw)
    return np.array([
        [cp * cy, sr * sp * cy - cr * sy, cr * sp * cy + sr * sy],
        [cp * sy, sr * sp * sy + cr * cy, cr * sp * sy - sr * cy],
        [-sp, sr * cp, cr * cp],
    ])


def wrap_pi(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def nan3():
    return [math.nan] * 3


class TimedBuffer:
    """(host monotonic 時間, 數值) 緩衝；at(t) 線性內插，angle_idx 欄位以角度方式內插"""

    def __init__(self, maxlen: int = 4000, angle_idx: Sequence[int] = ()):
        self.t: deque = deque(maxlen=maxlen)
        self.v: deque = deque(maxlen=maxlen)
        self.angle_idx = tuple(angle_idx)
        self.lock = threading.Lock()

    def append(self, t: float, values) -> None:
        with self.lock:
            if self.t and t < self.t[-1]:
                return
            self.t.append(t)
            self.v.append(list(values))

    def latest(self):
        with self.lock:
            return (self.t[-1], self.v[-1]) if self.t else (None, None)

    def at(self, t: float, max_gap: float = 0.25):
        with self.lock:
            n = len(self.t)
            if n == 0:
                return None
            ts = self.t
            if t >= ts[-1]:
                return list(self.v[-1]) if t - ts[-1] <= max_gap else None
            if t <= ts[0]:
                return list(self.v[0]) if ts[0] - t <= max_gap else None
            i = n - 1                       # 查詢時間通常接近最新資料：由尾端往回找
            while i > 0 and ts[i - 1] > t:
                i -= 1
                if n - i > 2000:            # 太舊：改用二分搜尋
                    tl = list(ts)
                    i = bisect.bisect_right(tl, t)
                    break
            t0, t1 = ts[i - 1], ts[i]
            v0, v1 = self.v[i - 1], self.v[i]
        if t1 - t0 > max_gap:
            return None
        a = (t - t0) / (t1 - t0) if t1 > t0 else 0.0
        out = []
        for k, (x0, x1) in enumerate(zip(v0, v1)):
            if k in self.angle_idx:
                out.append(wrap_pi(x0 + a * wrap_pi(x1 - x0)))
            else:
                out.append(x0 + a * (x1 - x0))
        return out

    def snapshot(self):
        with self.lock:
            return np.array(self.t), np.array(self.v)


class BootClock:
    """飛控 time_boot_ms → host monotonic（取最小傳輸延遲的 offset）"""

    def __init__(self):
        self.samples: deque = deque(maxlen=500)
        self.offset: Optional[float] = None

    def update(self, boot_s: float, host_s: float) -> None:
        if self.samples and boot_s + 1.0 < self.samples[-1][0]:     # 飛控重開機
            self.samples.clear()
        self.samples.append((boot_s, host_s - boot_s))
        self.offset = min(o for _, o in self.samples)

    def to_host(self, boot_s: float) -> Optional[float]:
        return None if self.offset is None else boot_s + self.offset


# ---------------------------------------------------------------------------
# 影像
# ---------------------------------------------------------------------------


class GstUdpCamera:
    """Gazebo GstCameraPlugin：H.264 RTP over UDP。用 python-gi（此環境 OpenCV 未編 GStreamer）"""

    def __init__(self, port: int):
        self.port = port
        self._lock = threading.Lock()
        self._frame = None
        self._t = 0.0
        self._seq = 0
        self._running = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._reset_backlog()

    def start(self) -> None:
        self._running.set()
        self._thread = threading.Thread(target=self._loop, name="cam", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._running.clear()
        if self._thread:
            self._thread.join(timeout=3.0)

    def latest(self):
        with self._lock:
            return self._frame, self._t, self._seq

    def _reset_backlog(self) -> None:
        self._rtp_last = None
        self._rtp_ext = 0
        self._off_min = None
        self._backlog = 0.0

    def backlog_s(self) -> float:
        """影像積壓（s）：封包到達與 RTP 拍攝時間戳的差，減去本次連線最小值。
        編碼器跟不上時 Gazebo 端的佇列會一直累積，曾觀察到 10 s 的舊影像。"""
        return self._backlog

    def _rtp_probe(self, pad, info):
        from gi.repository import Gst
        try:
            hdr = info.get_buffer().extract_dup(4, 4)      # RTP 標頭 byte 4~7 = 90 kHz 時間戳
            ts = int.from_bytes(hdr, "big")
            if self._rtp_last is not None:
                self._rtp_ext += (ts - self._rtp_last + (1 << 31)) % (1 << 32) - (1 << 31)
            self._rtp_last = ts
            off = time.monotonic() - self._rtp_ext / 90000.0
            if self._off_min is None or off < self._off_min:
                self._off_min = off
            self._backlog = off - self._off_min
        except Exception as e:  # noqa: BLE001
            log.debug("rtp probe: %s", e)
        return Gst.PadProbeReturn.OK

    def _loop(self) -> None:
        import gi
        gi.require_version("Gst", "1.0")
        from gi.repository import Gst
        Gst.init(None)
        desc = (f"udpsrc port={self.port} ! application/x-rtp,media=video,encoding-name=H264 ! "
                "rtph264depay name=depay ! h264parse ! avdec_h264 ! videoconvert ! video/x-raw,format=GRAY8 ! "
                "appsink name=sink sync=false max-buffers=1 drop=true")
        while self._running.is_set():
            pipe = Gst.parse_launch(desc)
            sink = pipe.get_by_name("sink")
            self._reset_backlog()
            pipe.get_by_name("depay").get_static_pad("sink").add_probe(Gst.PadProbeType.BUFFER, self._rtp_probe)
            pipe.set_state(Gst.State.PLAYING)
            log.info("UDP 影像接收中（port %d）", self.port)
            last_ok = time.monotonic()
            while self._running.is_set():
                smp = sink.emit("try-pull-sample", Gst.SECOND // 5)
                now = time.monotonic()
                if smp is None:
                    if now - last_ok > 5.0:
                        log.warning("UDP 影像 5 s 沒有資料，重建 pipeline")
                        break
                    continue
                buf = smp.get_buffer()
                st = smp.get_caps().get_structure(0)
                w, h = st.get_value("width"), st.get_value("height")
                ok, mi = buf.map(Gst.MapFlags.READ)
                if not ok:
                    continue
                try:
                    stride = len(mi.data) // h
                    img = np.frombuffer(mi.data, np.uint8).reshape(h, stride)[:, :w].copy()
                finally:
                    buf.unmap(mi)
                last_ok = now
                with self._lock:
                    self._frame, self._t, self._seq = img, now, self._seq + 1
            pipe.set_state(Gst.State.NULL)


def find_v4l2(name: str) -> str:
    """以裝置名稱找 V4L2 擷取節點（重新插拔後 /dev/videoN 編號會變）"""
    base = "/sys/class/video4linux"
    for d in sorted(os.listdir(base)) if os.path.isdir(base) else []:
        try:
            with open(f"{base}/{d}/name") as f:
                nm = f.read()
            with open(f"{base}/{d}/index") as f:
                idx = int(f.read())
        except OSError:
            continue
        if name in nm and idx == 0:
            return f"/dev/{d}"
    return ""


def undistort_maps(fx, fy, cx, cy, dist, w, h):
    """去畸變對照表；輸出影像沿用同一組 K，所以去畸變後直接用 fx/fy/cx/cy 當針孔模型"""
    import cv2
    K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1.0]])
    return cv2.initUndistortRectifyMap(K, np.array(dist, float), None, K, (w, h), cv2.CV_16SC2)


def load_camera_profile(path: str) -> dict:
    """camera_profiles/*.json（cam_test.py calib 產生）→ AppConfig 欄位"""
    with open(path, encoding="utf-8") as f:
        p = json.load(f)
    return {"camera_source": "device", "cam_name": p["device_name"], "cam_width": p["width"], "cam_height": p["height"],
            "cam_fps": p["fps"], "cam_exposure": p.get("exposure"), "fx": p["fx"], "fy": p["fy"], "cx": p["cx"],
            "cy": p["cy"], "cam_dist": tuple(p["dist"]), "video_latency_s": p.get("capture_to_host_s", 0.0)}


class UsbCamera:
    """USB UVC 相機（V4L2 + MJPEG），回傳灰階；參數見 camera_profiles/*.json

    - 直接取 MJPEG 原始資料再解成灰階：比 OpenCV 預設的 BGR 解碼省時間
    - 不可設 CAP_PROP_BUFFERSIZE = 1：驅動只剩一個緩衝區時幀率會掉一半（實測 30 → 15 fps）
    - 緩衝區留著預設值時，靠 backlog_s() 偵測「讀取跟不上、拿到舊幀」，並跳過解碼把舊幀清掉
    - remap：內參校正的去畸變對照表（cv2.initUndistortRectifyMap），偵測前先套用
    """

    def __init__(self, device, width: int = 1280, height: int = 720, fps: int = 30, exposure: Optional[float] = None,
                 remap=None):
        self.device, self.width, self.height, self.fps = device, width, height, fps
        self.exposure, self.remap = exposure, remap
        self._lock = threading.Lock()
        self._frame, self._t, self._seq = None, 0.0, 0
        self._t_buf = 0.0
        self._backlog = 0.0
        self._base = deque(maxlen=int(fps * 5))        # 近 5 s 的「取得 - 驅動時間戳」，取最小當基準
        self.skipped = 0
        self._running = threading.Event()
        self._thread: Optional[threading.Thread] = None

    def start(self) -> None:
        self._running.set()
        self._thread = threading.Thread(target=self._loop, name="usbcam", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._running.clear()
        if self._thread:
            self._thread.join(timeout=3.0)

    def latest(self):
        with self._lock:
            return self._frame, self._t, self._seq

    def latest_buf_time(self) -> float:
        with self._lock:
            return self._t_buf

    def backlog_s(self) -> float:
        with self._lock:
            return self._backlog

    def _open(self):
        import cv2
        cap = cv2.VideoCapture(self.device, cv2.CAP_V4L2)
        if not cap.isOpened():
            return None
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        cap.set(cv2.CAP_PROP_FPS, self.fps)
        cap.set(cv2.CAP_PROP_CONVERT_RGB, 0)
        if self.exposure is not None:
            cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)      # V4L2：1 = 手動、3 = 自動
            cap.set(cv2.CAP_PROP_EXPOSURE, self.exposure)
        else:
            cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 3)
        got = (int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))
        if got != (self.width, self.height):
            log.error("相機解析度 %dx%d，不是要求的 %dx%d（內參不適用），停止使用", got[0], got[1], self.width, self.height)
            cap.release()
            return None
        return cap

    def _loop(self) -> None:
        import cv2
        while self._running.is_set():
            cap = self._open()
            if cap is None:
                log.warning("相機開啟失敗：%s，1 s 後重試", self.device)
                time.sleep(1.0)
                continue
            log.info("相機 %s：%dx%d@%d MJPEG，曝光 %s", self.device, self.width, self.height, self.fps,
                     "自動" if self.exposure is None else self.exposure)
            self._base.clear()
            while self._running.is_set():
                ok, raw = cap.read()
                now = time.monotonic()
                if not ok or raw is None or raw.size == 0:
                    log.warning("相機讀取失敗，重新開啟")
                    break
                t_buf = cap.get(cv2.CAP_PROP_POS_MSEC) / 1000.0
                age = now - t_buf
                self._base.append(age)
                backlog = max(0.0, age - min(self._base))
                with self._lock:
                    self._backlog = backlog
                if backlog > 1.5 / self.fps:            # 佇列裡是舊幀：不解碼，盡快讀到最新一幀
                    self.skipped += 1
                    continue
                g = cv2.imdecode(raw.reshape(-1), cv2.IMREAD_GRAYSCALE)
                if g is None:                           # USB 傳輸錯誤造成的壞 JPEG
                    self.skipped += 1
                    continue
                if self.remap is not None:
                    g = cv2.remap(g, self.remap[0], self.remap[1], cv2.INTER_LINEAR)
                with self._lock:
                    self._frame, self._t, self._t_buf, self._seq = g, now, t_buf, self._seq + 1
            cap.release()


class CvCamera:
    """實機：RTSP / 其他 OpenCV 來源，回傳灰階（USB 相機請用 UsbCamera）"""

    def __init__(self, source):
        self.source = source
        self._lock = threading.Lock()
        self._frame, self._t, self._seq = None, 0.0, 0
        self._running = threading.Event()
        self._thread: Optional[threading.Thread] = None

    def start(self) -> None:
        self._running.set()
        self._thread = threading.Thread(target=self._loop, name="cam", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._running.clear()
        if self._thread:
            self._thread.join(timeout=3.0)

    def latest(self):
        with self._lock:
            return self._frame, self._t, self._seq

    def _loop(self) -> None:
        import cv2
        os.environ.setdefault("OPENCV_FFMPEG_CAPTURE_OPTIONS", "rtsp_transport;tcp|fflags;nobuffer|flags;low_delay")
        while self._running.is_set():
            cap = cv2.VideoCapture(self.source)
            if not cap.isOpened():
                log.warning("影像開啟失敗：%s，1 s 後重試", self.source)
                time.sleep(1.0)
                continue
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            while self._running.is_set():
                ok, fr = cap.read()
                now = time.monotonic()
                if not ok:
                    break
                g = cv2.cvtColor(fr, cv2.COLOR_BGR2GRAY) if fr.ndim == 3 else fr
                with self._lock:
                    self._frame, self._t, self._seq = g, now, self._seq + 1
            cap.release()


# ---------------------------------------------------------------------------
# AprilTag 偵測
# ---------------------------------------------------------------------------


@dataclass
class TagMeas:
    ok: bool = False
    tag_id: int = -1
    ids: str = ""
    u: float = math.nan
    v: float = math.nan
    edge_px: float = math.nan
    margin: float = math.nan
    p_cam: Optional[np.ndarray] = None     # 相機光軸座標下平台中心位置（m）
    yaw_cam: float = math.nan              # tag 在影像中的旋轉（rad）
    detect_ms: float = math.nan


def make_detector(cfg: AppConfig):
    """pupil_apriltags 固定用 2 bits 錯誤更正（48h12 需數 GB 記憶體），這裡改用 cfg.hamming_bits"""
    from pupil_apriltags import Detector
    det = Detector(families="tag16h5", nthreads=cfg.nthreads, quad_decimate=cfg.quad_decimate,
                   refine_edges=1, decode_sharpening=0.25)
    lib = det.libc
    lib.apriltag_detector_clear_families(det.tag_detector_ptr)
    lib.tag16h5_destroy.restype = None
    lib.tag16h5_destroy(det.tag_families.pop("tag16h5"))
    create = getattr(lib, f"{cfg.tag_family}_create")
    create.restype = lib.tag16h5_create.restype
    tf = create()
    det.tag_families[cfg.tag_family] = tf
    lib.apriltag_detector_add_family_bits(det.tag_detector_ptr, tf, int(cfg.hamming_bits))
    det.params["families"] = [cfg.tag_family]
    return det


class NestedTagDetector:
    def __init__(self, cfg: AppConfig):
        self.cfg = cfg
        self.det = make_detector(cfg)

    def detect(self, gray: np.ndarray, K: Tuple[float, float, float, float]) -> TagMeas:
        t0 = time.perf_counter()
        # tag_size = 1 → pose_t 再乘上該 ID 的實際尺寸（平移與 tag 尺寸成正比）
        res = self.det.detect(gray, estimate_tag_pose=True, camera_params=K, tag_size=1.0)
        m = TagMeas(detect_ms=(time.perf_counter() - t0) * 1000.0)
        good = []
        for r in res:
            if r.tag_id not in self.cfg.tag_sizes or r.decision_margin < self.cfg.tag_min_margin:
                continue
            c = r.corners
            edge = float(np.mean([np.linalg.norm(c[i] - c[(i + 1) % 4]) for i in range(4)]))
            if edge < self.cfg.tag_min_px:
                continue
            good.append((edge, r))
        m.ids = " ".join(str(r.tag_id) for _, r in sorted(good, key=lambda x: x[1].tag_id))
        if not good:
            return m
        edge, r = max(good, key=lambda x: x[0])         # 影像中最大的 tag：角點最精準
        p = np.asarray(r.pose_t, float).ravel() * self.cfg.tag_sizes[r.tag_id]
        if not np.all(np.isfinite(p)) or p[2] <= 0:
            return m
        R = np.asarray(r.pose_R, float)
        m.ok, m.tag_id, m.edge_px, m.margin = True, int(r.tag_id), edge, float(r.decision_margin)
        m.u, m.v = float(r.center[0]), float(r.center[1])
        m.p_cam = p
        m.yaw_cam = math.atan2(R[1, 0], R[0, 0])
        return m


# ---------------------------------------------------------------------------
# MAVLink
# ---------------------------------------------------------------------------


class MavLink:
    def __init__(self, cfg: AppConfig):
        self.cfg = cfg
        kw = dict(source_system=cfg.source_system, source_component=cfg.source_component, autoreconnect=True)
        if cfg.conn.startswith("/dev/"):
            kw["baud"] = cfg.baud
        self.m = mavutil.mavlink_connection(cfg.conn, **kw)
        self.tx_lock = threading.Lock()
        self.clock = BootClock()
        self.att = TimedBuffer(maxlen=20000, angle_idx=(2,))           # roll pitch yaw p q r（拍攝時間內插用）
        self.pos = TimedBuffer(maxlen=500)               # LOCAL_POSITION_NED x y z vx vy vz
        self.truth = TimedBuffer(maxlen=20000, angle_idx=(5,))         # SIM_STATE：N E D(相對開機點) roll pitch yaw vn ve vd
        self.latest: Dict[str, object] = {}
        self.mode = "?"
        self.armed = False
        self.landed_state = 0
        self.statustexts: deque = deque(maxlen=200)
        self.truth_origin = None                          # (lat, lon, alt)
        self._running = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self.heartbeat_seen = threading.Event()
        self._ready_hint_shown = False
        self.params: Dict[str, float] = {}

    def start(self) -> None:
        self._running.set()
        self._thread = threading.Thread(target=self._loop, name="mav", daemon=True)
        self._thread.start()
        if not self.heartbeat_seen.wait(timeout=60):
            raise RuntimeError(f"60 s 內沒有收到飛控 heartbeat（{self.cfg.conn}）")
        self.request_streams()
        threading.Thread(target=self._gcs_heartbeat, name="mav-hb", daemon=True).start()

    def stop(self) -> None:
        self._running.clear()
        if self._thread:
            self._thread.join(timeout=2.0)

    def request_streams(self) -> None:
        for _ in range(2):
            self._request_rates()
            time.sleep(0.3)

    def _request_rates(self) -> None:
        M = mavutil.mavlink
        rates = {M.MAVLINK_MSG_ID_ATTITUDE: 50, M.MAVLINK_MSG_ID_LOCAL_POSITION_NED: 30,
                 M.MAVLINK_MSG_ID_GLOBAL_POSITION_INT: 10, M.MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT: 20,
                 M.MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED: 20, M.MAVLINK_MSG_ID_RC_CHANNELS: 10,
                 M.MAVLINK_MSG_ID_EXTENDED_SYS_STATE: 5, M.MAVLINK_MSG_ID_SERVO_OUTPUT_RAW: 10}
        if self.cfg.sim_truth:
            rates[M.MAVLINK_MSG_ID_SIM_STATE] = 50
        for mid, hz in rates.items():
            self.command_long(M.MAV_CMD_SET_MESSAGE_INTERVAL, mid, 1e6 / hz)

    def _gcs_heartbeat(self) -> None:
        n = 0
        while self._running.is_set():
            try:
                with self.tx_lock:
                    self.m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                                              mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
                n += 1
                if n % 5 == 0:      # 定期重設串流頻率（GCS / MAVProxy 可能覆寫）
                    self._request_rates()
            except OSError as e:    # 連線中斷（SITL 重開等）：不要讓執行緒結束，等重新連線
                log.warning("MAVLink 連線中斷：%s", e)
            time.sleep(1.0)

    def _loop(self) -> None:
        while self._running.is_set():
            try:
                msg = self.m.recv_match(blocking=True, timeout=0.5)
            except Exception as e:      # noqa: BLE001
                log.debug("mavlink recv: %s", e)
                continue
            if msg is None:
                continue
            now = time.monotonic()
            if msg.get_srcSystem() != self.cfg.target_system:
                continue
            self._handle(msg, now)

    def _host_t(self, msg, now: float) -> float:
        """時間戳一律用 host 接收時間：SITL 與 Gazebo lockstep，模擬時間與實際時間會漂移
        （Gazebo 慢於即時時飛控時鐘變慢），用 time_boot_ms 換算會造成時間倒退 / 資料遺失。
        time_boot_ms 只用於紀錄。"""
        tb = getattr(msg, "time_boot_ms", None)
        if tb is not None:
            self.clock.update(tb / 1000.0, now)
        return now

    def _handle(self, msg, now: float) -> None:
        typ = msg.get_type()
        if typ == "BAD_DATA":
            return
        self.latest[typ] = msg
        if typ == "HEARTBEAT":
            if msg.get_srcComponent() != self.cfg.target_component:
                return
            self.mode = COPTER_MODES.get(msg.custom_mode, str(msg.custom_mode))
            self.armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            self.heartbeat_seen.set()
        elif typ == "ATTITUDE":
            self.att.append(self._host_t(msg, now), [msg.roll, msg.pitch, msg.yaw,
                                                     msg.rollspeed, msg.pitchspeed, msg.yawspeed])
        elif typ == "LOCAL_POSITION_NED":
            self.pos.append(self._host_t(msg, now), [msg.x, msg.y, msg.z, msg.vx, msg.vy, msg.vz])
        elif typ == "EXTENDED_SYS_STATE":
            self.landed_state = msg.landed_state
        elif typ == "PARAM_VALUE":
            self.params[str(msg.param_id).rstrip("\x00")] = float(msg.param_value)
        elif typ == "STATUSTEXT":
            txt = msg.text.rstrip("\x00") if isinstance(msg.text, str) else str(msg.text)
            self.statustexts.append((now, txt))
            log.info("[FC] %s", txt)
            if ("is using GPS" in txt and self.cfg.mode == "sim" and not self.cfg.sim_auto
                    and not self._ready_hint_shown):
                self._ready_hint_shown = True
                log.info("==== 飛控已可解鎖（EKF 使用 GPS）。MAVProxy：mode guided → arm throttle → takeoff 6 "
                         "→ 飛到平台上方附近 → 出現 PrecLand: Init Complete 後 mode land ====")
        elif typ == "SIM_STATE":
            lat = msg.lat_int * 1e-7 if getattr(msg, "lat_int", 0) else msg.lat
            lon = msg.lon_int * 1e-7 if getattr(msg, "lon_int", 0) else msg.lon
            if self.truth_origin is None:
                self.truth_origin = (lat, lon, msg.alt)
            lat0, lon0, alt0 = self.truth_origin
            n = math.radians(lat - lat0) * 6378137.0
            e = math.radians(lon - lon0) * 6378137.0 * math.cos(math.radians(lat0))
            self.truth.append(now, [n, e, -(msg.alt - alt0), msg.roll, msg.pitch, msg.yaw, msg.vn, msg.ve, msg.vd])

    # ---- 送出 ----
    def command_long(self, cmd: int, *params: float) -> None:
        p = list(params) + [0.0] * (7 - len(params))
        with self.tx_lock:
            self.m.mav.command_long_send(self.cfg.target_system, self.cfg.target_component, cmd, 0, *p)

    def get_param(self, name: str, timeout: float = 3.0) -> Optional[float]:
        self.params.pop(name, None)
        t_end = time.monotonic() + timeout
        while time.monotonic() < t_end:
            with self.tx_lock:
                self.m.mav.param_request_read_send(self.cfg.target_system, self.cfg.target_component,
                                                   name.encode(), -1)
            t_retry = time.monotonic() + 0.5
            while time.monotonic() < t_retry:
                if name in self.params:
                    return self.params[name]
                time.sleep(0.02)
        return None

    def set_param(self, name: str, value: float) -> None:
        with self.tx_lock:
            self.m.mav.param_set_send(self.cfg.target_system, self.cfg.target_component, name.encode(), float(value),
                                      mavutil.mavlink.MAV_PARAM_TYPE_REAL32)

    def set_mode(self, custom_mode: int) -> None:
        self.command_long(mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                          mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, custom_mode)

    def send_landing_target(self, time_usec: int, frame: int, xyz: np.ndarray, size_rad: float) -> None:
        x, y, z = (float(v) for v in xyz)
        dist = math.sqrt(x * x + y * y + z * z)
        # angle_x / angle_y 也一併填（ArduPilot：vec = (-tan(angle_y), tan(angle_x), 1)）
        ang_x = math.atan2(y, z)
        ang_y = math.atan2(-x, z)
        with self.tx_lock:
            self.m.mav.landing_target_send(
                time_usec, 0, frame, ang_x, ang_y, dist, size_rad, size_rad,
                x, y, z, [1.0, 0.0, 0.0, 0.0], LANDING_TARGET_TYPE_VISION_FIDUCIAL, 1)

    def goto_local(self, n: float, e: float, d: float, yaw: float) -> None:
        mask = 0b0000_1001_1111_1000          # 只用位置 + yaw
        with self.tx_lock:
            self.m.mav.set_position_target_local_ned_send(
                0, self.cfg.target_system, self.cfg.target_component, mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                mask, n, e, d, 0, 0, 0, 0, 0, 0, yaw, 0)


# ---------------------------------------------------------------------------
# Sim：Gazebo 真值
# ---------------------------------------------------------------------------


def gz_pose(name: str) -> List[float]:
    out = subprocess.check_output(["gz", "model", "-m", name, "-p"], text=True, timeout=15)
    vals = [float(v) for v in out.split()]
    if len(vals) < 6:
        raise RuntimeError(f"gz model -m {name} -p 回傳異常：{out!r}")
    return vals[:6]


def gz_to_ned(p) -> np.ndarray:
    """ArduPilotPlugin：gazeboXYZToNED = roll 180° → N = x, E = -y, D = -z"""
    return np.array([p[0], -p[1], -p[2]])


class SimTruth:
    """啟動時讀一次 Gazebo 位置；飛行中真值用 SIM_STATE（不持續訂閱 gz topic）"""

    def __init__(self, cfg: AppConfig, mav: MavLink):
        self.cfg, self.mav = cfg, mav
        self.vehicle0_ned = None         # 開機時無人機在 Gazebo 世界的 NED 位置
        self.marker_ned = None           # 平台中心（tag 平面）在 Gazebo 世界的 NED 位置
        self.pad_yaw_ned = math.nan

    def init(self) -> bool:
        t0 = time.monotonic()
        while self.mav.truth_origin is None and time.monotonic() - t0 < 10:
            time.sleep(0.1)
        if self.mav.truth_origin is None:
            log.warning("沒有收到 SIM_STATE，無法比對真值")
            return False
        try:
            drone = gz_pose(self.cfg.sim_vehicle_model)
            pad = gz_pose(self.cfg.sim_pad_model)
        except Exception as e:  # noqa: BLE001
            log.warning("讀取 Gazebo 真值失敗：%s", e)
            return False
        _, tv = self.mav.truth.latest()
        # SIM_STATE 原點 = 第一筆時的位置；用當下 gz 位置對齊
        self.vehicle0_ned = gz_to_ned(drone[:3]) - np.array(tv[:3])
        self.marker_ned = gz_to_ned([pad[0], pad[1], pad[2] + self.cfg.sim_pad_top_z])
        self.pad_yaw_ned = wrap_pi(-pad[5])
        log.info("模擬真值：平台中心（相對起飛點）NED = %s，平台 yaw %.1f°",
                 np.round(self.marker_rel_home(), 3), math.degrees(self.pad_yaw_ned))
        return True

    @property
    def ready(self) -> bool:
        return self.marker_ned is not None

    def marker_rel_home(self) -> np.ndarray:
        """平台中心相對開機點（= SITL / EKF 原點）的 NED"""
        return self.marker_ned - self.vehicle0_ned

    def at(self, t: float):
        """t 時刻：(機體真值位置 NED 相對開機點, roll, pitch, yaw)"""
        v = self.mav.truth.at(t, max_gap=0.2)
        if v is None:
            return None
        return np.array(v[:3]), v[3], v[4], v[5]

    def cam_to_marker_ned(self, t: float) -> Optional[np.ndarray]:
        s = self.at(t)
        if s is None or not self.ready:
            return None
        p, r, pi, y = s
        cam = p + rot_zyx(r, pi, y) @ np.array(self.cfg.sim_cam_offset_body)
        return self.marker_rel_home() - cam

    def landing_result(self) -> dict:
        drone = gz_pose(self.cfg.sim_vehicle_model)
        pad = gz_pose(self.cfg.sim_pad_model)
        dx, dy = drone[0] - pad[0], drone[1] - pad[1]
        # 平台座標系下的誤差（前後 / 左右）
        c, s = math.cos(pad[5]), math.sin(pad[5])
        return {"error_m": math.hypot(dx, dy), "err_pad_x_m": c * dx + s * dy, "err_pad_y_m": -s * dx + c * dy,
                "on_pad": drone[2] > pad[2] + 0.2, "drone_gz": drone, "pad_gz": pad,
                "drone_tilt_deg": math.degrees(math.hypot(drone[3], drone[4]))}


# ---------------------------------------------------------------------------
# Sim：自動測試
# ---------------------------------------------------------------------------


class SimAutoTest:
    """解鎖 → GUIDED 起飛 → 飛到平台附近（加定位誤差）→ 懸停 → LAND → 上鎖 → 量測"""

    def __init__(self, app: "PrecisionLandApp"):
        self.app = app
        self.mav, self.cfg, self.truth = app.mav, app.cfg, app.truth
        self.result: Optional[dict] = None
        self.phase = "init"
        self.t_land: Optional[float] = None
        self._thread = threading.Thread(target=self._run, name="sim-auto", daemon=True)
        self.rng = random.Random(self.cfg.sim_seed)

    def start(self) -> None:
        self._thread.start()

    def _wait(self, cond, timeout: float, what: str) -> bool:
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout and not self.app.stop_event.is_set():
            if cond():
                return True
            time.sleep(0.1)
        if not self.app.stop_event.is_set():
            log.warning("[auto] 等待逾時：%s", what)
        return False

    def _texts_since(self, t: float) -> List[str]:
        return [s for ts, s in list(self.mav.statustexts) if ts >= t]

    def _run(self) -> None:
        try:
            self._sequence()
        except Exception as e:  # noqa: BLE001
            log.exception("[auto] 例外：%s", e)
            self.result = {"landed": False, "error": str(e)}
        finally:
            self.app.stop_event.set()

    def _sequence(self) -> None:
        M = mavutil.mavlink
        mav = self.mav
        if not self._wait(lambda: self.truth.ready, 30, "Gazebo 真值"):
            self.result = {"landed": False, "error": "no truth"}
            return
        # 1. 解鎖（等 EKF / GPS 就緒；失敗就重試）
        self.phase = "arm"
        t0 = time.monotonic()
        while not mav.armed and time.monotonic() - t0 < 120 and not self.app.stop_event.is_set():
            mav.set_mode(4)                      # GUIDED
            time.sleep(0.5)
            mav.command_long(M.MAV_CMD_COMPONENT_ARM_DISARM, 1)
            if self._wait(lambda: mav.armed, 3, "解鎖"):
                break
            time.sleep(2.0)
        if not mav.armed:
            self.result = {"landed": False, "error": "arm failed"}
            return
        # 2. 起飛
        self.phase = "takeoff"
        log.info("[auto] 起飛到 %.1f m", self.cfg.sim_takeoff_alt_m)
        mav.command_long(M.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, self.cfg.sim_takeoff_alt_m)
        ok = self._wait(lambda: (mav.pos.latest()[1] or [0, 0, 0])[2] < -(self.cfg.sim_takeoff_alt_m - 0.3),
                        40, "起飛高度")
        if not ok:
            self.result = {"landed": False, "error": "takeoff failed"}
            return
        # 3. 飛到平台附近（模擬 GPS 航點誤差）
        self.phase = "approach"
        marker = self.truth.marker_rel_home()
        ang = self.rng.uniform(0, 2 * math.pi)
        r = self.cfg.sim_approach_err_m * math.sqrt(self.rng.random()) if self.cfg.sim_approach_err_m > 0 else 0.0
        yaw = (math.radians(self.cfg.sim_approach_yaw_deg) if self.cfg.sim_approach_yaw_deg is not None
               else self.rng.uniform(-math.pi, math.pi))
        goal = (marker[0] + r * math.cos(ang), marker[1] + r * math.sin(ang), -self.cfg.sim_takeoff_alt_m)
        log.info("[auto] 前往 NED (%.2f, %.2f, %.2f)，偏離平台 %.2f m，yaw %.0f°",
                 goal[0], goal[1], goal[2], r, math.degrees(yaw))
        self.approach = {"approach_offset_m": r, "approach_dir_deg": math.degrees(ang),
                         "approach_yaw_deg": math.degrees(yaw)}

        def arrived():
            mav.goto_local(goal[0], goal[1], goal[2], yaw)
            p = mav.pos.latest()[1]
            a = mav.att.latest()[1]
            return (p is not None and a is not None and math.hypot(p[0] - goal[0], p[1] - goal[1]) < 0.3
                    and math.hypot(p[3], p[4]) < 0.2 and abs(wrap_pi(a[2] - yaw)) < math.radians(5))
        if not self._wait(arrived, 60, "抵達平台上方"):
            self.result = {"landed": False, "error": "approach failed"}
            return
        time.sleep(self.cfg.sim_hover_s)
        # 4. LAND（PLND_ENABLED = 1 → LAND 自動使用 precision landing）
        self.phase = "land"
        self.t_land = time.monotonic()
        log.info("[auto] 切換 LAND")
        mav.set_mode(9)
        landed = self._wait(lambda: not mav.armed, self.cfg.sim_land_timeout_s, "降落並上鎖")
        dur = time.monotonic() - self.t_land
        self.phase = "landed" if landed else "timeout"
        time.sleep(1.5)
        res = {"landed": landed, "land_duration_s": dur, **self.approach}
        try:
            res.update(self.truth.landing_result())
        except Exception as e:  # noqa: BLE001
            res["error"] = f"truth: {e}"
        texts = self._texts_since(self.t_land)
        res["fc_messages"] = texts
        res["target_lost_count"] = sum("Target Lost" in s for s in texts)
        res["retry_count"] = sum("retry" in s.lower() for s in texts)
        self.result = res
        log.info("[auto] 落地=%s 耗時 %.1f s，與平台中心水平誤差 %.1f mm（平台座標 x %.1f / y %.1f mm），在平台上=%s",
                 landed, dur, res.get("error_m", math.nan) * 1000, res.get("err_pad_x_m", math.nan) * 1000,
                 res.get("err_pad_y_m", math.nan) * 1000, res.get("on_pad"))


# ---------------------------------------------------------------------------
# 每幀紀錄
# ---------------------------------------------------------------------------


def _cols(prefix, names):
    return [f"{prefix}{n}" for n in names]


class FrameLogger:
    def __init__(self, cfg: AppConfig):
        os.makedirs(cfg.log_dir, exist_ok=True)
        self.path = os.path.join(cfg.log_dir, datetime.datetime.now().strftime("plnd_%Y%m%d_%H%M%S.csv"))
        self.columns = (
            ["影像ID", "影像積壓_s", "取得時間_s", "拍攝時間_s", "飛控時間_s", "模式", "解鎖", "落地狀態", "自動階段"]
            + ["偵測", "使用ID", "偵測到ID", "像素u", "像素v", "tag邊長_px", "decision_margin", "偵測耗時_ms"]
            + _cols("相機", ["x_m", "y_m", "z_m"])
            + ["送出", "送出frame", "送出x_m", "送出y_m", "送出z_m", "送出距離_m", "處理延遲_ms"]
            + _cols("量測目標相對", ["N_m", "E_m", "D_m"])
            + _cols("位置", ["N_m", "E_m", "D_m"]) + _cols("目標位置", ["N_m", "E_m", "D_m"])
            + _cols("速度", ["N_mps", "E_mps", "D_mps"]) + _cols("目標速度", ["N_mps", "E_mps", "D_mps"])
            + ["Roll_deg", "Pitch_deg", "Yaw_deg", "目標Roll_deg", "目標Pitch_deg", "目標航向_deg",
               "Roll速率_dps", "Pitch速率_dps", "Yaw速率_dps"]
            + ["RC_RSSI"] + [f"CH{i}_us" for i in range(1, cfg.rc_channels_logged + 1)]
            + _cols("真值位置", ["N_m", "E_m", "D_m"]) + ["真值Roll_deg", "真值Pitch_deg", "真值Yaw_deg"]
            + _cols("真值目標相對", ["N_m", "E_m", "D_m"])
            + ["量測水平誤差_m", "量測垂直誤差_m", "EKF位置水平誤差_m", "真值與平台水平距離_m"]
        )
        self.f = open(self.path, "w", newline="", encoding="utf-8-sig")
        self.w = csv.writer(self.f)
        self.w.writerow(self.columns)
        self.rows = 0
        log.info("每幀紀錄 → %s", self.path)

    def write(self, values: dict) -> None:
        row = []
        for c in self.columns:
            v = values.get(c)
            if v is None or (isinstance(v, float) and not math.isfinite(v)):
                row.append("")
            elif isinstance(v, float):
                row.append(f"{v:.5f}")
            else:
                row.append(v)
        self.w.writerow(row)
        self.rows += 1
        if self.rows % 30 == 0:
            self.f.flush()

    def close(self) -> None:
        self.f.close()
        try:
            import pandas as pd
            pd.read_csv(self.path, encoding="utf-8-sig").to_excel(self.path.replace(".csv", ".xlsx"), index=False)
            log.info("Excel：%s", self.path.replace(".csv", ".xlsx"))
        except Exception as e:  # noqa: BLE001
            log.info("未產生 Excel（%s），CSV 可直接用 Excel 開啟", e)


# ---------------------------------------------------------------------------
# Application
# ---------------------------------------------------------------------------


def read_parm_file(path: str) -> Dict[str, float]:
    out: Dict[str, float] = {}
    with open(path, encoding="utf-8") as f:
        for line in f:
            p = line.split("#")[0].split()
            if len(p) >= 2:
                out[p[0]] = float(p[1])
    return out


def ensure_precland_params(mav: MavLink, cfg: AppConfig) -> None:
    """飛控沒設 PLND_TYPE = 1 時，LANDING_TARGET 會被忽略，LAND 變成原地降落（而且不會報錯）

    --add-param-file 只是當次啟動的預設值，不會存進 EEPROM，啟動指令少帶就回到 gazebo-iris.parm 的 PLND_TYPE = 3。
    sim：未解鎖時寫入整份參數檔並重開飛控（存進 EEPROM，之後不必再帶 --add-param-file）
    real：不動飛控參數，只報錯
    只有 PLND_TYPE / PLND_ENABLED 不對才動作：代表參數檔沒載入；不覆蓋使用者另外調的參數
    """
    enabled, ptype = mav.get_param("PLND_ENABLED"), mav.get_param("PLND_TYPE")
    if enabled is None:
        log.warning("讀不到飛控 PLND_ENABLED，無法確認 Precision Landing 設定")
        return
    if enabled == 1 and ptype == 1:
        log.info("飛控 Precision Landing 設定正確（PLND_TYPE = 1）")
        return
    problem = f"飛控 PLND_ENABLED = {enabled:g}、PLND_TYPE = {ptype}（需要 1 / 1）：LANDING_TARGET 會被忽略，LAND 只會原地降落"
    if cfg.mode != "sim":
        log.error("%s。請在 GCS 載入 params/precland_real_uhd4k.parm 後重開飛控", problem)
        return
    if mav.armed:
        log.error("%s。飛機已解鎖，無法自動修正：請落地後重新啟動本程式", problem)
        return
    log.warning("%s。自動寫入 %s 並重開飛控", problem, os.path.basename(SIM_PARAM_FILE))
    want = read_parm_file(SIM_PARAM_FILE)
    for k, v in want.items():
        mav.set_param(k, v)
        time.sleep(0.03)
    time.sleep(2.0)                                     # 等 EEPROM 寫入
    mav.command_long(mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, 1)
    time.sleep(3.0)
    t_end = time.monotonic() + 45
    while time.monotonic() < t_end:
        if mav.get_param("PLND_TYPE", 1.5) == 1:
            mav.request_streams()
            log.info("飛控已重開，PLND_TYPE = 1 生效並存入 EEPROM")
            return
    log.error("飛控重開後 PLND_TYPE 仍不是 1，LAND 會原地降落。請用 README 的指令重新啟動 SITL")


class PrecisionLandApp:
    def __init__(self, cfg: AppConfig):
        self.cfg = cfg
        self.stop_event = threading.Event()
        if cfg.camera_source == "udp":
            self.cam = GstUdpCamera(cfg.udp_port)
        elif cfg.camera_source == "device":
            dev = cfg.device or find_v4l2(cfg.cam_name)
            if not dev:
                raise SystemExit(f"找不到相機 {cfg.cam_name}，請用 --device /dev/videoN 指定")
            remap = None
            if cfg.cam_dist:
                if not cfg.fx:
                    raise SystemExit("有畸變係數但沒有 fx（請用 --cam-profile）")
                remap = undistort_maps(cfg.fx, cfg.fy or cfg.fx, cfg.cx, cfg.cy, cfg.cam_dist, cfg.cam_width, cfg.cam_height)
            self.cam = UsbCamera(dev, cfg.cam_width, cfg.cam_height, cfg.cam_fps, cfg.cam_exposure, remap)
        else:
            if not cfg.rtsp_url:
                raise SystemExit("real 模式需指定 --cam-profile（USB 相機）或 --rtsp URL")
            self.cam = CvCamera(cfg.rtsp_url)
        self.detector = NestedTagDetector(cfg)
        self.mav = MavLink(cfg)
        self.truth = SimTruth(cfg, self.mav) if cfg.sim_truth else None
        self.auto: Optional[SimAutoTest] = None
        self.logger = FrameLogger(cfg)
        self.C_bc = np.array(cfg.cam_to_body, float)
        self.latency_samples: List[Tuple[float, np.ndarray]] = []
        self.stats = {"frames": 0, "detections": 0, "sent": 0}
        self.meas_err: List[Tuple[float, float, float]] = []   # (真值高度, 水平誤差, 垂直誤差)
        self._proc_delays: List[float] = []                    # 取得影像 → 送出 LANDING_TARGET（ms）

    def stop(self, *_):
        self.stop_event.set()

    def intrinsics(self, w: int, h: int) -> Tuple[float, float, float, float]:
        c = self.cfg
        fx = c.fx or (w / 2) / math.tan(math.radians(c.fov_horizontal_deg) / 2)
        fy = c.fy or fx
        return fx, fy, (c.cx or w / 2), (c.cy or h / 2)

    def run(self) -> dict:
        signal.signal(signal.SIGINT, self.stop)
        signal.signal(signal.SIGTERM, self.stop)
        log.info("連線飛控 %s", self.cfg.conn)
        self.mav.start()
        ensure_precland_params(self.mav, self.cfg)
        self.cam.start()
        if self.truth is not None:
            self.truth.init()
        if self.cfg.sim_auto:
            self.auto = SimAutoTest(self)
            self.auto.start()
        t_start = time.monotonic()
        last_seq, last_send = -1, 0.0
        min_send_dt = 1.0 / self.cfg.max_send_hz
        last_print = 0.0
        last_stale_warn = 0.0
        try:
            while not self.stop_event.is_set():
                if self.cfg.run_timeout_s > 0 and time.monotonic() - t_start > self.cfg.run_timeout_s:
                    log.warning("超過 run_timeout %.0f s，結束", self.cfg.run_timeout_s)
                    break
                frame, t_arr, seq = self.cam.latest()
                if frame is None or seq == last_seq:
                    time.sleep(0.002)
                    continue
                last_seq = seq
                self.stats["frames"] += 1
                h, w = frame.shape[:2]
                K = self.intrinsics(w, h)
                det = self.detector.detect(frame, K)
                backlog = self.cam.backlog_s() if hasattr(self.cam, "backlog_s") else 0.0
                t_cap = t_arr - self.cfg.video_latency_s - backlog
                # 影像總年齡 = 佇列積壓 + 取得後的處理時間（CPU 滿載時延遲主要在後者，積壓看不出來）
                age = backlog + time.monotonic() - t_arr
                stale = age > self.cfg.max_video_backlog_s
                if stale:
                    self.stats["stale_frames"] = self.stats.get("stale_frames", 0) + 1
                    if time.monotonic() - last_stale_warn > 2.0:
                        last_stale_warn = time.monotonic()
                        log.warning("影像太舊 %.2f s（上限 %.2f）：停止送 LANDING_TARGET。電腦負載過高，"
                                    "請關閉 Gazebo 畫面 / MAVProxy 地圖，或降低相機解析度", age,
                                    self.cfg.max_video_backlog_s)
                self.stats["max_backlog_s"] = max(self.stats.get("max_backlog_s", 0.0), backlog)
                row = self._row_base(seq, t_arr, t_cap, det)
                row["影像積壓_s"] = backlog
                if det.ok:
                    self.stats["detections"] += 1
                    p_body = self.C_bc @ det.p_cam
                    a = self.mav.att.at(t_cap)
                    if self.cfg.send_frame == "local" and a is not None:
                        R_lvl = rot_zyx(a[0], a[1], 0.0)
                        send_vec, frame_id = R_lvl @ p_body, MAV_FRAME_LOCAL_FRD
                    else:
                        send_vec, frame_id = p_body, MAV_FRAME_BODY_FRD
                    now = time.monotonic()
                    sent = False
                    if now - last_send >= min_send_dt and not stale:
                        size = 2 * math.atan2(self.cfg.tag_sizes[det.tag_id] / 2, float(np.linalg.norm(det.p_cam)))
                        self.mav.send_landing_target(int(t_cap * 1e6), frame_id, send_vec, size)
                        last_send, sent = now, True
                        self.stats["sent"] += 1
                    row.update({"送出": int(sent), "送出frame": "BODY_FRD" if frame_id == MAV_FRAME_BODY_FRD else "LOCAL_FRD",
                                "送出x_m": send_vec[0], "送出y_m": send_vec[1], "送出z_m": send_vec[2],
                                "送出距離_m": float(np.linalg.norm(send_vec)),
                                "處理延遲_ms": (time.monotonic() - t_arr) * 1000})
                    if sent:
                        self._proc_delays.append((time.monotonic() - t_arr) * 1000)
                    if a is not None:
                        meas_ned = rot_zyx(a[0], a[1], a[2]) @ p_body
                        row.update(dict(zip(_cols("量測目標相對", ["N_m", "E_m", "D_m"]), meas_ned)))
                    self._truth_compare(row, t_cap, t_arr, det, row.get("量測目標相對N_m"))
                elif self.truth is not None:
                    self._truth_compare(row, t_cap, t_arr, det, None)
                self.logger.write(row)
                if self.cfg.show_preview:
                    self._draw(frame, det)
                if time.monotonic() - last_print > 2.0:
                    last_print = time.monotonic()
                    log.info("模式 %-7s 解鎖 %d | 影像 %d 偵測 %d 送出 %d | %s",
                             self.mav.mode, self.mav.armed, self.stats["frames"], self.stats["detections"],
                             self.stats["sent"],
                             f"ID {det.tag_id} cam=({det.p_cam[0]:+.3f},{det.p_cam[1]:+.3f},{det.p_cam[2]:.3f})"
                             if det.ok else "未偵測")
        finally:
            self.cam.stop()
            summary = self._finish()
            self.mav.stop()
        return summary

    def _row_base(self, seq, t_arr, t_cap, det: TagMeas) -> dict:
        mav = self.mav
        r = {"影像ID": seq, "取得時間_s": t_arr, "拍攝時間_s": t_cap, "模式": mav.mode, "解鎖": int(mav.armed),
             "落地狀態": mav.landed_state, "自動階段": self.auto.phase if self.auto else "",
             "偵測": int(det.ok), "使用ID": det.tag_id if det.ok else "", "偵測到ID": det.ids,
             "像素u": det.u, "像素v": det.v, "tag邊長_px": det.edge_px, "decision_margin": det.margin,
             "偵測耗時_ms": det.detect_ms}
        if mav.clock.offset is not None:
            r["飛控時間_s"] = t_cap - mav.clock.offset
        if det.ok:
            r.update(dict(zip(_cols("相機", ["x_m", "y_m", "z_m"]), det.p_cam)))
        p = mav.pos.at(t_cap)
        if p is not None:
            r.update(dict(zip(_cols("位置", ["N_m", "E_m", "D_m"]) + _cols("速度", ["N_mps", "E_mps", "D_mps"]), p)))
        a = mav.att.at(t_cap)
        if a is not None:
            r.update({"Roll_deg": math.degrees(a[0]), "Pitch_deg": math.degrees(a[1]), "Yaw_deg": math.degrees(a[2]),
                      "Roll速率_dps": math.degrees(a[3]), "Pitch速率_dps": math.degrees(a[4]),
                      "Yaw速率_dps": math.degrees(a[5])})
        pt = mav.latest.get("POSITION_TARGET_LOCAL_NED")
        if pt is not None:
            r.update({"目標位置N_m": pt.x, "目標位置E_m": pt.y, "目標位置D_m": pt.z,
                      "目標速度N_mps": pt.vx, "目標速度E_mps": pt.vy, "目標速度D_mps": pt.vz})
        nc = mav.latest.get("NAV_CONTROLLER_OUTPUT")
        if nc is not None:
            r.update({"目標Roll_deg": nc.nav_roll, "目標Pitch_deg": nc.nav_pitch, "目標航向_deg": nc.nav_bearing})
        rc = mav.latest.get("RC_CHANNELS")
        if rc is not None:
            r["RC_RSSI"] = rc.rssi
            for i in range(1, self.cfg.rc_channels_logged + 1):
                r[f"CH{i}_us"] = getattr(rc, f"chan{i}_raw", None)
        return r

    def _truth_compare(self, row: dict, t_cap: float, t_arr: float, det: TagMeas, meas_n) -> None:
        if self.truth is None or not self.truth.ready:
            return
        s = self.truth.at(t_cap)
        if s is None:
            return
        p, rr, pp, yy = s
        row.update(dict(zip(_cols("真值位置", ["N_m", "E_m", "D_m"]), p)))
        row.update({"真值Roll_deg": math.degrees(rr), "真值Pitch_deg": math.degrees(pp), "真值Yaw_deg": math.degrees(yy)})
        tr = self.truth.cam_to_marker_ned(t_cap)
        row.update(dict(zip(_cols("真值目標相對", ["N_m", "E_m", "D_m"]), tr)))
        mk = self.truth.marker_rel_home()
        row["真值與平台水平距離_m"] = float(math.hypot(p[0] - mk[0], p[1] - mk[1]))
        if "位置N_m" in row:
            row["EKF位置水平誤差_m"] = math.hypot(row["位置N_m"] - p[0], row["位置E_m"] - p[1])
        if meas_n is not None:
            meas = np.array([row["量測目標相對N_m"], row["量測目標相對E_m"], row["量測目標相對D_m"]])
            eh = float(np.hypot(*(meas[:2] - tr[:2])))
            ev = float(meas[2] - tr[2])
            row["量測水平誤差_m"], row["量測垂直誤差_m"] = eh, ev
            self.meas_err.append((float(tr[2]), eh, ev))
        if det.ok and len(self.latency_samples) < 20000:
            self.latency_samples.append((t_arr, det.p_cam.copy()))

    def _finish(self) -> dict:
        self.logger.close()
        summary = {"log_csv": self.logger.path, **self.stats}
        if self.meas_err:
            arr = np.array(self.meas_err)
            for name, sel in (("all", np.ones(len(arr), bool)), ("below_2m", arr[:, 0] < 2.0),
                              ("below_1m", arr[:, 0] < 1.0)):
                if sel.any():
                    eh = arr[sel, 1]
                    summary[f"meas_err_h_{name}"] = {"n": int(sel.sum()), "mean_m": float(eh.mean()),
                                                     "p95_m": float(np.percentile(eh, 95)), "max_m": float(eh.max())}
        if self.truth is not None and self.truth.ready:
            summary["marker_rel_home"] = self.truth.marker_rel_home().tolist()
            pdl = [r for r in self._proc_delays] if hasattr(self, "_proc_delays") else []
            if pdl:
                summary["proc_delay_ms_mean"] = float(np.mean(pdl))
        if self.auto is not None:
            summary["result"] = self.auto.result
        if self.truth is not None and self.truth.ready:
            try:   # 原始時間序列（離線分析延遲 / PLND_LAG 用）
                at, av = self.mav.att.snapshot()
                tt, tv = self.truth.mav.truth.snapshot()
                np.savez_compressed(self.logger.path.replace(".csv", "_raw.npz"), att_t=at, att_v=av, truth_t=tt,
                                    truth_v=tv, marker_rel_home=self.truth.marker_rel_home(),
                                    cam_t=np.array([s[0] for s in self.latency_samples]),
                                    cam_p=np.array([s[1] for s in self.latency_samples]),
                                    cam_offset=np.array(self.cfg.sim_cam_offset_body), C_bc=self.C_bc)
                from plnd_analyze import latency_from_npz
                est = latency_from_npz(self.logger.path.replace(".csv", "_raw.npz"))
                if est:
                    est.pop("curve")
                    summary["video_latency"] = est
                    lag = est["best_latency_ms"] + summary.get("proc_delay_ms_mean", 0.0)
                    summary["suggested_PLND_LAG_s"] = round(lag / 1000.0, 3)
                    log.info("影像延遲估測：%d ms（量測水平 RMS %.1f mm；不補償延遲時 %.1f mm）→ 建議 PLND_LAG ≈ %.3f s",
                             est["best_latency_ms"], est["rms_at_best_m"] * 1000, est["rms_at_0_m"] * 1000,
                             lag / 1000.0)
            except Exception as e:  # noqa: BLE001
                log.warning("raw npz / 延遲估測失敗：%s", e)
        if self.cfg.result_json:
            with open(self.cfg.result_json, "w", encoding="utf-8") as f:
                json.dump(summary, f, ensure_ascii=False, indent=1, default=lambda o: o.tolist()
                          if isinstance(o, np.ndarray) else str(o))
        return summary

    def _draw(self, gray, det: TagMeas) -> None:
        import cv2
        img = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        if det.ok:
            cv2.circle(img, (int(det.u), int(det.v)), 8, (0, 0, 255), 2)
            cv2.putText(img, f"ID{det.tag_id} {det.p_cam[0]:+.2f},{det.p_cam[1]:+.2f},{det.p_cam[2]:.2f}m",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        cv2.imshow("precland", img)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            self.stop_event.set()


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------


def build_config_from_args(argv=None) -> AppConfig:
    p = argparse.ArgumentParser(description="ArduPilot AprilTag precision landing（LANDING_TARGET）")
    p.add_argument("--mode", choices=("real", "sim"), default="real")
    p.add_argument("--conn", help="MAVLink 連線（sim 預設 tcp:127.0.0.1:5762＝SITL SERIAL1 直連，不經 MAVProxy（經 MAVProxy 轉送會增加延遲 / 抖動，且 MAVProxy 會覆寫串流頻率））")
    p.add_argument("--baud", type=int)
    p.add_argument("--rtsp", help="real：RTSP URL")
    p.add_argument("--camera", choices=("udp", "rtsp", "device"))
    p.add_argument("--cam-profile", help="USB 相機校正檔（camera_profiles/*.json）：解析度、內參、畸變、延遲")
    p.add_argument("--device", help="USB 相機 /dev/videoN（預設依 profile 內的名稱尋找）")
    p.add_argument("--exposure", type=float, help="USB 相機手動曝光（100 µs 單位，預設自動）")
    p.add_argument("--cam-yaw", type=float, default=0.0,
                   help="下視相機安裝方向：影像上方相對機頭的順時針角度 deg（0 = 影像上方朝機頭）")
    p.add_argument("--tag-sizes", help="實際 tag 邊長 m，例如 0:0.75,1:0.15,2:0.03（要以尺實量印出來的 tag）")
    p.add_argument("--udp-port", type=int)
    p.add_argument("--fov-h", type=float, help="水平 FOV（deg）")
    p.add_argument("--fx", type=float); p.add_argument("--fy", type=float)
    p.add_argument("--cx", type=float); p.add_argument("--cy", type=float)
    p.add_argument("--video-latency", type=float, help="影像延遲 s（拍攝→取得）")
    p.add_argument("--frame", choices=("body", "local"), help="LANDING_TARGET 座標系")
    p.add_argument("--decimate", type=float, help="AprilTag quad_decimate")
    p.add_argument("--preview", action="store_true")
    p.add_argument("--log-dir")
    p.add_argument("--timeout", type=float, help="整體執行時間上限 s")
    p.add_argument("--sim-auto", action="store_true", help="sim：自動起飛 + 精準降落 + 誤差報告")
    p.add_argument("--takeoff-alt", type=float)
    p.add_argument("--approach-err", type=float, help="sim-auto：航點相對平台的隨機水平誤差上限 m")
    p.add_argument("--approach-yaw", type=float, help="sim-auto：降落時機頭方向 deg（預設隨機）")
    p.add_argument("--seed", type=int)
    p.add_argument("--result-json")
    a = p.parse_args(argv)

    cfg = replace(AppConfig(mode=a.mode), **PRESETS[a.mode])
    if a.cam_profile:
        cfg = replace(cfg, **load_camera_profile(a.cam_profile))
    if a.cam_yaw:
        cy_, sy_ = math.cos(math.radians(a.cam_yaw)), math.sin(math.radians(a.cam_yaw))
        C = np.array([[cy_, -sy_, 0], [sy_, cy_, 0], [0, 0, 1]]) @ np.array(cfg.cam_to_body, float)
        cfg = replace(cfg, cam_to_body=tuple(tuple(float(v) for v in r) for r in C))
    if a.tag_sizes:
        cfg = replace(cfg, tag_sizes={int(k): float(v) for k, v in (x.split(":") for x in a.tag_sizes.split(","))})
    upd = {"device": a.device, "cam_exposure": a.exposure,
           "conn": a.conn, "baud": a.baud, "rtsp_url": a.rtsp, "camera_source": a.camera, "udp_port": a.udp_port,
           "fov_horizontal_deg": a.fov_h, "fx": a.fx, "fy": a.fy, "cx": a.cx, "cy": a.cy,
           "video_latency_s": a.video_latency, "send_frame": a.frame, "quad_decimate": a.decimate,
           "log_dir": a.log_dir, "run_timeout_s": a.timeout, "sim_takeoff_alt_m": a.takeoff_alt,
           "sim_approach_err_m": a.approach_err, "sim_approach_yaw_deg": a.approach_yaw, "sim_seed": a.seed,
           "result_json": a.result_json}
    cfg = replace(cfg, **{k: v for k, v in upd.items() if v is not None})
    if a.preview:
        cfg = replace(cfg, show_preview=True)
    if a.sim_auto:
        if cfg.mode != "sim":
            raise SystemExit("--sim-auto 只能在 --mode sim 使用")
        cfg = replace(cfg, sim_auto=True)
    return cfg


def main(argv=None) -> int:
    cfg = build_config_from_args(argv)
    os.makedirs(cfg.log_dir, exist_ok=True)
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s",
                        handlers=[logging.StreamHandler(sys.stdout)])
    app = PrecisionLandApp(cfg)
    summary = app.run()
    res = summary.get("result") or {}
    if cfg.sim_auto:
        return 0 if res.get("landed") else 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
