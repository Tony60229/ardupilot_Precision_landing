#!/usr/bin/env python3
"""
ArduPilot 視覺 Precision Landing 批次測試（Gazebo Classic + SITL，真值比對）

每一回合：
  1. 產生世界檔：降落平台隨機位置 / 方向
  2. 啟動 gzserver（無畫面）+ ArduCopter SITL（sim_vehicle.py --no-mavproxy，每回合清 EEPROM）
  3. ap_precland.py --mode sim --sim-auto：解鎖 → 起飛 → 飛到平台附近（隨機航點誤差 / 機頭方向）→ LAND
  4. 關閉模擬器；分析 DataFlash（PL 估測 vs SITL 真值、觸地速度、傾角）與影像延遲
全部結束輸出 summary.csv / summary.json 與統計。

  python3 plnd_batch.py --runs 10
  python3 plnd_batch.py --runs 20 --pad-range 3 --approach-err 1.5 --seed 7
  python3 plnd_batch.py --runs 5 --param PLND_LAG=0.2 --param LAND_SPD_MS=0.3 --tag lag200

輸出：logs/batch_YYYYmmdd_HHMMSS[_tag]/
  summary.csv / summary.json        每回合一列 + 統計
  run_NN/world.world  gz.log  sitl.log  plnd.log  result.json  analysis.json
  run_NN/plnd_*.csv / .xlsx / _raw.npz   每幀紀錄
  run_NN/logs/*.BIN                       ArduPilot DataFlash

注意：執行前不可有其他 gzserver / arducopter 在跑（會占用 11345 / 5760 / 9002 / 5600）
"""

from __future__ import annotations

import argparse
import csv
import datetime
import json
import math
import os
import random
import signal
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import plnd_analyze  # noqa: E402

BASE_WORLD = os.path.join(HERE, "worlds", "apriltag_landing.world")
PARAM_FILE = os.path.join(HERE, "params", "precland_mavlink.parm")
ARDUPILOT = os.path.expanduser(os.environ.get("ARDUPILOT_DIR", "~/ardupilot"))
SIM_VEHICLE = os.path.join(ARDUPILOT, "Tools", "autotest", "sim_vehicle.py")


def running_sims():
    out = []
    for name in ("gzserver", "gzclient", "arducopter"):
        r = subprocess.run(["pgrep", "-x", name], capture_output=True, text=True)
        out += [f"{name}:{p}" for p in r.stdout.split()]
    out += [f"mavproxy:{p}" for p in mavproxy_pids()]
    return out


def mavproxy_pids():
    r = subprocess.run(["ps", "-eo", "pid,args"], capture_output=True, text=True)
    return [ln.split()[0] for ln in r.stdout.splitlines()[1:] if "mavproxy.py" in ln and "--master" in ln]


def make_world(path: str, pad_xy, pad_yaw: float) -> None:
    tree = ET.parse(BASE_WORLD)
    for inc in tree.getroot().iter("include"):
        name = inc.find("name")
        if name is not None and name.text == "landing_pad":
            inc.find("pose").text = f"{pad_xy[0]:.4f} {pad_xy[1]:.4f} 0 0 0 {pad_yaw:.5f}"
    tree.write(path)


class Proc:
    def __init__(self, cmd, cwd, log_path, env=None):
        self.log = open(log_path, "w")
        self.p = subprocess.Popen(cmd, cwd=cwd, env=env, stdout=self.log, stderr=subprocess.STDOUT,
                                  stdin=subprocess.DEVNULL, start_new_session=True)

    def alive(self):
        return self.p.poll() is None

    def stop(self):
        for sig, wait in ((signal.SIGINT, 5), (signal.SIGTERM, 5), (signal.SIGKILL, 5)):
            if self.p.poll() is not None:
                break
            try:
                os.killpg(self.p.pid, sig)
            except ProcessLookupError:
                break
            try:
                self.p.wait(timeout=wait)
            except subprocess.TimeoutExpired:
                pass
        self.log.close()


def wait_gazebo(timeout=90) -> bool:
    t0 = time.time()
    while time.time() - t0 < timeout:
        try:
            r = subprocess.run(["gz", "model", "-m", "landing_pad", "-p"], capture_output=True, text=True, timeout=15)
            if r.returncode == 0 and len(r.stdout.split()) >= 6:
                return True
        except subprocess.TimeoutExpired:
            pass
        time.sleep(1.0)
    return False


def stats(vals):
    v = np.array([x for x in vals if x is not None and math.isfinite(x)])
    if len(v) == 0:
        return {}
    return {"n": int(len(v)), "mean": float(v.mean()), "median": float(np.median(v)), "std": float(v.std()),
            "p95": float(np.percentile(v, 95)), "max": float(v.max()), "min": float(v.min())}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--runs", type=int, default=10)
    ap.add_argument("--seed", type=int)
    ap.add_argument("--pad-center", type=float, nargs=2, default=(5.0, 0.0), help="平台中心（Gazebo x y）")
    ap.add_argument("--pad-range", type=float, default=2.0, help="平台位置隨機範圍 ±m")
    ap.add_argument("--pad-yaw", choices=("random", "zero"), default="random")
    ap.add_argument("--approach-err", type=float, default=1.5, help="航點相對平台的隨機水平誤差上限 m")
    ap.add_argument("--takeoff-alt", type=float, default=6.0)
    ap.add_argument("--frame", choices=("body", "local"), default="body")
    ap.add_argument("--video-latency", type=float, default=None, help="ap_precland --video-latency（記錄用）")
    ap.add_argument("--param", action="append", default=[], help="額外飛控參數 NAME=VALUE（可多次）")
    ap.add_argument("--mavproxy", action="store_true", help="同時啟動 MAVProxy（與手動操作相同）；companion 預設直連 SERIAL1 5762")
    ap.add_argument("--conn", help="--mavproxy 時 companion 的連線（例：udpin:0.0.0.0:14550 測試經 MAVProxy 轉送）")
    ap.add_argument("--bare", action="store_true",
                    help="不帶參數檔啟動 SITL（PLND_TYPE = 3），驗證 ap_precland 會自動修正")
    ap.add_argument("--load", type=int, default=0, help="額外 N 個 CPU 滿載行程（模擬 Gazebo 畫面 / 地圖負載）")
    ap.add_argument("--run-timeout", type=float, default=300.0)
    ap.add_argument("--success-radius", type=float, default=0.05, help="成功判定：降落誤差 < 此值（m）")
    ap.add_argument("--tag", default="")
    ap.add_argument("--out", default=os.path.join(HERE, "logs"))
    args = ap.parse_args()

    busy = running_sims()
    if busy:
        sys.exit(f"已有模擬器在執行，請先關閉：{busy}")

    seed = args.seed if args.seed is not None else random.randrange(1 << 30)
    rng = random.Random(seed)
    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    out_dir = os.path.join(args.out, f"batch_{stamp}" + (f"_{args.tag}" if args.tag else ""))
    os.makedirs(out_dir)
    extra_param = os.path.join(out_dir, "extra.parm")
    with open(extra_param, "w") as f:
        for kv in args.param:
            k, v = kv.split("=", 1)
            f.write(f"{k.strip()} {v.strip()}\n")
    with open(os.path.join(out_dir, "batch.json"), "w") as f:
        json.dump({"seed": seed, "args": vars(args)}, f, indent=1)
    print(f"輸出：{out_dir}（seed {seed}）")

    env = dict(os.environ)
    rows = []
    for k in range(1, args.runs + 1):
        rd = os.path.join(out_dir, f"run_{k:02d}")
        os.makedirs(rd)
        cx, cy = args.pad_center
        pad = (cx + rng.uniform(-args.pad_range, args.pad_range), cy + rng.uniform(-args.pad_range, args.pad_range))
        pad_yaw = rng.uniform(-math.pi, math.pi) if args.pad_yaw == "random" else 0.0
        run_seed = rng.randrange(1 << 30)
        world = os.path.join(rd, "world.world")
        make_world(world, pad, pad_yaw)
        print(f"\n=== run {k}/{args.runs}：平台 ({pad[0]:.2f}, {pad[1]:.2f}) yaw {math.degrees(pad_yaw):.0f}° ===",
              flush=True)
        row = {"run": k, "pad_x": pad[0], "pad_y": pad[1], "pad_yaw_deg": math.degrees(pad_yaw)}
        gz = sitl = None
        burners = [Proc([sys.executable, "-c", "while True: pass"], rd, os.devnull) for _ in range(args.load)]
        try:
            genv = dict(env, HEADLESS="1", WORLD=world)
            gz = Proc([os.path.join(HERE, "run_sim.sh")], rd, os.path.join(rd, "gz.log"), genv)
            if not wait_gazebo():
                raise RuntimeError("Gazebo 啟動逾時")
            sv = [sys.executable, SIM_VEHICLE, "-v", "ArduCopter", "-f", "gazebo-iris", "-N", "-w"]
            if not args.bare:
                sv += [f"--add-param-file={PARAM_FILE}", f"--add-param-file={extra_param}"]
            if args.mavproxy:      # 與手動操作相同：經 MAVProxy 轉送（udp 14550）
                sv += ["--mavproxy-args=--daemon --non-interactive"]
                conn = args.conn or "tcp:127.0.0.1:5762"
            else:
                sv += ["--no-mavproxy"]
                conn = "tcp:127.0.0.1:5760"
            sitl = Proc(sv, rd, os.path.join(rd, "sitl.log"), env)
            cmd = [sys.executable, "-u", os.path.join(HERE, "ap_precland.py"), "--mode", "sim", "--sim-auto",
                   "--conn", conn, "--log-dir", rd, "--result-json", os.path.join(rd, "result.json"),
                   "--seed", str(run_seed), "--approach-err", str(args.approach_err),
                   "--takeoff-alt", str(args.takeoff_alt), "--frame", args.frame, "--timeout", str(args.run_timeout)]
            if args.video_latency is not None:
                cmd += ["--video-latency", str(args.video_latency)]
            time.sleep(3.0)
            with open(os.path.join(rd, "plnd.log"), "w") as lf:
                try:
                    subprocess.run(cmd, cwd=rd, stdout=lf, stderr=subprocess.STDOUT, timeout=args.run_timeout + 60)
                except subprocess.TimeoutExpired:
                    row["note"] = "timeout"
        except Exception as e:  # noqa: BLE001
            row["note"] = str(e)
            print("  錯誤：", e)
        finally:
            for bp in burners:
                bp.stop()
            if sitl:
                sitl.stop()
            for pid in mavproxy_pids():        # MAVProxy 可能脫離 process group
                try:
                    os.kill(int(pid), signal.SIGTERM)
                except ProcessLookupError:
                    pass
            if gz:
                gz.stop()
            time.sleep(2.0)

        try:
            with open(os.path.join(rd, "result.json"), encoding="utf-8") as f:
                summ = json.load(f)
        except Exception:  # noqa: BLE001
            summ = {}
        res = summ.get("result") or {}
        row.update({"landed": res.get("landed"), "on_pad": res.get("on_pad"), "error_mm": None,
                    "err_pad_x_mm": None, "err_pad_y_mm": None, "land_s": res.get("land_duration_s"),
                    "approach_offset_m": res.get("approach_offset_m"), "approach_yaw_deg": res.get("approach_yaw_deg"),
                    "target_lost": res.get("target_lost_count"), "retries": res.get("retry_count"),
                    "frames": summ.get("frames"), "detections": summ.get("detections"),
                    "max_backlog_s": summ.get("max_backlog_s"), "stale_frames": summ.get("stale_frames", 0)})
        if res.get("error_m") is not None:
            row["error_mm"] = res["error_m"] * 1000
            row["err_pad_x_mm"] = res["err_pad_x_m"] * 1000
            row["err_pad_y_mm"] = res["err_pad_y_m"] * 1000
        vl = summ.get("video_latency") or {}
        row["video_latency_ms"] = vl.get("best_latency_ms")
        row["meas_rms_mm"] = vl.get("rms_at_best_m", math.nan) * 1000 if vl else None
        row["suggested_lag_s"] = summ.get("suggested_PLND_LAG_s")
        for key in ("all", "below_1m"):
            m = summ.get(f"meas_err_h_{key}") or {}
            row[f"meas_err_{key}_mean_mm"] = m.get("mean_m", math.nan) * 1000 if m else None
        binf = plnd_analyze.latest_bin(rd)
        ana = {}
        if binf and summ.get("marker_rel_home"):
            try:
                ana = plnd_analyze.analyze_bin(binf, summ["marker_rel_home"])
            except Exception as e:  # noqa: BLE001
                ana = {"error": str(e)}
        with open(os.path.join(rd, "analysis.json"), "w") as f:
            json.dump(ana, f, indent=1, ensure_ascii=False)
        td = ana.get("touchdown") or {}
        row.update({"td_vd_mps": td.get("vd_mps"), "td_vh_mps": td.get("vh_mps"), "td_err_mm": td.get("horiz_err_mm"),
                    "slide_mm": ana.get("slide_after_touchdown_mm"), "max_tilt_2m_deg": ana.get("max_tilt_below_2m_deg"),
                    "err_at_1m_mm": ana.get("horiz_err_at_1m_mm"), "err_at_0.5m_mm": ana.get("horiz_err_at_0.5m_mm"),
                    "max_err_below_1m_mm": ana.get("max_horiz_err_below_1m_mm")})
        for band, v in (ana.get("pl_est_err") or {}).items():
            row[f"pl_err_{band}_mean_mm"] = v["mean_mm"]
        row["success"] = int(bool(row.get("landed")) and bool(row.get("on_pad")) and row["error_mm"] is not None
                             and row["error_mm"] < args.success_radius * 1000)
        rows.append(row)
        print(f"  落地={row['landed']} 在平台上={row['on_pad']} 誤差={row['error_mm'] if row['error_mm'] is None else round(row['error_mm'], 1)} mm "
              f"觸地 vd={td.get('vd_mps')} 影像延遲={row['video_latency_ms']} ms", flush=True)

        cols = []
        for r in rows:
            cols += [c for c in r if c not in cols]
        with open(os.path.join(out_dir, "summary.csv"), "w", newline="", encoding="utf-8-sig") as f:
            w = csv.DictWriter(f, fieldnames=cols)
            w.writeheader()
            w.writerows(rows)

    landed = [r for r in rows if r.get("landed") and r.get("on_pad")]
    agg = {"runs": len(rows), "landed_on_pad": len(landed),
           "success": sum(r["success"] for r in rows), "success_radius_m": args.success_radius,
           "error_mm": stats([r["error_mm"] for r in landed]),
           "td_vd_mps": stats([r.get("td_vd_mps") for r in landed]),
           "max_tilt_2m_deg": stats([r.get("max_tilt_2m_deg") for r in landed]),
           "video_latency_ms": stats([r.get("video_latency_ms") for r in rows]),
           "suggested_lag_s": stats([r.get("suggested_lag_s") for r in rows]),
           "target_lost": stats([r.get("target_lost") for r in rows])}
    for thr in (20, 30, 50, 100):
        agg[f"within_{thr}mm"] = sum(1 for r in landed if r["error_mm"] is not None and r["error_mm"] < thr)
    with open(os.path.join(out_dir, "summary.json"), "w") as f:
        json.dump({"seed": seed, "args": vars(args), "aggregate": agg, "runs": rows}, f, indent=1, ensure_ascii=False)
    e = agg["error_mm"]
    print(f"\n==== 批次結果：{len(rows)} 回合，落在平台上 {len(landed)}，誤差 < {args.success_radius * 1000:.0f} mm：{agg['success']} ====")
    if e:
        print(f"降落誤差 mm：mean {e['mean']:.1f}  median {e['median']:.1f}  p95 {e['p95']:.1f}  max {e['max']:.1f}")
        print("  " + "  ".join(f"<{t}mm: {agg[f'within_{t}mm']}" for t in (20, 30, 50, 100)))
    print(f"彙整：{os.path.join(out_dir, 'summary.csv')}")


if __name__ == "__main__":
    main()
