# ArduPilot 視覺導引 Precision Landing（AprilTag + Gazebo Classic）

companion 程式辨識平台上的 AprilTag，以 MAVLink `LANDING_TARGET` 送給 ArduCopter，
由 AC_PrecLand（`PLND_TYPE=1`）在 LAND / RTL 時精準降落。架構參考 PX4 VTE 專案
（`~/work_space/vte_land`），並以 Gazebo 真值做多回合驗證。

```
ardupilot_Precision_landing/
├── ap_precland.py                 # companion 主程式（sim / real）
├── cam_test.py                    # USB 相機實測：延遲 / 漂移 / 內參 / 偵測 / 完整流程
├── camera_profiles/               # USB 相機校正檔（解析度、內參、畸變）
├── plnd_batch.py                  # Gazebo + SITL 自動批次測試（真值比對）
├── plnd_analyze.py                # 離線分析：影像延遲 / PLND_LAG、PL 估測誤差、觸地穩定性
├── params/precland_mavlink.parm   # 飛控參數（已調校）
├── run_sim.sh                     # 啟動 Gazebo（WORLD= 可指定世界檔）
├── worlds/apriltag_landing.world  # 草地 + 降落平台 + iris_downcam
├── models/apriltag_landing_pad/   # 1.5x1.5x0.4 m 平台，頂面 tagCustom48h12 nested（ID 0/1/2）
├── models/iris_downcam/           # ArduPilot iris + 固定下視相機（H.264 → udp 5600）
└── logs/                          # 每幀 CSV / xlsx / raw npz、批次結果
```

## 1. 快速開始（模擬）

```bash
# 終端機 1：Gazebo
./run_sim.sh

# 終端機 2：SITL + MAVProxy（參數由 ap_precland 連上後自動檢查；PLND_TYPE 不對時會寫入並重開飛控一次）
cd ~/ardupilot/ArduCopter
../Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris --console --map

# 終端機 3：companion（預設直連 SITL SERIAL1 = tcp:127.0.0.1:5762，與實機 companion 接飛控 TELEM 口相同）
cd ~/work_space/ardupilot_Precision_landing
python3 ap_precland.py --mode sim               # 只送目標：在 MAVProxy 起飛到平台上方後 `mode LAND`
python3 ap_precland.py --mode sim --sim-auto    # 全自動：起飛 → 飛到平台附近 → LAND → 量測誤差
```

**不要**讓 companion 經 MAVProxy 轉送（`--conn udpin:0.0.0.0:14550`）：MAVProxy 是 Python 轉送，
在 2 核 VM 上會增加 LANDING_TARGET 延遲 / 抖動（實測降落誤差 mean 20.5 → 52 mm，6 次中 1 次發散擺盪），
而且 MAVProxy 會把串流頻率重設為 4 Hz。直連 5762 不受影響，MAVProxy / QGC 照常用 14550。

**LAND 變成原地降落：** 代表飛控 `PLND_TYPE` 不是 1，LANDING_TARGET 會被忽略。`--add-param-file` 不會存進 EEPROM，
所以 `ap_precland.py --mode sim` 連上時會檢查：未解鎖就寫入參數檔並重開飛控一次（log 出現「飛控已重開」後再解鎖），已解鎖則只報錯。
請先開 companion 再解鎖。real 模式只報錯，不改飛控參數。

**SITL 開機後約 35~45 秒才能解鎖**：要等 `EKF3 IMU0 is using GPS`（ap_precland 會印出「飛控已可解鎖」提示）；
在此之前 arm 會被拒絕（`AHRS: waiting for home` / `Need Position Estimate`）。`takeoff` 只能在 GUIDED 模式使用，
STABILIZE 下解鎖後不推油門會自動上鎖。

手動流程（MAVProxy）：`mode guided` → `arm throttle` → `takeoff 6` → 飛到平台上方附近（平台在起飛點北方 5 m）
→ 出現 `PrecLand: Init Complete` 後 `mode LAND`。

批次測試（不需要先開模擬器；會自行啟動 / 關閉 gzserver 與 SITL）：
```bash
python3 plnd_batch.py --runs 20                                   # 使用 params/ 的參數
python3 plnd_batch.py --runs 5 --param PSC_NE_POS_P=1.0 --tag cmp # 臨時覆寫參數比較
```
每回合隨機：平台位置（±2 m）與方向、航點相對平台的誤差（≤1.5 m，模擬 GPS 導航誤差）、降落機頭方向。

## 2. 運作方式

1. **偵測**：`pupil-apriltags`（tagCustom48h12）。三個同中心 tag，取影像中最大的那個解 PnP，
   平移量 × 該 ID 的 tag_size（0.75 / 0.15 / 0.03 m）→ 相機到平台中心的 3D 向量。
   高空用 ID 0，接近時換 ID 1，最後換 ID 2（切換高度依解析度而定），觸地時仍可偵測。
   pupil-apriltags 對 48h12 固定用 2-bit 錯誤更正會吃 ~4.5 GB 記憶體，程式改成 1-bit（~150 MB，48h12 最小漢明距離 12，誤判風險極低）。
2. **送出**：`LANDING_TARGET`，`frame = MAV_FRAME_BODY_FRD`，`position_valid = 1`，x/y/z + distance。
   飛控直接用視覺距離，不需要測距儀，平台高度（0.4 m）不影響；`PLND_ALT_MIN/MAX = 0` 關閉高度範圍檢查。
   相機 → 機體：下視、影像上方 = 機頭，`body = (-y_cam, x_cam, z_cam)`（`AppConfig.cam_to_body` 可改）。
3. **舊影像保護**：由每個 RTP 封包的時間戳量測影像積壓（到達時間 − 拍攝時間，相對本次連線最小值），
   超過 `max_video_backlog_s`（0.15 s）就停止送 `LANDING_TARGET` 並警告。曾發生 x264 軟體編碼跟不上
   （720p30 + Gazebo 畫面 + 地圖，2 核 VM），影像積壓到 10 s，無人機依舊影像飛離 44 m；
   現改為 960x540 @20 Hz 並加上此保護。積壓值記錄在 CSV 的 `影像積壓_s`。
4. **飛控**：AC_PrecLand Kalman filter 估測目標相對位置，LAND 模式下水平追蹤目標、距目標 2 m 內減速。

## 3. 調校結果與原因（Gazebo 真值）

| 參數 | 預設 | 本專案 | 原因 |
|---|---|---|---|
| `PLND_TYPE` | 3（gazebo-iris.parm） | 1 | MAVLink companion |
| `PLND_LAG` | 0.02 | 0.05 | 真值估測：一般負載 ~35 ms、高負載 ~90 ms（影像 + 處理），取偏一般負載的值 |
| `PLND_OPTIONS` | 0 | 1 | Moving target：目標速度 = EKF 速度 + 視覺相對速度，抵銷 EKF 速度誤差 |
| `PSC_NE_POS_P` | 1.0 | 2.0 | 讓視覺位置誤差更快修正；3.0 開始出現擺盪 |
| `LAND_SPD_MS` | 0.5 | 0.3 | 最後下降更慢，有時間收斂；觸地 ~0.3 m/s |
| `PLND_XY_DIST_MAX` | 2.5 | 1.0 | 水平誤差 > 1 m 先不下降 |

**主要誤差來源不是視覺**：低空時視覺量測與飛控 PL 估測都在 1~2 cm 內；殘餘誤差來自 EKF 水平速度誤差
（姿態估測約 0.5~0.9° 誤差 → 0.1~0.16 m/s 速度偏差），預設位置增益下會造成約 10 cm 穩態偏移。
`PLND_OPTIONS=1` + `PSC_NE_POS_P=2` 後顯著改善。

各設定降落誤差（下方 5 回合者使用相同隨機種子 33）：

| 設定 | 回合 | mean | max |
|---|---|---|---|
| 只調 LAG 0.08（無 moving target、預設增益、LAND 0.5） | 3（種子 21） | 71 mm | 113 mm |
| Moving target + LAG 0.08 | 3（種子 21） | 22 mm | 35 mm |
| Moving target + LAG 0.08 | 5 | 53 mm | 76 mm |
| + LAND 0.3 m/s | 5 | 33 mm | 65 mm |
| + LAG 0.05 + `PSC_NE_POS_P=2.0`（採用） | 5 | **20.5 mm** | **33 mm** |
| 同上但關閉 moving target | 5 | 28 mm | 84 mm |
| `PSC_NE_POS_P=3.0` | 5 | 22.7 mm | 31 mm（傾角 3~4°、低空擺盪變大） |
| 採用設定，MAVProxy 執行中、companion 經 MAVProxy 轉送（udp 14550） | 5 | 52 mm | 78 mm（另一次手動測試發散擺盪） |
| 採用設定，MAVProxy 執行中、companion 直連 SERIAL1（tcp 5762，預設） | 5 | **24.0 mm** | **34.7 mm** |

以上比較都是舊相機（1280x720 @30 Hz）。目前相機設定的驗證結果在下方第一個表。

**目前設定驗證（960x540 @20 Hz，MAVProxy 執行中，companion 直連 5762）**

| 項目 | 一般：20 回合，6 m，偏移 ≤ 1.5 m | 高負載：8 回合，18 m，偏移 ≤ 5 m，+2 個 CPU 滿載行程 |
|---|---|---|
| 落在平台上 | **20 / 20** | **8 / 8** |
| 降落水平誤差 | median **23 mm**、mean 25.7 mm、p95 51 mm、max 58 mm | median 60 mm、mean 57 mm、p95 98 mm、max 104 mm |
| 分布 | < 2 cm：7；< 3 cm：14；< 5 cm：18 | < 2 cm：1；< 5 cm：3；< 10 cm：7 |
| 觸地速度（垂直 / 水平） | 0.29 / 0.02 m/s | 0.30 / 0.03 m/s |
| 最後 2 m 最大傾角 | 平均 1.1°，max 1.5° | 平均 5.7°，max 13° |
| 影像積壓 max | 0.05 s（未擋任何幀） | 0.48 s（平均每回合擋 27 幀） |
| 目標遺失 / 重試 | 0 / 0 | 0 / 0 |
| 視覺量測誤差（1 m 以下） | mean 9.7 mm | mean 10.8 mm |
| 建議 PLND_LAG | 0.035 s | 0.091 s |

資料：`logs/batch_20261104_163842_cam540_final20/`、`logs/batch_20261104_171120_cam540_stress8/`

**舊相機（1280x720 @30 Hz）驗證：20 回合（種子 2026，`logs/batch_20261104_131835_final20/`，無 MAVProxy）**

| 項目 | 結果 |
|---|---|
| 落在平台上 | **20 / 20** |
| 降落水平誤差 | median **27 mm**、mean 35.5 mm、p95 73 mm、max 127 mm |
| 分布 | < 20 mm：8；< 30 mm：11；< 50 mm：14；< 100 mm：19 |
| 觸地垂直速度 | 0.29 m/s（0.27~0.32） |
| 觸地水平速度 | 0.02 m/s（max 0.06） |
| 最後 2 m 最大傾角 | 1.4°（max 2.3°） |
| 觸地後滑移 | ~1 mm |
| 目標遺失 / 重試 | 1 次（觸地後）/ 0 |
| 視覺量測誤差（1 m 以下） | mean 11 mm |
| 飛控 PL 估測誤差 | 0~0.5 m：9 mm；0.5~1 m：8 mm；1~2 m：14 mm |
| 影像延遲（估測） | 34 ms → `PLND_LAG` 0.05 s |

**限制**：視覺量測與飛控估測都達公分級，但 ArduPilot LAND 的水平控制在最後 0.5 m 仍受 EKF 速度誤差影響，
目前設定一般條件 20 回合有 2 回合 > 5 cm（最大 58 mm），高負載 8 回合有 5 回合 > 5 cm（最大 104 mm）。
若要穩定 < 3 cm，下一步可考慮：
1. companion 接管最後 1 m（GUIDED + 視覺速度指令，對準後才下降，類似 PX4 offboard 做法）
2. 調 EKF（`EK3_VELNE_M_NSE` 等）降低速度偏差（需實機 GPS 條件驗證）
3. 加裝測距儀，讓 LAND 的高度判斷不依賴氣壓

## 4. 紀錄

`logs/plnd_YYYYmmdd_HHMMSS.csv`（每幀一列，時間同步於該影像的拍攝時間）：
影像 ID、偵測（使用 ID、像素、tag 邊長、相機座標）、送出的 LANDING_TARGET、量測目標相對 NED、
位置 / 目標位置、速度 / 目標速度、姿態 / 目標姿態 / 角速度、RC 通道、
（sim）真值位置 / 姿態 / 目標相對位置、量測水平 / 垂直誤差、EKF 位置誤差。
結束另存 `.xlsx` 與 `_raw.npz`（離線算延遲）。

批次每回合：`world.world gz.log sitl.log plnd.log result.json analysis.json logs/*.BIN`。
`analysis.json`：PL 估測誤差（依高度）、觸地垂直 / 水平速度、觸地後滑移、最後 2 m 最大傾角、1 m / 0.5 m 時的水平誤差。

## 5. 實機（USB 相機 UHD4K）

```bash
python3 ap_precland.py --mode real --conn /dev/ttyACM0 --baud 921600 \
    --cam-profile camera_profiles/uhd4k_1280x720.json \
    --tag-sizes 0:0.75,1:0.15,2:0.03 --cam-yaw 0
```

程式在 real 模式只送 `LANDING_TARGET`，不解鎖、不切模式；由飛手切 LAND / RTL。

**首飛前一定要做：**

1. 在 GCS 載入 [params/precland_real_uhd4k.parm](params/precland_real_uhd4k.parm)，量好 `PLND_CAM_POS_X/Y/Z`，重開機
2. `--tag-sizes` 填**用尺量印出來的** tag 邊長（距離與水平位置都與它成正比）；ID 0/1/2 邊長比例為全圖寬的 0.5 / 0.1 / 0.02
3. 相機要鎖死（見下方漂移）；影像上方朝機頭，否則用 `--cam-yaw`（順時針 deg）
4. 不解鎖檢查方向：tag 放機頭前方 → log 的 x 為正；放右方 → y 為正。方向錯誤會讓飛機往反方向追
5. 換 companion 後重跑 `cam_test.py e2e`，依結果重設 `PLND_LAG`（本表數據來自 VMware 虛擬機）
6. 換解析度要重新校正：內參只對校正的解析度有效，`UsbCamera` 拿到不同解析度會拒絕使用

`PSC_NE_POS_P=2`、`PLND_OPTIONS=1` 是 SITL 調出來的，刻意沒放進實機參數檔，要在實機上逐步驗證。

### 相機實測（cam_test.py，相機固定對著螢幕）

相機：SunplusIT UHD4K，USB 2.0，經 VMware USB 直通。結果在 `logs/camtest/`。

| 項目 | 結果 |
|---|---|
| 幀率 | 320x240~3840x2160 全部 MJPEG 30.1 fps，週期 p99 < 40 ms，100 次翻轉不漏看 |
| 螢幕 → 解出 ArUco | 1280x720 自動曝光中位約 90~99 ms；解析度影響不明顯；手動曝光 2~5 ms 約 82 ms，但在完整流程中沒有明顯差異（95~98 ms），所以預設自動曝光 |
| 螢幕 → 飛控收到（完整 ap_precland，3 輪 x 100 次） | 中位 97 ms（96.5~97.2），p95 118，max 141，29 Hz |
| 同上，2 個 CPU 滿載 | 中位 185 ms，15 Hz（AprilTag 8 → 65 ms） |
| 拍攝 → 飛控收到（推算） | 30~63 ms：97 減幀相位 16.7 減螢幕延遲 17~50（螢幕延遲無法單獨量測，是假設值） |
| 偵測（1280x720、decimate 2） | 8~9 ms；ID0 邊長 14.7 px 仍 100%，再小一級就 0%（此時螢幕上 tag 僅 38 px，顯示本身已失真，不能當相機極限）；靜止抖動約 0.02 px |
| 水平定位（已知位移，26 點，相機傾斜 10°） | 校正後 2.5 mm / 每 m 高，最大 4.3 mm；未校正 3.2 mm，最大 5.8 mm。此測試驗不到固定偏移（含主點） |
| 內參（59 視角聯合） | f ≈ 760（f 未定案，實際可能在 760~815），主點 (648.1, 369.4)，k1 0.072，RMS 0.13 px |

**內參的可信度：** 主點與畸變可用，**f 還沒定案**（校正檔標記 `f_reliable: false`）。

| 資料 | f | 主點 |
|---|---|---|
| 傾斜 2.8°（單一姿態） | 1018 | (648.3, 370.2) |
| 傾斜 6°（單一姿態） | 797 | (647.2, 372.6) |
| 傾斜 10°（單一姿態） | 797 | (652.1, 375.3) |
| 6° + 10° 聯合（採用） | 760 | (648.1, 369.4) |

相機和螢幕同一姿態時所有點都在同一平面，單一姿態的 f 可差 25%、主點差 5 px，遠大於統計誤差，所以 `calib` 改為各視角聯合求解。
聯合解每次去掉一輪仍是 758~764，但姿態差只有 3.8°，且只有這兩組。
影像下緣約 10%（最初姿態）看到的是螢幕邊框。f 錯 ±20% 時水平定位只從 2.5 變 3.0 mm/m，距離 Z 則等比例偏差。
相機鎖好後執行 `cam_test.py calib --multi`（8 個視角，傾斜 10~30°、方向不同）取得完整內參。

**主點的意義：** 主點比影像中心偏 (+8.1, +9.4) px，以 f=760 計，不校正會造成約 1.6 cm / 每 m 高的固定偏移（高度越低越小）。
相機安裝角度造成的偏移性質相同且無法在桌上校正：1° 約 1.7 cm / 每 m 高。

**支架鬆動：** 測試期間相機一直在動，1 小時內從約 1° 轉到約 16°，傾斜 2.8° → 10°，後段每 15 秒約 0.4°，是真實移動。
延遲、幀率、偵測的結果不受影響，但「相機固定」的前提不成立。機上相機必須鎖死。

**影像保護：** 影像年齡（USB 積壓 + 處理時間）超過 0.15 s 就停止送 `LANDING_TARGET`。4 個 CPU 滿載實測：處理延遲 194 ms，超過 150 ms 的 186 幀全部擋下，送出的沒有超過 160 ms。

**飛控驗證：** ArduCopter SITL 飛一趟（`--mode real`，tag 在螢幕上，起飛 6 m → LAND）：`PL.Heal` 100%、`TAcq` 96%、離群值 0，觸地 0.30 m/s。SITL 的飛機不會讓畫面變化，只驗證資料路徑，不代表閉迴路精度。

**注意 `CAP_PROP_BUFFERSIZE`：** 設 1 會讓此相機幀率掉到 15 fps，`UsbCamera` 用預設緩衝加積壓偵測。

**偵測結果的限制：** 全部在室內、靜止、高對比螢幕上測得，沒有動態模糊、戶外強光、陰影或反光，實飛的最遠偵測距離一定較短。
理論高度上限（ID0 0.75 m、14.7 px）約 39 m 只是像素換算，不是可用距離。

## 6. 模擬環境備註

- 下視相機：960x540 @20 Hz、hfov 80°，fx = fy = 572.0，cx = 480，cy = 270；安裝於 base_link（IMU）下方 3 cm，
  `models/iris_downcam/model.sdf` 的 link pose 相對外層 iris_demo 原點（腳架底），所以 z = 0.165
- 影像串流用 PX4 編好的 `libgazebo_gst_camera_plugin.so`（`PX4_DIR` 可覆寫路徑）；
  此環境 OpenCV 未編 GStreamer，程式用 python-gi 接收
- 真值：Gazebo `gz model -p`（起始 / 落地）與 SITL `SIM_STATE`（飛行中）；Gazebo +X = 北、+Y = 西
- SITL 與 Gazebo lockstep，模擬時間會與實際時間漂移，因此程式所有時間戳都用 host 接收時間
# ardupilot_Precision_landing
