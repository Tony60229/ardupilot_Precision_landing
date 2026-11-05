#!/usr/bin/env bash
# ArduPilot + Gazebo Classic：AprilTag 視覺導引降落世界
#   ./run_sim.sh            有 Gazebo 畫面
#   HEADLESS=1 ./run_sim.sh 無畫面（gzserver）
#
# 另開終端機啟動 ArduCopter SITL：
#   cd ~/ardupilot/ArduCopter
#   ../Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris --console --map
#   （沒帶參數檔時 PLND_TYPE = 3；ap_precland.py --mode sim 連上後會自動寫入並重開飛控一次）
#
# 下視相機影像：H.264 RTP → udp 127.0.0.1:5600（與 vte_land.py sim 模式相同的接收 pipeline）
set -e
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PX4_DIR="${PX4_DIR:-$HOME/px4/PX4-Autopilot}"
GST_PLUGIN_DIR="$PX4_DIR/build/px4_sitl_default/build_gazebo-classic"   # libgazebo_gst_camera_plugin.so

source /usr/share/gazebo/setup.sh
export GAZEBO_MODEL_PATH="$HERE/models:$HOME/ardupilot_gazebo/models:$GAZEBO_MODEL_PATH"
export GAZEBO_RESOURCE_PATH="$HERE/worlds:$GAZEBO_RESOURCE_PATH"
export GAZEBO_PLUGIN_PATH="$HOME/ardupilot_gazebo/build:$GST_PLUGIN_DIR:$GAZEBO_PLUGIN_PATH"
export LD_LIBRARY_PATH="$GST_PLUGIN_DIR:$LD_LIBRARY_PATH"
export GAZEBO_MODEL_DATABASE_URI=""   # 不連線抓線上模型庫（離線時會卡住啟動）

if [ ! -f "$GST_PLUGIN_DIR/libgazebo_gst_camera_plugin.so" ]; then
  echo "[warn] 找不到 $GST_PLUGIN_DIR/libgazebo_gst_camera_plugin.so，相機串流不會啟動" >&2
fi

WORLD="${WORLD:-$HERE/worlds/apriltag_landing.world}"   # 批次測試會用 WORLD 指定隨機平台位置的世界檔
if [ -n "$HEADLESS" ]; then
  exec gzserver --verbose "$WORLD"
else
  exec gazebo --verbose "$WORLD"
fi
