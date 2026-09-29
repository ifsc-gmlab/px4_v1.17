#!/usr/bin/env bash
#===============================================================================
# 一键干净启动 PX4 SITL 飞行汽车仿真 (FS-FASMv8_quad_v2)
# 用法:
#   ./start_flying_car.sh                    # 默认: gui:=true interactive:=false
#   ./start_flying_car.sh headless:=true     # 传入自定义 roslaunch 参数(覆盖默认)
#   ./start_flying_car.sh --no-clean         # 跳过进程清理
#===============================================================================

set -e

# ===== 路径配置 =====
PX4_DIR="$HOME/new/px4-fs-quad-v2"
BUILD_DIR="$PX4_DIR/build/px4_sitl_default"
SITL_DIR="$BUILD_DIR/sitl_gazebo-classic_flying_car"
LAUNCH_FILE="FS_flying_car_quad_v2.launch"

# ===== 解析 --no-clean 参数 =====
DO_CLEAN=1
LAUNCH_ARGS=()
for arg in "$@"; do
    if [ "$arg" = "--no-clean" ]; then
        DO_CLEAN=0
    else
        LAUNCH_ARGS+=("$arg")
    fi
done

# ===== 1. 清理残留进程 =====
if [ "$DO_CLEAN" -eq 1 ]; then
    echo "[1/4] 清理残留的 px4 / gazebo 进程..."
    pkill -9 -f "build/px4_sitl_default/bin/px4" 2>/dev/null || true
    pkill -9 -f "px4-simulator_mavlink"          2>/dev/null || true
    pkill -9 -f "gzserver"                       2>/dev/null || true
    pkill -9 -f "gzclient"                       2>/dev/null || true
    sleep 1
else
    echo "[1/4] 跳过进程清理 (--no-clean)"
fi

# ===== 2. 加载 ROS 环境 =====
echo "[2/4] 加载 ROS + Gazebo 环境..."
source /opt/ros/noetic/setup.bash

# ===== 3. 设置 ROS 包路径 =====
echo "[3/4] 设置 ROS_PACKAGE_PATH..."
export ROS_PACKAGE_PATH="$SITL_DIR:$PX4_DIR:$ROS_PACKAGE_PATH"

cd "$PX4_DIR"
source Tools/simulation/gazebo-classic/setup_gazebo.bash "$PX4_DIR" "$BUILD_DIR"
unset HEADLESS GAZEBO_MASTER_URI

# ===== 4. 启动仿真 =====
if [ ${#LAUNCH_ARGS[@]} -eq 0 ]; then
    LAUNCH_ARGS=(gui:=true interactive:=false)
fi
echo "[4/4] 启动仿真: roslaunch px4 $LAUNCH_FILE ${LAUNCH_ARGS[*]}"
roslaunch px4 "$LAUNCH_FILE" "${LAUNCH_ARGS[@]}"
