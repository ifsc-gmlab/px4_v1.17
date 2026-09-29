# FS-FASMv8 飞行汽车：机型说明与实验方法

> 适用范围：PX4 v1.17.0 SITL（Gazebo Classic + ROS Noetic + MAVROS）。
> 本文档说明 80003 / 80004 / 80005 各机型的用途、资产绑定关系与标准实验方法。

## 1. 平台概述

FS-FASMv8 是一类“飞行汽车”（Flying Car）实验平台：同一台车在地面以差速轮式（Rover）移动，
在空中以多旋翼飞行。PX4 侧通过自定义 `MAV_TYPE` 参数（10=地面 Rover，2/14=多旋翼飞行）在
Commander 中切换飞控栈（`rover_differential` ↔ `control_allocator` + MC 控制器）。

每个机型对应**独立的 Gazebo SDF 模型目录**，互不共享模型文件（绑定关系见第 2 节表格，
规则详见 `Tools/simulation/gazebo-classic/flying_car/MODEL_BINDINGS.md`）。

## 2. 机型总览

| Airframe（SYS_AUTOSTART） | 构型 | 整车规模 | 飞行 MAV_TYPE | SDF 模型 | launch 文件 | 验证状态 |
| --- | --- | --- | --- | --- | --- | --- |
| 80003 | Quad + Rover Differential（4 旋翼 + 4 轮） | 机体 8 kg，整车约 16 kg | 2 | `FS-FASMv8_quad` | `launch/FS_flying_car_quad.launch` | ✅ SITL 端到端验证（2026-08-12） |
| 80004 | Octo 差速飞车（8 旋翼 + 4 轮） | 1000 kg 级 | 14 | `FS-FASMv8_octo` | `launch/FS_flying_car.launch` | ✅ 完整实验验证（0 validation failure） |
| 80005 | Octo 差速飞车 Light（8 旋翼 + 4 轮） | 50 kg | 14 | `FS-FASMv8_octo_light` | `launch/FS_flying_car_light.launch` | ⚠️ 可启动（autostart=80005），未跑完整实验 |
| 80005 + realsense | 80004 + 相机模型 | 1000 kg 级 | 14 | `FS-FASMv8_octo_realsense` | `launch/FS_flying_car_realsense.launch` | ⚠️ 模型存在，未单独验证 |

> 地面模式均为 `MAV_TYPE=10`。模式切换必须在 **disarmed** 状态下进行（armed 时 Commander 会锁定 MAV_TYPE）。

## 3. 各机型用途与适用场景

### 3.1 80003 —— 轻量四旋翼飞车（实验性）

- **用途**：验证“小型四旋翼 + 差速轮式”构型的飞控栈切换与 SITL 可行性；作为轻量平台，便于后续
  做算法快速迭代（机体小、推力裕度高：TWR≈2.5）。
- **适用场景**：四旋翼飞行汽车构型研究、控制器移植、地面/空中混合模态实验。
- **资产**：`ROMFS/px4fmu_common/init.d-posix/airframes/80003_gazebo-classic_fs_fasmv8_flying_car_quad`
  （F450 X 布局，旋翼 M1..M4 为输出 1..4，轮子为输出 9..12，`RD_MOT_OFFSET=8`）。
- **已知限制**（详见第 5 节）：ODE 圆柱轮地接触产生约 0.009 m/s² 侧向漂移；Rover 轮地牵引力不足
  （指令链路已打通，物理牵引需继续调参）；地面漂移会引入 EKF 高度偏差。

### 3.2 80004 —— Octo 差速飞车（基准机型，1000 kg 级）

- **用途**：项目**基准验证机型**，用于端到端自动实验（地面前进 → 起飞 → 空中前进 → 悬停 →
  降落 → 地面转向/斜行）、飞行稳定性与 PID 调参。
- **适用场景**：完整实验管线验证、回归测试基准、参数整定。
- **资产**：`80004_gazebo-classic_fs_fasmv8_flying_car`（8 旋翼 + 4 轮，`RD_MOT_OFFSET=8`，
  SDF 已设 `enable_lockstep=false`）。
- **状态**：完整 SITL 实验通过，0 validation failure。

### 3.3 80005 —— Octo Light（50 kg 轻量变体）

- **用途**：与 80004 同构型的轻量化变体，用于对比质量/推力对动力学与参数的影响。
- **状态**：可正常启动（MAVROS 连接、autostart=80005），尚未跑完整实验；light SDF 仍为
  `lockstep=1`，与 80004 不一致，跑实验前建议先对齐。

### 3.4 FS_flying_car_realsense —— 视觉版（80004 + 相机）

- **用途**：在 80004 基础上挂载 realsense 相机模型，用于视觉算法 / 相机-飞控联合实验。
- **状态**：模型存在；realsense SDF 仍为 `lockstep=1`，未单独验证。

## 4. 实验方法

### 4.1 自动实验（推荐，适用 80004/80005）

自动实验由 `Tools/flying_car_test` 包驱动：`experiment_controller` 按配置发布设定点，
`experiment_monitor` + `safety_guard` 采集并校验，`run_trials.py` 负责跑多轮并输出验证报告。

轨迹阶段（与 `config/experiment.yaml` 对应）：

`GROUND_FORWARD → TAKEOFF → AIR_FORWARD → HOVER → MANUAL_AIR → LAND → GROUND_TURN → GROUND_DIAGONAL → COMPLETE`

一键运行：

```bash
source /opt/ros/noetic/setup.bash
export ROS_PACKAGE_PATH=$PWD:$PWD/Tools/simulation/gazebo-classic/sitl_gazebo-classic:/opt/ros/noetic/share
source Tools/simulation/gazebo-classic/setup_gazebo.bash $PWD build/px4_sitl_default

# 80004（默认）或 80005：full_experiment.launch 由 run_trials.py 内部拉起
python3 Tools/flying_car_test/scripts/run_trials.py \
  --output-root /tmp/flying_car_exp --trials 1 --timeout-s 900
```

主要验证指标（容差见 `config/experiment.yaml`）：

- 地面前进/斜行距离与横向偏差
- 起飞高度（`takeoff_agl_m=4.0`，容差 0.75 m）
- 空中前进距离与横向偏差
- 悬停高度与时间
- 降落收敛与最终地面位置

### 4.2 手动实验（QGC / 实体遥控器，适用 80004/80005）

运行方式见 `Tools/flying_car_test/README.md`：使用 `monitor_only.launch`（被动监听），
操作者通过 QGC 虚拟摇杆或实体遥控器按 7 个 stage marker 执行：

`ground_forward_start → flight_switch_start → takeoff_start → air_forward_start → landing_start → final_ground_start → experiment_end`

```bash
roslaunch flying_car_test monitor_only.launch gui:=false
# 另一终端，每段开始时发布对应 marker
rosrun flying_car_test stage_marker.py ground_forward_start
...
```

### 4.3 80003 SITL 验证实验

80003 目前使用冒烟脚本 `work/exp_80003_smoke.sh`（Windows 侧，经
`wsl -d px4_sitl -- bash ...` 执行），流程：

1. 启动 `FS_flying_car_quad.launch`，等待 `SYS_AUTOSTART=80003` 与 MAVROS 连接，EKF 静置 15 s；
2. **地面段**：`MAV_TYPE=10` → MANUAL → 先建立连续手动输入（20 Hz，无间隙）→ 上锁 →
   推油门（z=750）8 s → 中位 → 解锁；
3. **飞行段**：`MAV_TYPE=2` → ALTCTL → 中位（z=500）上锁 → z=650 爬升约 16 s →
   中位悬停 12 s → AUTO.LAND → 解锁。

预期结果（2026-08-12 实测）：Rover 上锁成功；MC 上锁成功；ALTCTL 起飞至约 13.7 m；
悬停阶段高度收敛；AUTO.LAND 落地并完成解锁。

> 注意：MC 上锁要求油门杆在最低位（手动 z=0，即 `throttle < -0.8`），上锁后再推油门；
> Rover 上锁要求先有有效手动输入。

### 4.4 飞行/地面模式切换（通用）

```bash
# 查看当前模式（14/2=飞行，10=地面）
rosservice call /mavros/param/get "param_id: 'MAV_TYPE'"
# 切地面
rosservice call /mavros/param/set "{param_id: 'MAV_TYPE', value: {integer: 10, real: 0.0}}"
# 切飞行（80003 用 2，80004/80005 用 14）
rosservice call /mavros/param/set "{param_id: 'MAV_TYPE', value: {integer: 14, real: 0.0}}"
```

必须在 disarmed 状态切换；切换后 Commander 会自动启停对应控制模块
（日志可见 `Flying Car: switched to ROVER/MULTICOPTER mode`）。

### 4.5 启动命令速查

```bash
cd /path/to/PX4-Autopilot
source /opt/ros/noetic/setup.bash
export ROS_PACKAGE_PATH=$PWD:$PWD/Tools/simulation/gazebo-classic/sitl_gazebo-classic:/opt/ros/noetic/share
source Tools/simulation/gazebo-classic/setup_gazebo.bash $PWD build/px4_sitl_default

# 80004（1000 kg）
roslaunch px4 FS_flying_car.launch gui:=false
# 80005（50 kg）
roslaunch px4 FS_flying_car_light.launch gui:=false
# 80003（轻量四旋翼飞车）
roslaunch px4 FS_flying_car_quad.launch gui:=false interactive:=false
# 80004 + realsense
roslaunch px4 FS_flying_car_realsense.launch gui:=false
```

## 5. 验证状态与已知限制

| 机型 | 验证状态 | 已知限制 |
| --- | --- | --- |
| 80003 | ✅ 启动、Rover 上锁/指令链路、MC 上锁、ALTCTL 起飞 13.7 m、悬停、AUTO.LAND | ODE 轮地漂移（~0.009 m/s²）；Rover 地面牵引不足；EKF 高度偏差约 2–4 m；STABILIZED 高油门时姿态振荡（增益待整定） |
| 80004 | ✅ 完整自动实验 0 failure | 无（作为基准） |
| 80005 | ⚠️ 可启动 | light SDF `lockstep=1` 与 80004 不一致，未跑完整实验 |
| 80005 realsense | ⚠️ 模型存在 | realsense SDF `lockstep=1`，未单独验证 |

跨机型通用注意点：

- WSL 嵌套虚拟化下会出现 `TM: Time jump detected`（约每 50 s 一次），属环境噪声；
- SITL 记录器在上锁时会轮转 ulog（一次实验产生两个文件，分析时都要取）；
- `rover_differential Task already running` 为切换时重复 start 的无害报错；
- 修改任一机型只允许改动它自己的 SDF 目录（见 `MODEL_BINDINGS.md`）。

## 6. 相关文档

- `Tools/flying_car_test/README.md`：手动实验（QGC / 实体遥控）操作手册
- `launch/FS_flying_car_light.README.md`：80005 light 运行说明
- `Tools/simulation/gazebo-classic/flying_car/MODEL_BINDINGS.md`：机型↔模型绑定表
- fs-fasmv8 仓库 `docs/reference/airframes-and-experiments.md`：overlay 视角的机型与实验文档
