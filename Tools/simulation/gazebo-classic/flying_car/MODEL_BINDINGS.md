# FS-FASMv8 机型与 Gazebo 模型绑定表

每个 airframe 使用独立的 SDF 入口。融合版将飞行汽车资源放在
`Tools/simulation/gazebo-classic/flying_car/models`，并由 `prepare.py`
复制到 Gazebo Classic 构建目录。

| SYS_AUTOSTART | 启动方式 | Gazebo 模型 |
| --- | --- | --- |
| 80003 | `make px4_sitl gazebo-classic_fs_fasmv8_flying_car_quad` | `fs_fasmv8_flying_car_quad` / `FS-FASMv8_quad` |
| 80004 | `make px4_sitl gazebo-classic_fs_fasmv8_flying_car` | `fs_fasmv8_flying_car` / `FS-FASMv8_octo` |
| 80005 | `make px4_sitl gazebo-classic_fs_fasmv8_flying_car_light` | `fs_fasmv8_flying_car_light` / `FS-FASMv8_octo_light` |

ROS launch 文件继续使用原版模型名：

- `launch/FS_flying_car.launch` -> `FS-FASMv8_octo`
- `launch/FS_flying_car_light.launch` -> `FS-FASMv8_octo_light`
- `launch/FS_flying_car_realsense.launch` -> `FS-FASMv8_octo_realsense`
- `launch/FS_flying_car_quad_v1.launch` -> `FS-FASMv8_quad_v1`
- `launch/FS_flying_car_quad_v2.launch` -> `FS-FASMv8_quad_v2`

直接 `make` 使用的两个小写模型目录是 SDF 入口别名。它们引用原版八旋翼模型的
mesh 资源，避免复制大型 mesh 文件；PX4 的 `PX4_SIM_MODEL` 名称仍与 airframe
后缀一致，从而分别选择 80004 和 80005。

共享资源包括：

- `worlds/FS_flying_car.world`
- Gazebo MAVLink 和电机插件
- `FS-FASMv8_octo` 的 mesh；light SDF 本身也引用这些 mesh

