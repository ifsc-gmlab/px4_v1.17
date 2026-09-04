# PX4 Fully Actuated and Flying Car Integration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Create `D:\PX4-v1.17` from the fully actuated PX4 v1.17 baseline and integrate airframe 80003, its Pixhawk 6C firmware support, Gazebo Classic assets, launch files, and tests without changing airframes 6003/6004 or native quadrotor behavior.

**Architecture:** The clean fully actuated repository remains authoritative. A centralized `SYS_AUTOSTART` classifier makes the 6003/6004 fully actuated path and the 80003 flying-car path mutually exclusive; shared Commander and rover files receive narrow, tested branches rather than copies from the reference tree. Hardware, ROMFS, Gazebo Classic, and tests are registered only for their matching airframe.

**Tech Stack:** PX4 v1.17, C++17, PX4 modules and parameters, uORB, NuttX for Pixhawk 6C, POSIX SITL, Gazebo Classic, CMake, GoogleTest, Python `unittest`.

**Spec:** `docs/superpowers/specs/2026-09-04-px4-fully-actuated-flying-car-integration-design.md`

## Global Constraints

- Use `D:\flying_car\fully_actuated_hexav1.17-fullycontrol-1.17.0\fully_actuated_hexav1.17-fullycontrol-1.17.0` as the sole implementation baseline.
- Treat `C:\Users\Lenovo\Downloads\PX4-Autopilot_20260828\PX4-Autopilot` as read-only flying-car reference material.
- Put every implementation change in the new repository `D:\PX4-v1.17`; do not modify either source tree after this plan commit.
- Airframe 6003 retains the existing fully actuated six-rotor controller, allocator, parameters, geometry, and Motor1-6 semantics.
- Airframe 6004 retains airframe 6003 behavior plus the HX-30HM arm and TELEM2 configuration.
- Airframe 80003 uses the native four-rotor controller and a differential-wheel output block; it never enables `CA_AIRFRAME=16`.
- Native airframes, including `4001_quad_x`, never run flying-car switching or shifted wheel-output behavior.
- The real firmware target is `px4_fmu-v6c_default`.
- Integrate Gazebo Classic, not Gazebo GZ, for airframe 80003.
- Do not import `.bak` files, existing `build/` directories, 80004/80005 airframes, octo/light assets, or unrelated source-tree differences.
- Each task updates `docs/integration/flying-car-integration-log.md` and ends in a recoverable Git commit.

---

### Task 1: Create and Verify the New Integration Repository

**Files:**
- Create: `D:\PX4-v1.17\` by cloning the local baseline
- Create: `D:\PX4-v1.17\docs\integration\source-baseline.md`
- Modify: `D:\PX4-v1.17\docs\integration\flying-car-integration-log.md`

**Interfaces:**
- Consumes: baseline commit containing the approved design and this plan.
- Produces: a clean, independent Git repository at the exact user-selected destination.

- [ ] **Step 1: Verify that the destination can be created safely**

Run:

```powershell
Test-Path -LiteralPath 'D:\PX4-v1.17'
git -C 'D:\flying_car\fully_actuated_hexav1.17-fullycontrol-1.17.0\fully_actuated_hexav1.17-fullycontrol-1.17.0' status --short
```

Expected: the destination does not exist and the baseline has no uncommitted files. If the destination exists, stop without deleting or overwriting it and report the exact contents.

- [ ] **Step 2: Clone the local baseline without sharing hard links**

Run:

```powershell
git clone --no-hardlinks 'D:\flying_car\fully_actuated_hexav1.17-fullycontrol-1.17.0\fully_actuated_hexav1.17-fullycontrol-1.17.0' 'D:\PX4-v1.17'
```

Expected: `D:\PX4-v1.17\.git` exists and `git status --short` is empty.

- [ ] **Step 3: Record immutable source identities**

Create `docs/integration/source-baseline.md` with the exact results of:

```powershell
git -C 'D:\PX4-v1.17' rev-parse HEAD
git -C 'D:\PX4-v1.17' describe --always --dirty
Get-FileHash 'C:\Users\Lenovo\Downloads\PX4-Autopilot_20260828\PX4-Autopilot\ROMFS\px4fmu_common\init.d\airframes\80003_flying_car' -Algorithm SHA256
```

- [ ] **Step 4: Add the first target-repository log entry and commit**

Run:

```powershell
git add docs/integration/source-baseline.md docs/integration/flying-car-integration-log.md
git commit -m "chore: create integrated PX4 v1.17 baseline"
```

---

### Task 2: Add Mutually Exclusive Airframe Classification and Mode Policy

**Files:**
- Create: `src/lib/systemlib/flying_car_airframe.h`
- Create: `src/modules/commander/FlyingCarMode.hpp`
- Create: `src/modules/commander/FlyingCarModeTest.cpp`
- Modify: `src/modules/commander/CMakeLists.txt`
- Modify: `docs/integration/flying-car-integration-log.md`

**Interfaces:**
- Consumes: integer `SYS_AUTOSTART`, armed state, current stack, AUX request, and requested MAV type.
- Produces: `px4::isFlyingCarAirframe(int32_t)`, `FlyingCarTargetStack requestedStack(...)`, and an isolated control-allocation ownership decision.

- [ ] **Step 1: Write the failing airframe-isolation tests**

Create `FlyingCarModeTest.cpp` with at least these assertions:

```cpp
TEST(FlyingCarMode, AirframeSetsAreMutuallyExclusive)
{
	EXPECT_TRUE(px4::isFlyingCarAirframe(80003));
	EXPECT_FALSE(px4::isFlyingCarAirframe(6003));
	EXPECT_FALSE(px4::isFlyingCarAirframe(6004));
	EXPECT_FALSE(px4::isFlyingCarAirframe(4001));
	EXPECT_TRUE(px4::isFullyActuatedAirframe(6003));
	EXPECT_TRUE(px4::isFullyActuatedAirframe(6004));
	EXPECT_FALSE(px4::isFullyActuatedAirframe(80003));
}

TEST(FlyingCarMode, NativeQuadCannotRequestFlyingCarStack)
{
	EXPECT_EQ(requestedStack(false, false, false, 1.f, 10, 2), FlyingCarTargetStack::None);
}

TEST(FlyingCarMode, ArmedVehicleCannotSwitchStacks)
{
	EXPECT_EQ(requestedStack(true, true, false, 1.f, 10, 2), FlyingCarTargetStack::None);
}
```

- [ ] **Step 2: Register and run the focused test to establish RED**

Add `px4_add_unit_gtest(SRC FlyingCarModeTest.cpp)` to Commander CMake, then run:

```sh
make px4_sitl_default tests TESTFILTER=FlyingCarMode
```

Expected: compilation fails because `flying_car_airframe.h` and policy functions are absent.

- [ ] **Step 3: Implement the minimal classifier**

Create `flying_car_airframe.h`:

```cpp
#pragma once
#include <stdint.h>

namespace px4
{
constexpr bool isFlyingCarAirframe(const int32_t sys_autostart)
{
	return sys_autostart == 80003;
}
} // namespace px4
```

- [ ] **Step 4: Implement the gated policy helper**

Port the reference `FlyingCarMode.hpp`, remove 80004/80005 handling, include both airframe classifier headers, and use this signature:

```cpp
FlyingCarTargetStack requestedStack(bool flying_car_enabled, bool armed,
		bool currently_rover, float aux1, int32_t mav_type, int32_t flight_mav_type);
```

The first statement returns `None` when `flying_car_enabled` is false. `flyingCarFlightMavType()` returns Quadrotor (`2`) only for 80003 and otherwise preserves the current MAV type. `flyingCarControlAllocationEnabled()` suppresses the allocator only for enabled 80003 rover mode.

- [ ] **Step 5: Run focused and fully actuated tests**

```sh
make px4_sitl_default tests TESTFILTER=FlyingCarMode
make px4_sitl_default tests TESTFILTER=FullyActuated
```

Expected: both suites pass; 6003/6004 and 80003 membership is disjoint.

- [ ] **Step 6: Update the log and commit**

```sh
git add src/lib/systemlib/flying_car_airframe.h src/modules/commander docs/integration/flying-car-integration-log.md
git commit -m "feat: isolate flying car airframe policy"
```

---

### Task 3: Merge Flying-Car Mode Management into Commander

**Files:**
- Modify: `src/modules/commander/Commander.hpp`
- Modify: `src/modules/commander/Commander.cpp`
- Modify: `docs/integration/flying-car-integration-log.md`

**Interfaces:**
- Consumes: `SYS_AUTOSTART`, `MAV_TYPE`, arming state, and RC AUX1.
- Produces: guarded selection between the existing multicopter stack and rover-differential stack for airframe 80003 only.

- [ ] **Step 1: Add policy tests for all transition cases**

Extend `FlyingCarModeTest.cpp` to cover disarmed Flight-to-Rover, disarmed Rover-to-Flight, MAV-type requests, armed rejection in both directions, flight type restoration after booting with saved rover type, and allocator ownership. Include this native-airframe invariant:

```cpp
EXPECT_TRUE(flyingCarControlAllocationEnabled(false, true, true));
EXPECT_EQ(flyingCarFlightMavType(4001, 2), 2);
EXPECT_EQ(flyingCarFlightMavType(6003, 13), 13);
```

- [ ] **Step 2: Run the test before Commander integration**

```sh
make px4_sitl_default tests TESTFILTER=FlyingCarMode
```

Expected: policy tests pass while no Commander runtime path uses them yet.

- [ ] **Step 3: Add one cached physical-airframe flag**

In Commander initialization, derive the flag only from `SYS_AUTOSTART`:

```cpp
const bool flying_car_enabled = px4::isFlyingCarAirframe(_param_sys_autostart.get());
```

Do not infer the physical airframe from mutable `MAV_TYPE` and do not change the existing fully actuated status subscription or ground-taxi behavior.

- [ ] **Step 4: Semantically port the reference switch blocks**

Use the reference Commander only to locate required start/stop calls. Every block must be wrapped by `flying_car_enabled`, must call `requestedStack(flying_car_enabled, ...)`, must disarm before Rover-to-Flight switching, and must retain existing non-flying-car flow:

```cpp
if (flying_car_enabled) {
	const FlyingCarTargetStack target = requestedStack(true, armed, currently_rover,
		manual_control_setpoint.aux1, mav_type, flight_mav_type);
	// Perform the reference stack switch only when target is not None.
}
```

Do not copy `Commander.cpp.bak` or `Commander.cpp.bak-20260826`. Do not replace the whole baseline Commander file.

- [ ] **Step 5: Enforce invalid-combination warnings**

At startup, reject the combination `isFlyingCarAirframe(SYS_AUTOSTART) && CA_AIRFRAME == 16` with a PX4 error and leave the native quad allocator selected. Preserve the existing full-actuated warning outside the 6003/6004/4026/22000 allowlist.

- [ ] **Step 6: Build and inspect the isolation diff**

```sh
make px4_sitl_default
git diff --check
rg -n "80004|80005|Commander.cpp.bak" src/modules/commander
```

Expected: SITL builds, diff check is clean, and no excluded airframe or backup reference is imported.

- [ ] **Step 7: Update the log and commit**

```sh
git add src/modules/commander docs/integration/flying-car-integration-log.md
git commit -m "feat: add gated flying car mode switching"
```

---

### Task 4: Add Offset Wheel Mapping Without Changing Native Rover Semantics

**Files:**
- Create: `src/modules/rover_differential/RoverDifferentialActuatorMapping.hpp`
- Create: `src/modules/rover_differential/RoverDifferentialActuatorMappingTest.cpp`
- Modify: `src/modules/rover_differential/DifferentialActControl/DifferentialActControl.hpp`
- Modify: `src/modules/rover_differential/DifferentialActControl/DifferentialActControl.cpp`
- Modify: `src/modules/rover_differential/RoverDifferential.cpp`
- Modify: `src/modules/rover_differential/RoverDifferential.hpp`
- Modify: `src/modules/rover_differential/module.yaml`
- Modify: `src/modules/rover_differential/CMakeLists.txt`
- Modify: `docs/integration/flying-car-integration-log.md`

**Interfaces:**
- Consumes: `RD_MOT_OFFSET`, left/right wheel commands, armed state, and current MAV type.
- Produces: ordinary Rover output at indices 0/1 when offset is 0, 80003 wheel-pair output beginning at index 4 on hardware, and neutral heartbeat output while disarmed.

- [ ] **Step 1: Write failing mapping tests**

Test the exact contracts:

```cpp
actuator_motors_s native{};
ASSERT_TRUE(writeWheelCommands(native, 0, -0.25f, 0.75f));
EXPECT_FLOAT_EQ(native.control[0], -0.25f);
EXPECT_FLOAT_EQ(native.control[1], 0.75f);

actuator_motors_s car{};
ASSERT_TRUE(writeWheelCommands(car, 4, -0.25f, 0.75f));
EXPECT_FLOAT_EQ(car.control[4], -0.25f);
EXPECT_FLOAT_EQ(car.control[5], 0.75f);
```

Also assert that offset `9` is rejected, flight motors 0-3 are never overwritten by offset 4, a disarmed heartbeat is permitted, an armed Quadrotor heartbeat is denied, and an armed Rover command is permitted.

- [ ] **Step 2: Register and run the focused test to establish RED**

```sh
make px4_sitl_default tests TESTFILTER=RoverDifferentialActuatorMapping
```

Expected: compile failure because the helper is absent.

- [ ] **Step 3: Implement the mapping helper and parameter**

Port the reference helper, retain bounds checking against `actuator_motors_s::NUM_CONTROLS`, and add `RD_MOT_OFFSET` to the v1.17 rover module metadata with default `0`, minimum `0`, and maximum `8`. A zero offset must preserve the baseline Rover Motor1/Motor2 behavior.

- [ ] **Step 4: Merge only the required actuator-control changes**

Update `DifferentialActControl` to read the current throttle from `offset` wheel entries, copy the current actuator frame before writing the wheel block, publish neutral wheel heartbeat at the offset, and reject an invalid offset with `PX4_ERR`. Keep unrelated differential drive-mode and controller files from the fully actuated baseline.

- [ ] **Step 5: Gate publish behavior**

Apply `shouldPublishActuatorFrame(armed, mav_type)` only to the reference heartbeat/publish additions. Native rover with `RD_MOT_OFFSET=0` continues publishing Motor1/Motor2 as before; native multicopter never starts rover-differential and therefore cannot acquire wheel outputs.

- [ ] **Step 6: Run targeted Rover regression tests**

```sh
make px4_sitl_default tests TESTFILTER=RoverDifferentialActuatorMapping
make px4_sitl_default tests TESTFILTER=Rover
```

Expected: mapping and existing Rover tests pass.

- [ ] **Step 7: Update the log and commit**

```sh
git add src/modules/rover_differential docs/integration/flying-car-integration-log.md
git commit -m "feat: add isolated flying car wheel offset"
```

---

### Task 5: Register Airframe 80003 and Its Startup Scripts

**Files:**
- Create: `ROMFS/px4fmu_common/init.d/airframes/80003_flying_car`
- Create: `ROMFS/px4fmu_common/init.d-posix/airframes/80003_gazebo-classic_fs_fasmv8_flying_car_quad`
- Create: `ROMFS/px4fmu_common/init.d/rc.flying_car_defaults`
- Create: `ROMFS/px4fmu_common/init.d/rc.flying_car_apps`
- Create: `ROMFS/px4fmu_common/init.d/rc.flying_car_sitl_defaults`
- Modify: `ROMFS/px4fmu_common/init.d/CMakeLists.txt`
- Modify: `ROMFS/px4fmu_common/init.d/airframes/CMakeLists.txt`
- Modify: `ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt`
- Modify: `ROMFS/px4fmu_common/init.d/rc.vehicle_setup`
- Create: `test/integrated_airframe_isolation_test.py`
- Modify: `docs/integration/flying-car-integration-log.md`

**Interfaces:**
- Consumes: `SYS_AUTOSTART=80003`.
- Produces: the flying-car startup branch, standard four-rotor allocation, hardware wheels beginning at Motor5, and Gazebo Classic wheels beginning at the model's declared offset.

- [ ] **Step 1: Write the failing static isolation test**

The Python test must parse the target files and assert:

```python
self.assertIn("6003_fully_actuated_hexa", hw_registry)
self.assertIn("6004_fully_actuated_hexa_arm", hw_registry)
self.assertEqual(hw_registry.count("80003_flying_car"), 1)
self.assertEqual(posix_registry.count("80003_gazebo-classic_fs_fasmv8_flying_car_quad"), 1)
self.assertIn("param set-default CA_AIRFRAME 16", airframe_6003)
self.assertNotIn("CA_AIRFRAME 16", airframe_80003)
self.assertIn("param set-default RD_MOT_OFFSET 4", airframe_80003)
self.assertIn("param set-default CA_R_REV 48", flying_car_defaults)
self.assertNotIn("rc.flying_car_apps", quad_4001)
```

- [ ] **Step 2: Run the static test to establish RED**

```powershell
python test/integrated_airframe_isolation_test.py
```

Expected: failure because the 80003 files are not present.

- [ ] **Step 3: Port and normalize the hardware files**

Use the reference `80003_flying_car` and `rc.flying_car_*` scripts as source material. Keep `MAV_TYPE=2`, `CA_AIRFRAME=0`, `CA_ROTOR_COUNT=4`, the four-rotor geometry, `RD_MOT_OFFSET=4`, Motor1-4 rotor functions, Motor5-6 wheel functions, wheel neutral 1500 microseconds, wheel range 1100-1900 microseconds, and `CA_R_REV=48`. Do not import 80004/80005 defaults.

- [ ] **Step 4: Merge registries without replacing baseline entries**

Add one 80003 hardware row, one 80003 Gazebo Classic row, and the three rc scripts. Preserve 6003, 6004, 4026, and 22000 registrations byte-for-byte. Add a `VEHICLE_TYPE=flying_car` branch that sources `rc.flying_car_apps`; do not edit the existing `mc`, `rover`, or fully actuated branches.

- [ ] **Step 5: Validate shell references and metadata**

```powershell
python test/integrated_airframe_isolation_test.py
rg -n "80003|6003|6004" ROMFS/px4fmu_common/init.d/airframes/CMakeLists.txt ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt
```

Then, in a PX4 Linux environment:

```sh
make px4_sitl_default metadata_airframes
```

Expected: static test passes, each registration is unique, and metadata generation reports no duplicate ID.

- [ ] **Step 6: Update the log and commit**

```sh
git add ROMFS test/integrated_airframe_isolation_test.py docs/integration/flying-car-integration-log.md
git commit -m "feat: register airframe 80003"
```

---

### Task 6: Enable 80003 on Pixhawk 6C Without Disturbing the Arm Variant

**Files:**
- Modify: `boards/px4/fmu-v6c/default.px4board`
- Modify: `boards/px4/fmu-v6c/init/rc.board_defaults`
- Modify: `test/integrated_airframe_isolation_test.py`
- Create: `docs/integration/pixhawk6c-flying-car-output-map.md`
- Modify: `docs/integration/flying-car-integration-log.md`

**Interfaces:**
- Consumes: airframe 80003 startup and the existing Pixhawk 6C DShot/PWM drivers.
- Produces: one `px4_fmu-v6c_default` image containing full-actuated, arm, and rover-differential functionality, selected at runtime by airframe.

- [ ] **Step 1: Extend the failing board test**

Add assertions that `default.px4board` contains all of:

```text
CONFIG_MODULES_COMMANDER=y
CONFIG_MODULES_CONTROL_ALLOCATOR=y
CONFIG_MODULES_ROVER_DIFFERENTIAL=y
CONFIG_MODULES_MC_ATT_CONTROL=y
CONFIG_MODULES_MC_POS_CONTROL=y
CONFIG_MODULES_MC_RATE_CONTROL=y
CONFIG_DRIVERS_DSHOT=y
CONFIG_DRIVERS_PWM_OUT=y
CONFIG_DRIVERS_ACTUATORS_HX30HM_ARM=y
```

Also assert that the TELEM2 release block remains in airframe 6004 and is absent from 80003.

- [ ] **Step 2: Run the test to establish RED**

```powershell
python test/integrated_airframe_isolation_test.py
```

Expected: failure for the missing Rover Differential board selection.

- [ ] **Step 3: Add the minimum board selection**

Add only `CONFIG_MODULES_ROVER_DIFFERENTIAL=y` when the other required selections already exist. Keep `CONFIG_DRIVERS_ACTUATORS_HX30HM_ARM=y`, all full-actuated build dependencies, and existing `rc.board_defaults` behavior. Do not copy the reference FMUv6X board files.

- [ ] **Step 4: Document the physical output contract**

Record AUX1-4 as DShot600 rotor outputs, AUX5-6 as 50 Hz reversible PWM wheels, 1500 microseconds as wheel neutral, and TELEM2 as 6004-only arm transport. Include a propeller-off warning and the requirement to verify Pixhawk 6C timer grouping before powered tests.

- [ ] **Step 5: Run static and firmware-build validation**

```powershell
python test/integrated_airframe_isolation_test.py
```

Then, in the configured PX4 toolchain:

```sh
make px4_fmu-v6c_default
```

Expected: static test passes and firmware builds with Commander, full-actuated control, arm driver, rover-differential, DShot, and PWM support.

- [ ] **Step 6: Update the log and commit**

```sh
git add boards/px4/fmu-v6c test/integrated_airframe_isolation_test.py docs/integration
git commit -m "feat: enable flying car on Pixhawk 6C"
```

---

### Task 7: Integrate the 80003 Gazebo Classic Models and Launch Chain

**Files:**
- Create: `Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/FS-FASMv8_quad/`
- Create: `Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/FS-FASMv8_quad_v1/`
- Create: `Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/FS-FASMv8_quad_v1_hitl/`
- Create: `Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/FS-FASMv8_quad_v2/`
- Create: `Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/FS-FASMv8_quad_v2_hitl/`
- Create: `Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/FS_flying_car.world`
- Create: `launch/FS_flying_car_quad.launch`
- Create: `launch/FS_flying_car_quad_v1.launch`
- Create: `launch/FS_flying_car_quad_v1_hitl.launch`
- Create: `launch/FS_flying_car_quad_v2.launch`
- Create: `launch/FS_flying_car_quad_v2_hitl.launch`
- Create: `test/fs_fasmv8_80003_model_contract_test.py`
- Modify: `docs/integration/flying-car-integration-log.md`

**Interfaces:**
- Consumes: the 80003 POSIX airframe, Gazebo Classic motor channels, and MAVROS launch conventions.
- Produces: reproducible 80003 SITL/HITL model variants with four rotors and observable differential-wheel joints.

- [ ] **Step 1: Write the failing model-contract test**

Use `xml.etree.ElementTree` to assert for each non-HITL SDF:

```python
self.assertEqual(len(motor_plugins_for_rotors), 4)
self.assertGreaterEqual(len(wheel_joints), 2)
self.assertEqual(sorted(rotor_motor_numbers), [0, 1, 2, 3])
self.assertTrue({8, 9, 10, 11}.issubset(set(wheel_control_channels)))
```

Assert that the primary launch file uses vehicle `fs_fasmv8_flying_car_quad`, model `FS-FASMv8_quad`, and world `FS_flying_car.world`. Assert that no octo/light/realsense model or launch is added by this task.

- [ ] **Step 2: Run the test to establish RED**

```powershell
python test/fs_fasmv8_80003_model_contract_test.py
```

Expected: failure because the model assets are absent.

- [ ] **Step 3: Copy only the 80003 quad-family assets**

Copy the five `FS-FASMv8_quad*` directories, the flying-car world, and the five `FS_flying_car_quad*` launch files from the read-only reference. Preserve binary mesh bytes. Do not copy `FS-FASMv8_octo*`, `FS_flying_car_light.launch`, `FS_flying_car.launch`, or `FS_flying_car_realsense.launch`.

- [ ] **Step 4: Normalize the primary airframe/model mapping**

Make the 80003 wrapper, `FS_flying_car_quad.launch`, and `FS-FASMv8_quad/model.config` agree on the same vehicle/model names. Preserve the model's four-rotor and four-wheel simulation channels; hardware continues to expose only Motor5/Motor6 because its PWM function map ends at AUX6.

- [ ] **Step 5: Run model and launch tests**

```powershell
python test/fs_fasmv8_80003_model_contract_test.py
python test/integrated_airframe_isolation_test.py
```

In a ROS/Gazebo Classic environment, run:

```sh
roslaunch px4 FS_flying_car_quad.launch gui:=false
```

Expected: PX4 reports `SYS_AUTOSTART=80003`, the model spawns, MAVROS connects, four rotor channels and four simulated wheel channels are addressable, and no 80004/80005 airframe is selected.

- [ ] **Step 6: Update the log and commit**

```sh
git add Tools/simulation/gazebo-classic launch test/fs_fasmv8_80003_model_contract_test.py docs/integration/flying-car-integration-log.md
git commit -m "feat: add 80003 Gazebo Classic simulation"
```

---

### Task 8: Integrate 80003 Test Tools and Run the Full Regression Gate

**Files:**
- Create: `Tools/flying_car_test/` with 80003-capable experiment sources, scripts, launch, config, tests, and fixtures
- Create: `docs/integration/flying-car-validation-report.md`
- Create: `docs/integration/pixhawk6c-propeller-off-bench-test.md`
- Modify: `docs/integration/flying-car-integration-log.md`

**Interfaces:**
- Consumes: all preceding code, airframes, hardware configuration, models, and launch files.
- Produces: repeatable evidence for 4001, 6003, 6004, and 80003 plus a safe real-hardware verification procedure.

- [ ] **Step 1: Copy and normalize the 80003 experiment package**

Copy `Tools/flying_car_test` from the reference, then set the default experiment airframe/model/launch to 80003, `FS-FASMv8_quad`, and `FS_flying_car_quad.launch`. Keep tests and fixtures needed by the package; remove registrations that claim 80004/80005 are supported by the integrated target.

- [ ] **Step 2: Run platform-independent Python tests**

```powershell
python -m pytest test/integrated_airframe_isolation_test.py test/fs_fasmv8_80003_model_contract_test.py Tools/flying_car_test/test -q
```

Expected: all static, configuration, safety, reporting, and package tests pass.

- [ ] **Step 3: Run focused PX4 C++ tests in Linux/WSL**

```sh
make px4_sitl_default tests TESTFILTER=FlyingCarMode
make px4_sitl_default tests TESTFILTER=RoverDifferentialActuatorMapping
make px4_sitl_default tests TESTFILTER=FullyActuated
make px4_sitl_default tests TESTFILTER=Rover
```

Expected: all targeted suites pass.

- [ ] **Step 4: Run full build regression**

```sh
make px4_sitl_default
make px4_fmu-v6c_default
make tests
```

Expected: SITL, Pixhawk 6C firmware, and the full unit suite succeed. Record exact failures separately when they are caused by unavailable Windows/WSL/Gazebo dependencies.

- [ ] **Step 5: Verify source isolation and excluded content**

```powershell
git status --short
rg -n "80004|80005" ROMFS/px4fmu_common/init.d/airframes ROMFS/px4fmu_common/init.d-posix/airframes
Get-ChildItem -Recurse -File | Where-Object Name -Match '\.bak($|-)|Commander\.cpp\.bak'
```

Expected: Git is clean before the report commit, no new 80004/80005 registration was imported, and no backup file exists in the integration delta.

- [ ] **Step 6: Write the validation and bench reports**

The validation report records command, date, result, firmware path/hash, skipped checks, and reason. The bench procedure requires propellers removed, wheels off the ground, emergency power cut available, AUX1-4 checked one at a time as rotors, AUX5-6 checked one at a time as reversible wheels, 1500 microsecond neutral verified, 6004 TELEM2 arm checked separately, and immediate abort on any cross-channel motion.

- [ ] **Step 7: Commit final evidence**

```sh
git add Tools/flying_car_test docs/integration
git commit -m "test: validate integrated PX4 configurations"
git status --short
```

Expected: the final working tree is clean and the validation report distinguishes tested behavior from hardware-only checks.
