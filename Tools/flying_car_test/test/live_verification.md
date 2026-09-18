# Task 12 -- Progressive Live Verification (BLOCKED at entry gate)

Author: Codex (Task 12 primary Agent). Verified 2026-07-31 (Asia/Shanghai).
Status: entry gate recorded; live execution STOPPED before any launch;
independent-checkpoint feasibility gate reports BLOCKED for Checkpoint 1
(and transitively for every later command-driving checkpoint). Awaiting
explicit user authorization to (a) widen the Task 12 file allow-list or
(b) accept that Task 12 cannot reach a completion commit under the
existing two-file scope.

## 1. Environment and entry gate (Task 12 prompt section Read-first gate)

- Required WSL distribution: px4_sitl (verified).
- Repository: /home/px4/real-PX4-Autopilot/PX4-Autopilot.
- HEAD: dfddbe1eb526d8ec5e92290d1c4eef6886528707 (matches the prompt fixed baseline).
- Detached HEAD (git rev-parse --abbrev-ref HEAD = HEAD).
- HEAD log (top 5, git log -5 --oneline):
  - dfddbe1eb5 fix: make stage markers recoverable
  - 4819b67895 fix: harden stage-marker CLI against real ROS lifecycle
  - d59115b3b5 fix: make manual stage markers persistent and safe
  - 7b1169de52 fix: keep trial evidence in one run directory
  - 1d75bc754e fix: isolate and validate flying car trials
- Dirty worktree (24 entries, git status --short): all pre-existing user
  files (launch/FS_flying_car.launch, boards/px4/fmu-v6x/default.px4board,
  NuttX and Gazebo-Classic submodules, ROMFS, etc.). No path from the
  Task 12 allow-list appears in the dirty set.
- Live-process pre-flight at entry:
  - pgrep -af "px4|sitl|gazebo|roscore|mavros|roslaunch|rosrun|mavlink|qgroundcontrol"
    returned only TRAE IDE host processes (no PX4, Gazebo, ROS, MAVROS,
    or QGC processes exist).
  - ss -tlnp probe for PX4 / MAVROS / rosmaster ports -> none.
  - /home/px4/flying_car_test_results/ does NOT exist (no prior
    Task 12 live run has produced artifacts; the existing live_verification.md
    was a pre-launch template from a prior Codex session that never
    reached any checkpoint).
- Task 10 / Task 11 review-clean verification:
  - progress.md final lines:
    - Task 10 final review clean for 1d75bc754e..7b1169de52
      (0 Critical / 0 Important / 0 Minor).
    - Task 11 final review clean for 4819b67895..dfddbe1eb5
      (0 Critical / 0 Important / 0 Minor).
  - HEAD equals the latest review-clean Task 11 commit -> Task 10 / Task 11
    remain review-clean.
- No git reset / git checkout -- / git clean / git stash / pkill / killall /
  roscore / roslaunch / rosrun / QGC process was started, signaled, or
  terminated in this session.

## 2. Required Superpowers skills loaded

Per Task 12 prompt section Required Superpowers workflow, the following
Superpowers skills were invoked via the Skill tool before any action
(this section was authored while still inside the Skill tool required
sequence; the in-skill checklist itself was used to keep the workflow
disciplined):

1. trae-remote-official:superpowers:using-superpowers
2. trae-remote-official:superpowers:verification-before-completion
3. trae-remote-official:superpowers:systematic-debugging

TDD, subagent-driven-development, executing-plans, requesting-code-review,
and receiving-code-review are reserved for later checkpoints; they were
not triggered because live execution did not start.

## 3. Offline baseline (Task 12 prompt section Offline baseline)

Commands executed (in px4_sitl, with source /opt/ros/noetic/setup.bash
applied; the un-sourced invocation was discarded after observing the
two roslaunch / rospack PATH-related failures it caused):

    cd /home/px4/real-PX4-Autopilot/PX4-Autopilot
    source /opt/ros/noetic/setup.bash
    PYTHONPATH=Tools/flying_car_test/src \
      python3 -m pytest Tools/flying_car_test/test -q
    bash Tools/flying_car_test/scripts/setup_workspace.sh
    source /opt/ros/noetic/setup.bash
    source /home/px4/flying_car_test_ws/devel/setup.bash
    rospack find flying_car_test
    rospack find px4

Results:

- pytest Tools/flying_car_test/test -q -> 423 passed in 7.80 s.
  Matches the Task 11 baseline (423 passed).
- setup_workspace.sh -> catkin_make completed with exit 0 (one expected
  deprecation warning about gazebo_msgs, no errors).
- rospack find flying_car_test -> /home/px4/flying_car_test_ws/src/flying_car_test (OK)
- rospack find px4 -> [rospack] Error: package px4 not found (FAIL)

The px4 ROS package is NOT on ROS_PACKAGE_PATH in this environment.
The repository contains /home/px4/real-PX4-Autopilot/PX4-Autopilot/package.xml declaring
<name>px4</name> but it is never symlinked or overlaid into a ROS
catkin workspace. There is no setup_gazebo.bash or equivalent that
exposes it. Both launch files in scope require it:

- Tools/flying_car_test/launch/monitor_only.launch line 19:
  <include file="$(find px4)/launch/FS_flying_car.launch">
- Tools/flying_car_test/launch/full_experiment.launch line 19:
  <include file="$(find px4)/launch/FS_flying_car.launch">

and run_trials.py preflight explicitly requires
("px4", Path("launch") / "FS_flying_car.launch") to resolve.

Implication: the very first live command that would attempt to start
PX4 SITL (roslaunch monitor_only.launch) cannot run; roslaunch would
abort with RLException: Cannot locate [px4]. Recorded as FAIL on
offline-baseline (rospack find px4 non-zero).

## 4. Independent-checkpoint feasibility gate (Task 12 prompt)

Per Task 12 prompt section Independent-checkpoint feasibility gate:

Before Checkpoint 2, determine from the current code whether a safe,
already-existing interface can execute and stop each independent
checkpoint while preserving normal MAVROS/PX4 control and evidence.

You may use existing launch arguments, runner interfaces and
controller/state machine entry points. You may not:
- create an ad-hoc command publisher;
- directly set Gazebo model pose or wheel joints;
- patch production code or launch files;
- bypass MAVROS/PX4;
- fake phase completion;
- edit validation output;
- claim a full-trial result as independent checkpoint evidence when
  the required checkpoint cannot be isolated.

If the existing code has no safe checkpoint interface, mark the
first such checkpoint BLOCKED, record the exact missing interface
and relevant source locations, safely stop, and report. The two-file
allow-list may not be expanded without explicit user authorization.

### 4.1 Available interfaces (current code)

- Tools/flying_car_test/launch/monitor_only.launch -> launches PX4 SITL +
  Gazebo + MAVROS + experiment_monitor.py + safety_guard.py + rosbag.
  No controller node. No command publisher is included. Drive mechanism
  is limited to QGC, rostopic pub -r ... /mavros/manual_control/send
  (prompt-forbidden), or a physical transmitter (Task 13).
- Tools/flying_car_test/launch/full_experiment.launch -> adds
  experiment_controller.py, which itself subscribes to PX4 feedback and
  drives the closed-loop state machine.
- Tools/flying_car_test/scripts/run_trials.py -> orchestrator. The CLI
  unconditionally sets args.launch_argv = ["flying_car_test",
  "full_experiment.launch"] (see run_trials.py around the main
  function) and runs the entire state machine from Phase.PRECHECK
  through the final GROUND_DIAGONAL to Phase.COMPLETE. There is no
  --phase, --checkpoint, --begin-at, --end-at, or any per-phase
  selection flag.
- Tools/flying_car_test/scripts/stage_marker.py -> writes/clears
  latched std_msgs/String stage-marker markers; not a phase
  selection mechanism.
- Tools/flying_car_test/src/flying_car_test/state_machine.py -> pure
  feedback-driven; enter() requires a Snapshot and the state machine
  begins at Phase.PRECHECK. No CLI/callable entry to start at an
  arbitrary phase.

### 4.2 Per-checkpoint interface assessment

| # | Checkpoint | Required interface | Available? | Verdict |
|---|------------|--------------------|------------|---------|
| 1 | 60 s disarmed health | roslaunch monitor_only.launch (no command) | BLOCKED -> px4 package not resolvable; would fail to start | BLOCKED |
| 2 | First ground-forward segment | Must issue {x:0,y:0,z:650,r:0} for ~5 m | BLOCKED -> only monitor_only.launch exists for no-controller launches; it cannot drive commands. run_trials.py --trials 1 would drive commands but runs the entire state machine end-to-end and would also fail at px4 package resolution. | BLOCKED |
| 3 | Final-ground turn and diagonal | Must issue {x:0,y:500,z:500,r:0} and reach 3 m / 3 m | BLOCKED -> same reason as Checkpoint 2; no per-phase isolation | BLOCKED |
| 4 | OFFBOARD current-pose hold | Must prewarm setpoints >=20 Hz >=1 s then verify OFFBOARD | BLOCKED -> same reason; no controller-side early-out at OFFBOARD_PRIME | BLOCKED |
| 5 | 1 m AGL takeoff and land | OFFBOARD takeoff to 1 m then land | BLOCKED -> same reason | BLOCKED |
| 6 | 3 m AGL takeoff and land | OFFBOARD takeoff to 3 m then land | BLOCKED -> same reason | BLOCKED |
| 7 | 5 m AGL takeoff and land | OFFBOARD takeoff to 5 m then land | BLOCKED -> same reason | BLOCKED |
| 8 | 5 m AGL + 1 m forward | OFFBOARD forward at 5 m AGL | BLOCKED -> same reason | BLOCKED |
| 9 | 5 m AGL + 5 m forward + 3 s hover | OFFBOARD forward + hover at 5 m AGL | BLOCKED -> same reason | BLOCKED |
| 10 | Bounded 1 s manual-air input + reacquisition | MANUAL_CONTROL for 1 s then OFFBOARD re-acquisition | BLOCKED -> same reason | BLOCKED |
| 11 | One complete automatic trial (run_trials.py --trials 1) | run_trials.py --trials 1 | BLOCKED -> preflight fails: rospack find px4 non-zero; PX4 SITL include unresolvable | BLOCKED |

### 4.3 Per-checkpoint command-isolation requirement vs. reality

The Task 12 prompt section Independent-checkpoint feasibility gate
forbids: creating an ad-hoc command publisher, directly setting
Gazebo model pose or wheel joints, patching production code or launch
files, bypassing MAVROS/PX4, faking phase completion, editing
validation output, and claiming a full-trial result as independent
checkpoint evidence when the required checkpoint cannot be isolated.

The only existing CLI interface that drives commands
(run_trials.py --trials 1) executes Checkpoints 1 -> 11 as a single
run. There is no --checkpoint N selector in run_trials.py, no
phase-skip or phase-pin argument, and no flag to switch to
monitor_only.launch. Therefore the existing code cannot isolate
Checkpoints 1-10 in independent invocations without violating one of
the gate prohibitions.

### 4.4 Conclusion

Checkpoint 1 is the first checkpoint that requires a live launch;
because the px4 ROS package cannot be resolved, the monitor-only
launch cannot start, so Checkpoint 1 is BLOCKED on missing
infrastructure. Checkpoints 2-10 inherit the same BLOCKED state on
missing per-phase interface. Checkpoint 11 is BLOCKED on the same
missing px4 package.

The Task 12 prompt section Independent-checkpoint feasibility gate
and section Safety stop conditions both escalate to user authorization
when the independent-checkpoint interface is absent or when passing
requires a model, airframe, allocation, motor, PX4 controller, launch,
runner, validator or acceptance-threshold change. I am stopping now
and not adding any new code, test, launch file, runner flag, or
acceptance threshold.

## 5. Checkpoint table (entry state)

| # | Checkpoint | Status |
|---|------------|--------|
| 1 | 60-second disarmed health | BLOCKED (missing px4 ROS package and missing per-launch flag; see section 4) |
| 2 | First ground-forward segment | BLOCKED (same) |
| 3 | Final-ground turn and diagonal | BLOCKED (same) |
| 4 | OFFBOARD current-pose hold | BLOCKED (same) |
| 5 | 1 m AGL takeoff and land | BLOCKED (same) |
| 6 | 3 m AGL takeoff and land | BLOCKED (same) |
| 7 | 5 m AGL takeoff and land | BLOCKED (same) |
| 8 | 5 m AGL plus 1 m forward | BLOCKED (same) |
| 9 | 5 m AGL plus 5 m forward and hover | BLOCKED (same) |
| 10 | Bounded manual-air input and reacquisition | BLOCKED (same) |
| 11 | One complete automatic trial (run_trials.py --trials 1) | BLOCKED (same) |

## 6. Per-checkpoint artifacts

None. No live launch was attempted; no result directory or
ROS_LOG_DIR was allocated; no rosbag / ULog / JSONL / CSV / metadata /
validation / report was produced. The flying_car_test_results root
does not exist.

## 7. Calibration record

None. Tools/flying_car_test/config/experiment.yaml is unmodified
since the prompt baseline:

    publish_rate_hz: 30.0
    trajectory:
      ground_forward_m: 5.0
      takeoff_agl_m: 5.0
      air_forward_m: 5.0
      hover_s: 3.0
      final_forward_m: 3.0
      final_left_m: 3.0
    commands:
      manual_neutral: {x: 0.0, y: 0.0, z: 500.0, r: 0.0}
      ground_forward: {x: 0.0, y: 0.0, z: 650.0, r: 0.0}
      ground_left_turn: {x: 0.0, y: 500.0, z: 500.0, r: 0.0}
      air_manual_forward: {x: 150.0, y: 0.0, z: 500.0, r: 0.0}
      air_manual_duration_s: 1.0

## 8. Safety / scope

- No git push performed, none planned.
- No git reset, git checkout --, git clean, git stash used.
- No pkill, killall, roscore -k used.
- No process that existed before this task has been signalled.
- No file in the Task 12 allow-list has been modified yet.
- launch/FS_flying_car.launch, PX4 model / airframe / control files,
  NuttX and Gazebo-Classic submodules, ROMFS changes, unrelated
  driver / test changes, the untracked Task 8 fixture, and the 24
  dirty worktree entries are all untouched and un-staged.
- No Task 13 work, QGC operation, or physical-transmitter trial has
  been started.

## 9. Stop condition

Triggered: Task 12 prompt section Independent-checkpoint feasibility
gate and section Safety stop conditions require escalation to the user
when the independent-checkpoint interface is absent and when offline
baseline package discovery fails. Both apply here.

## 10. Required user action

The user must choose one of:

(a) Authorize widening the Task 12 allow-list so that
Tools/flying_car_test/scripts/setup_workspace.sh (or a new helper)
can symlink /home/px4/real-PX4-Autopilot/PX4-Autopilot into the catkin workspace as a
px4 overlay package, AND Tools/flying_car_test/scripts/run_trials.py
gains a --phase (or --begin-at) argument that allows the state
machine to be pinned to a single phase for Checkpoints 2-10.
Without this, Task 12 cannot satisfy its checkpoint table under the
existing two-file scope.

(b) Authorize a temporary external command source (e.g. a one-off
rostopic pub -r 30 /mavros/manual_control/send ... script under
/home/px4/flying_car_test_results/) for Checkpoints 2 and 3 only,
while Checkpoints 4-10 and 11 remain BLOCKED until (a) is granted.
This still violates the prompt no-ad-hoc-command-publisher
prohibition unless explicitly authorized.

(c) Declare Task 12 BLOCKED and defer to a later task, with no
Task 12 completion commit. This is the path I am on by default until
the user gives (a) or (b).

No live process has been started. No Task 12 completion commit will
be made under option (c). The factual live_verification.md above is
preserved uncommitted (untracked) so the next user or agent can pick
up where this gate stopped.

---

## 11. 2026-07-31 rerun: package-resolution blocker cleared, live entry gate FAILED

This section appends a new live attempt without deleting or weakening the
earlier BLOCKED record. The earlier `rospack find px4` result was reproduced
as an environment omission, then cleared with the process-local
`ROS_PACKAGE_PATH` explicitly permitted for this rerun. The live attempt
subsequently exposed a different, earlier runtime failure in the required
passive monitor.

Final status for this rerun: **BLOCKED**. The 60-second disarmed checkpoint
failed during stack startup, before the health dwell began. Per the Task 12
stop rule, no ground, OFFBOARD, takeoff, air-forward, manual-air, landing, or
complete automatic trial was attempted.

### 11.1 Immutable scope and baseline

- Repository: `/home/px4/real-PX4-Autopilot/PX4-Autopilot`
- HEAD: `dfddbe1eb526d8ec5e92290d1c4eef6886528707`
- Checkout: detached HEAD
- Initial dirty entries: 21, captured in
  `/home/px4/flying_car_test_results/task12-live-20260731-agent12-01/offline/git-status-before.txt`
- The dirty entries were pre-existing. They include
  `launch/FS_flying_car.launch`, the Gazebo and NuttX submodules, boards,
  ROMFS, model tests, and unrelated untracked files. None was edited or
  staged by this rerun.
- Task 10 and Task 11 remain recorded review-clean in `progress.md`.
- `Tools/flying_car_test/config/experiment.yaml` was not changed. Its SHA-256
  during this run was
  `c682337fc48233c04f075c3046e83cfe470c6dc7483ba2c967c40a07004d5fdf`.
- Unique result root:
  `/home/px4/flying_car_test_results/task12-live-20260731-agent12-01`
- Free disk before launch: 928 GiB on `/dev/sdd`.

### 11.2 Full offline gate

Commands:

```bash
cd /home/px4/real-PX4-Autopilot/PX4-Autopilot
source /opt/ros/noetic/setup.bash
source /home/px4/flying_car_test_ws/devel/setup.bash
export ROS_PACKAGE_PATH=/home/px4/real-PX4-Autopilot/PX4-Autopilot:\
/home/px4/flying_car_test_ws/src:/opt/ros/noetic/share
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=Tools/flying_car_test/src \
  python3 -m pytest Tools/flying_car_test/test -v

bash Tools/flying_car_test/scripts/setup_workspace.sh

find Tools/flying_car_test -type f -name '*.py' -print0 |
  sort -z |
  PYTHONPYCACHEPREFIX=/home/px4/flying_car_test_results/\
task12-live-20260731-agent12-01/offline/pycache \
  xargs -0 python3 -m py_compile

git diff --check
```

Results:

| Gate | Result | Evidence |
|---|---|---|
| Full pytest | PASS: 423 passed in 8.46 s | `offline/pytest-initial.log` |
| Workspace setup / catkin build | PASS, exit 0 | `offline/setup-workspace.log` |
| All package Python files compile | PASS | `offline/py-compile.log` |
| Initial `git diff --check` | PASS, exit 0 | `offline/git-diff-check-initial.log` |

The first Gazebo environment-source probe used `set -u`; the upstream
`setup_gazebo.bash` reads an initially unset `GAZEBO_PLUGIN_PATH`, so that
probe stopped with:

```text
setup_gazebo.bash: line 19: GAZEBO_PLUGIN_PATH: unbound variable
```

No ROS, PX4, MAVROS, or Gazebo process had been started. Re-running without
shell `nounset` loaded the Gazebo plugin/model paths successfully.

The next static launch expansion still failed because an overly narrow
replacement `ROS_PACKAGE_PATH` omitted the Gazebo ROS package:

```text
Resource not found ... mavlink_sitl_gazebo
```

Adding the package's actual directory was sufficient; no file or persistent
shell setting was changed:

```bash
source /opt/ros/noetic/setup.bash
source /home/px4/flying_car_test_ws/devel/setup.bash
source /home/px4/real-PX4-Autopilot/PX4-Autopilot/Tools/simulation/gazebo-classic/setup_gazebo.bash \
  /home/px4/real-PX4-Autopilot/PX4-Autopilot \
  /home/px4/real-PX4-Autopilot/PX4-Autopilot/build/px4_sitl_default
export ROS_PACKAGE_PATH=/home/px4/real-PX4-Autopilot/PX4-Autopilot:\
/home/px4/real-PX4-Autopilot/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic:\
/home/px4/flying_car_test_ws/src:/opt/ros/noetic/share

rospack find flying_car_test
# /home/px4/flying_car_test_ws/src/flying_car_test
rospack find px4
# /home/px4/real-PX4-Autopilot/PX4-Autopilot
rospack find mavlink_sitl_gazebo
# /home/px4/real-PX4-Autopilot/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic

roslaunch --files flying_car_test monitor_only.launch
PYTHONPATH=Tools/flying_car_test/src \
  rosrun flying_car_test run_trials.py --trials 1 --preflight-only \
  --output-root /home/px4/flying_car_test_results/\
task12-live-20260731-agent12-01/preflight-only
```

`roslaunch --files` resolved every include and runner preflight printed
`preflight OK`. Immediately before the live launch, exact-name checks found
no `px4`, `gzserver`, `gzclient`, `roscore`, `rosmaster`, `roslaunch`,
`mavros_node`, or `rosbag` process. `ss -lntup` found no owner on ports
11311, 11345, 14540, 14550, 14580, 4560, or 4561.

### 11.3 Checkpoint 1 command and immediate result

Result directory:

```text
/home/px4/flying_car_test_results/task12-live-20260731-agent12-01/
checkpoint-01-disarmed-health
```

The recorded runner established a new session/process group for this launch,
set the unique `ROS_LOG_DIR` below the checkpoint directory, snapshotted the
existing ULog paths, and executed:

```bash
setsid roslaunch --screen flying_car_test monitor_only.launch \
  gui:=false \
  result_dir:=/home/px4/flying_car_test_results/\
task12-live-20260731-agent12-01/checkpoint-01-disarmed-health \
  rosbag_name:=health-60
```

No controller was launched and no command publisher, arm request, mode
request, MAV_TYPE write, Gazebo pose write, or wheel-joint write was used.
The intended state was disarmed/monitor-only.

Observed sequence:

1. `roslaunch` PID 144 started at `2026-07-31T17:06:15+08:00`.
2. PX4, Gazebo, MAVROS, the passive monitor, the safety guard, and rosbag
   began startup.
3. Before MAVROS-connected/disarmed plus `/clock` readiness could be
   established, the required passive monitor raised an exception.
4. Roslaunch reported the required process death and initiated shutdown.
5. The health runner stopped with rc 30 at
   `2026-07-31T17:06:21+08:00`; the 60-second dwell never started.
6. The roslaunch parent returned 0 after its required-node shutdown, but the
   required process death and runner result make this checkpoint an
   unambiguous FAIL.
7. Post-cleanup exact-name process checks and all guarded-port checks were
   empty.

First application error:

```text
Traceback (most recent call last):
  File ".../experiment_monitor.py", line 10, in <module>
    main()
  File ".../flying_car_test/monitor.py", line 317, in main
    raise ValueError("~launch_arguments must be a mapping")
ValueError: ~launch_arguments must be a mapping
```

Required-process failure:

```text
REQUIRED process [flying_car_experiment_monitor-6] has died!
process has died [pid 270, exit code 1, cmd ...experiment_monitor.py ...]
```

The later `ERROR [px4] Startup script returned with return value: 2` and
`spawn_model` shutdown exception occurred while roslaunch was tearing down
the already-failed stack; they do not precede or replace the monitor
parameter-type failure.

### 11.4 Root-cause boundary

The live traceback and a non-launching parameter dump agree:

```bash
roslaunch --dump-params flying_car_test monitor_only.launch \
  gui:=false result_dir:=<checkpoint-dir>
```

produced:

```text
/flying_car_experiment_monitor/launch_arguments: dict(gui:=false x:=0 y:=30)
```

`Tools/flying_car_test/launch/monitor_only.launch` declares that value with a
plain XML `<param ... value="dict(...)"/>`, so ROS stores a string.
`Tools/flying_car_test/src/flying_car_test/monitor.py` immediately requires
`isinstance(launch_arguments, Mapping)` and raises otherwise. The same
plain-string parameter declaration is present in `full_experiment.launch`.

Therefore neither the mandatory monitor-only entry gate nor the later full
automatic trial can start with the current interface. Repair requires a
change to a launch file or the monitor's parameter contract, both outside
Task 12's two-file allow-list. No command calibration in
`experiment.yaml` can affect this startup type mismatch.

### 11.5 Checkpoint table

| # | Checkpoint | Phase/state/command | Measured result | Verdict |
|---|---|---|---|---|
| 0 | Offline suite/build | No live state or command | 423 tests pass; catkin build, py_compile, and diff check pass | PASS |
| 1 | 60 s disarmed health | Startup, intended disarmed monitor-only; no command | Monitor died before readiness/dwell; no sample or actuator evidence | **FAIL / BLOCKED** |
| 2 | First ground-forward | Not entered | Stopped after checkpoint 1 | NOT RUN |
| 3 | Final-ground turn/diagonal | Not entered | Stopped after checkpoint 1 | NOT RUN |
| 4 | OFFBOARD current-pose hold | Not entered | Stopped after checkpoint 1 | NOT RUN |
| 5 | 1 m AGL takeoff/land | Not entered | Stopped after checkpoint 1 | NOT RUN |
| 6 | 3 m AGL takeoff/land | Not entered | Stopped after checkpoint 1 | NOT RUN |
| 7 | 5 m AGL takeoff/land | Not entered | Stopped after checkpoint 1 | NOT RUN |
| 8 | 5 m AGL + 1 m forward | Not entered | Stopped after checkpoint 1 | NOT RUN |
| 9 | 5 m AGL + 5 m forward + 3 s hover | Not entered | Stopped after checkpoint 1 | NOT RUN |
| 10 | Bounded manual-air + reacquisition | Not entered | Stopped after checkpoint 1 | NOT RUN |
| 11 | One complete automatic trial | Not entered; same launch parameter defect exists | Stopped after checkpoint 1 | NOT RUN |

Because the monitor failed before its `EvidenceRecorder` was constructed and
PX4 shut down before a logger run was established, this checkpoint produced
no `metadata.json`, JSONL, CSV, finalized rosbag, or new ULog. Consequently
the clock-monotonic, continuous MAVROS-connection, and twelve-zero-actuator
requirements are **unmeasured**, not PASS.

### 11.6 Validation and independent GNU grep

The existing validator was run without editing its output:

```bash
PYTHONPATH=Tools/flying_car_test/src \
python3 Tools/flying_car_test/scripts/validate_run.py \
  --result-dir <checkpoint-dir> \
  --config Tools/flying_car_test/config/experiment.yaml \
  --log-rules Tools/flying_car_test/config/log_rules.yaml \
  --output <checkpoint-dir>/validation.json
```

It exited 1 and wrote `overall: FAIL`. It necessarily reports missing
samples and missing ULog. Its fatal count is inflated because it also scans
the saved independent grep output and the `ros_log/latest` symlink; this does
not alter the verdict or the first-error diagnosis.

Independent scan used GNU grep 3.4:

```bash
grep -RInE --include='*.log' \
  'HEARTBEAT timed out|poll timeout|process has died|Segmentation fault|\
lockstep.*timeout|TM: Time jump detected|Traceback \(most recent call last\)|\
\[ERROR\]|\[FATAL\]|(^|[^[:alnum:]_])ERROR([^[:alnum:]_]|$)|\
(^|[^[:alnum:]_])FATAL([^[:alnum:]_]|$)' \
  <checkpoint-dir>
```

It recorded 20 line matches, including duplicate paths through
`ros_log/latest`: the monitor traceback, PX4 shutdown error, ROS master
shutdown tracebacks, and the required-process death. A separate warning scan
recorded five MAVROS mount-plugin default-parameter warnings. Since the
fatal startup failure had already invalidated the checkpoint, those warnings
were not treated as an allow-list decision.

### 11.7 Artifact index and stop decision

Primary artifacts:

- `offline/pytest-initial.log`
- `offline/setup-workspace.log`
- `offline/py-compile.log`
- `offline/git-diff-check-initial.log`
- `offline/ros-environment-check-resolved.log`
- `offline/process-port-check.log`
- `offline/preflight-only.log`
- `checkpoint-01-disarmed-health/health-runner.log`
- `checkpoint-01-disarmed-health/roslaunch.log`
- `checkpoint-01-disarmed-health/ros_log/`
- `checkpoint-01-disarmed-health/first-error.txt`
- `checkpoint-01-disarmed-health/first-error-context.txt`
- `checkpoint-01-disarmed-health/dump-params.yaml`
- `checkpoint-01-disarmed-health/fatal-error-scan.txt`
- `checkpoint-01-disarmed-health/warning-scan.txt`
- `checkpoint-01-disarmed-health/validation.json`
- `checkpoint-01-disarmed-health/post-cleanup-recheck.txt`
- `checkpoint-01-disarmed-health/ulog-before.txt`
- `checkpoint-01-disarmed-health/ulog-after-new.txt` (empty)

Stop condition: first mandatory live checkpoint failed and its minimal repair
is outside Task 12 scope. No config value was adjusted, no acceptance
threshold was relaxed, no validation artifact was edited, no later
checkpoint was run, and no Task 12 completion commit will be created.

### 11.8 Final verification before BLOCKED handoff

Fresh post-report checks:

- Required full command
  `PYTHONPATH=Tools/flying_car_test/src python3 -m pytest
  Tools/flying_car_test/test -v` -> **423 passed in 9.19 s**.
- `git diff --check` -> exit 0.
- Separate no-index whitespace checks for this untracked live record and the
  ignored Task 12 report -> no findings.
- Final config diff -> empty.
- Git status entry set -> identical to the 21-entry pre-run snapshot (the
  content of this already-untracked file changed, but its status entry did
  not).
- Fresh GNU grep scan of actual `*.log` files -> 20 fatal/error line matches
  and 5 warning line matches, preserved as `fatal-error-scan-final.txt` and
  `warning-scan-final.txt`.
- Final exact-name process and guarded-port probes -> empty.
