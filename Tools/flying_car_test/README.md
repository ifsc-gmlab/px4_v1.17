# Flying Car Experiment - Manual QGC and Physical-Transmitter Replication
> 机型说明与实验方法总览（80003/80004/80005）：[AIRFRAMES_AND_EXPERIMENTS.md](AIRFRAMES_AND_EXPERIMENTS.md)

This document is the **operator-facing** runbook for manually replicating
the flying-car end-to-end experiment with one of two real controllers:

* **QGC** (QGroundControl on the workstation): the operator uses the
  configured virtual joystick while watching the simulated vehicle.
* **A physical RC transmitter** (handheld remote controller): the
  operator drives the vehicle through a receiver connected directly
  to PX4, while MAVROS exposes telemetry for evidence.

In both cases the experiment itself is the **passive ``monitor_only``**
ROS launch (no ``experiment_controller``).  The operator's only signal
that reaches the ``experiment_monitor`` and ``safety_guard`` is the
strict seven-marker sequence published by ``stage_marker.py``.  After
``experiment_end`` the script also publishes a terminal ``COMPLETE``
phase string so the passive monitor can close with ``clean=true`` when
the launch is shut down, allowing the validator / reporting pipeline
to produce a clean artifact.

The script enforces a contiguous 1..7 sequence and rejects any
duplicate, out-of-order, or unknown marker with a non-zero exit status
and **no published message**.  The sequence persists across
independent shell invocations through an on-disk state file with an
``fcntl`` exclusive lock, so two overlapping operators cannot both
advance the sequence.  Before publishing, the CLI independently waits
for a subscriber on the stage-marker topic (normally rosbag) and the
phase topic (the passive monitor).  A durable write-ahead record makes
an interrupted invocation retryable without falsely committing an
unpublished marker.

## Overview

The seven canonical stage markers (1-based sequence in parentheses):

| Sequence | Marker                   | Meaning                                              |
| -------- | ------------------------ | ---------------------------------------------------- |
| 1        | ``ground_forward_start`` | Operator is ready to drive forward on the ground     |
| 2        | ``flight_switch_start``  | Operator is switching MAV_TYPE to flight mode        |
| 3        | ``takeoff_start``        | Operator is ready to take off                        |
| 4        | ``air_forward_start``    | Operator is ready to move forward in the air         |
| 5        | ``landing_start``        | Operator is ready to land                            |
| 6        | ``final_ground_start``   | Operator is ready to drive to the left-front         |
| 7        | ``experiment_end``       | Trial is complete; monitor closes on ``COMPLETE``    |

> **Marker timing rule**: publish each marker **BEFORE** its
> corresponding segment begins - never after.  The script refuses
> out-of-order publication, but the operator's responsibility is
> to invoke ``stage_marker.py <marker>`` at the START of each
> stage, not retroactively after the segment has run.  This keeps
> the rosbag evidence aligned with the operator's actual actions.

```bash
rosrun flying_car_test stage_marker.py ground_forward_start
rosrun flying_car_test stage_marker.py flight_switch_start
rosrun flying_car_test stage_marker.py takeoff_start
rosrun flying_car_test stage_marker.py air_forward_start
rosrun flying_car_test stage_marker.py landing_start
rosrun flying_car_test stage_marker.py final_ground_start
rosrun flying_car_test stage_marker.py experiment_end
```

Each command must exit with status ``0`` and print
``published stage marker '<marker>' (sequence N) to /flying_car_test/stage_marker``.
Validation and subscriber failures happen before publication, so those
failures publish nothing.  A transport failure or abrupt process
interruption can happen after one topic has received the event.  In
that case, inspect the error and re-run the **same** marker: the durable
write-ahead record reuses the same sequence, timestamp, and payload.
Do not reset the marker state while a pending marker is being recovered.

After ``monitor_only.launch`` is running, start a fresh trial with
``rosrun flying_car_test stage_marker.py --reset``.  The one-shot form
``STAGE_MARKER_RESET=1 rosrun flying_car_test stage_marker.py`` is
equivalent.  Do not combine either reset form with a marker argument:
reset is a separate operation that clears both the on-disk state and
the public ``/flying_car_test/stage_sequence`` ROS parameter.

## Experiment ManualControl values vs QGC joystick controls

The values in ``config/experiment.yaml`` are
``mavros_msgs/ManualControl``/MAVLink ``MANUAL_CONTROL`` axis units,
not RC PWM microseconds.  MAVLink defines joystick axes in normalized
``-1000..+1000`` units.  QGC presents the same axes as a virtual
joystick, normally as ``-1..+1`` or percentages.  Use the corresponding
percentage and verify the resulting evidence on
``/mavros/manual_control/control`` before moving the vehicle.  The QGC
MAVLink Inspector is observational; it is not a control-entry widget.

| Action                    | Project `ManualControl` values      | QGC virtual joystick equivalent |
| ------------------------- | ----------------------------------- | ------------------------------- |
| Neutral                   | ``x=0, y=0, z=500, r=0``            | axes centred; z at +50%          |
| Ground forward            | ``x=0, y=0, z=650, r=0``            | z at +65%                        |
| Ground left turn          | ``x=0, y=500, z=500, r=0``          | y at +50%; z at +50%             |
| Air forward               | ``x=150, y=0, z=500, r=0``          | x at +15%; z at +50%             |

The exact project values mirror
``Tools/flying_car_test/config/experiment.yaml``:

```yaml
commands:
  manual_neutral: {x: 0.0, y: 0.0, z: 500.0, r: 0.0}
  ground_forward: {x: 0.0, y: 0.0, z: 650.0, r: 0.0}
  ground_left_turn: {x: 0.0, y: 500.0, z: 500.0, r: 0.0}
  air_manual_forward: {x: 150.0, y: 0.0, z: 500.0, r: 0.0}
```

Do not type ``650`` into a normalized QGC field.  Set the virtual
joystick to the equivalent percentage (``65%`` for the project's
``z=650`` command) and confirm the observed ManualControl value before
arming.

## 1. Build and source the workspace

Build the catkin workspace and source every overlay ROS needs:

```bash
cd ~/PX4-Autopilot
make px4_sitl_default
source /opt/ros/noetic/setup.bash
./Tools/flying_car_test/scripts/setup_workspace.sh
source ~/flying_car_test_ws/devel/setup.bash
source ~/PX4-Autopilot/Tools/simulation/gazebo-classic/setup_gazebo.bash \
    "$PWD" "$PWD/build/px4_sitl_default"
```

The ROS source provides rospy, rosmsg, and the MAVROS launch files.
``scripts/setup_workspace.sh`` creates ``~/flying_car_test_ws`` - the
actual catkin workspace the helper initialises - and the next command
sources its devel overlay.  The final source command
(``Tools/simulation/gazebo-classic/setup_gazebo.bash``) exposes the
Gazebo Classic plugins and models using the repository root plus
``build/px4_sitl_default``; the file lives under
``Tools/simulation/gazebo-classic/``, not at the top of ``Tools/``.
Both ``rospack find flying_car_test`` and ``roslaunch flying_car_test
monitor_only.launch`` must succeed; an unresolved ``flying_car_test``
package means the catkin overlay wasn't sourced.

Verify the package is discoverable:

```bash
rospack find flying_car_test
rosrun flying_car_test stage_marker.py --help
```

The ``--help`` output must list all seven canonical markers
(``ground_forward_start``, ``flight_switch_start``, ``takeoff_start``,
``air_forward_start``, ``landing_start``, ``final_ground_start``,
``experiment_end``).  Note: invoking the script with no marker
argument prints help and exits with status ``2`` so a missing marker
is not silently treated as success.

## 2. Launch monitor_only.launch

Launch the passive monitoring session.  This file includes the PX4
SITL + Gazebo Classic stack, the ``experiment_monitor`` (passive
recorder), the ``safety_guard`` (with ``heartbeat_required=false`` so
the operator can drive without the automatic controller's heartbeat),
and a rosbag that records the topics needed for the manual report:

```bash
roslaunch flying_car_test monitor_only.launch gui:=true
```

The launch prints the standard PX4/SITL banner and blocks until you
``Ctrl-C`` it.  Once Gazebo shows the FS-FASMv8_octo model spawned,
proceed to the next step.

> **monitor_only uses ``heartbeat_required=false``.**  This means the
> safety guard does NOT fail-closed just because the
> ``experiment_controller``'s automatic-controller heartbeat is
> absent.  The guard still checks MAVROS connection, attitude,
> altitude ceiling, planar deviation, and stopped-speed limits on
> every cycle; only the explicit automatic-run heartbeat requirement
> is disabled.  If you see the safety guard abort without an
> automatic-controller heartbeat, the cause is something else
> (connection loss, attitude limit, etc.), not the missing heartbeat.

## 3. Verify MAVROS connection and QGC heartbeat

Open QGroundControl and:

1. Enable QGC's UDP AutoConnect and let QGC listen on its normal
   ``14550`` GCS port.  PX4 sends its GCS stream to that port.
   MAVROS uses a separate FCU transport,
   ``udp://:14540@localhost:14580``; port ``14550`` is not the MAVROS
   endpoint.
2. Wait for the QGC vehicle icon to appear; the top status bar must
   show **"Connected"**.  Observe the current ``MAV_TYPE`` without
   requiring it to be 10 yet—the configured airframe normally starts
   at ``MAV_TYPE=14`` (``MAV_TYPE_OCTOROTOR``), and step 4 deliberately
   changes it to 10.
3. In a separate terminal confirm MAVROS is connected:

   ```bash
   rostopic echo /mavros/state -n1
   ```

   The output must include ``connected: True``.

If QGC stays disconnected, restart ``monitor_only.launch`` and re-add
the link.  Do **not** proceed until both QGC and MAVROS report a live
heartbeat.

## 4. Configure ground mode (MAV_TYPE=10)

BEFORE you set MAV_TYPE, confirm ALL of the following preconditions
- skipping any one will arm either with the wrong mode or with the
vehicle still moving:

* the vehicle is **disarmed** (``/mavros/state`` shows
  ``armed: false``);
* the vehicle has been **stopped** for at least
  ``safety.stopped_speed_mps`` (0.15 m/s): observe
  ``/mavros/local_position/velocity_local`` for at least one second
  and confirm ``|v| < 0.15 m/s`` on every axis;
* the manual sticks have been **neutral** (``x=0, y=0, z=500,
  r=0``) for the configured ``safety.neutral_precondition_s`` window.

If the safety guard intervenes during this check, abort and restart
from step 1.

Change the airframe's normal ``MAV_TYPE=14`` (``MAV_TYPE_OCTOROTOR``)
startup value to ``MAV_TYPE_GROUND_ROVER`` (decimal ``10``):

```bash
rosrun mavros mavparam set MAV_TYPE 10
```

Verify in QGC: **Vehicle Setup -> Parameters -> MAV_TYPE** must show
``10``.  Wait for the parameter to persist on the autopilot before
proceeding.

## 5. Manual ground run

**Publish the first marker** immediately before starting step 1:

```bash
rosrun flying_car_test stage_marker.py ground_forward_start
```

The script must print
``published stage marker 'ground_forward_start' (sequence 1) to /flying_car_test/stage_marker``.
A non-zero exit means the marker was rejected or publication did not
complete; inspect the error and re-invoke the same marker when safe.

1. In QGC select the **MANUAL** flight mode (or arm the handheld
   transmitter and verify MANUAL on its mode display).
2. Confirm the vehicle is still **disarmed**.
3. Click **Arm** and wait for QGC to confirm ``armed: true``.
4. Apply the **ground forward** command (``x=0, y=0, z=650,
   r=0``) for the duration configured in
   ``Tools/flying_car_test/config/experiment.yaml``
   (``trajectory.ground_forward_m / velocity.ground_mps``).
5. Neutralize (``x=0, y=0, z=500, r=0``), verify
   ``|velocity| < 0.15 m/s``, and only THEN disarm.
6. **Disarm** before leaving the ground segment.

If the safety guard intervenes (see step 11), abort and restart from
step 1.

## 6. Configure flight mode (MAV_TYPE=14)

Repeat the precondition checklist from step 4 BEFORE changing
MAV_TYPE:

* the vehicle is **disarmed** (``/mavros/state`` reports
  ``armed: false``);
* the vehicle is **stopped** (``/mavros/local_position/velocity_local``
  shows ``|v| < 0.15 m/s`` for at least one second);
* the manual sticks have been **neutral** (``x=0, y=0, z=500,
  r=0``) for the configured window.

> **Note**: the PX4 airframe
> ``80004_gazebo-classic_fs_fasmv8_flying_car`` sets the default
> ``MAV_TYPE`` to ``14`` (``MAV_TYPE_OCTOROTOR`` in the MAVLink
> common.xml enumeration), not ``MAV_TYPE_QUADROTOR`` (``2``).
> The operator must match the airframe default.

Do not retake the safety guard's word - it would already have aborted
the trial if it could not reach this state.

**Publish the flight-switch marker** immediately before changing
``MAV_TYPE``:

```bash
rosrun flying_car_test stage_marker.py flight_switch_start
```

Switch the vehicle to flight mode by setting ``MAV_TYPE`` to
``MAV_TYPE_OCTOROTOR`` (octo-cox airframe flight mode, decimal ``14``):

```bash
rosrun mavros mavparam set MAV_TYPE 14   # MAV_TYPE_OCTOROTOR (decimal 14)
```

Verify in QGC: **Vehicle Setup -> Parameters -> MAV_TYPE** must show
``14``.  Wait for the parameter to persist before proceeding.  Confirm
the vehicle is still disarmed.

## 7. Manual flight run

1. In QGC select the documented flight mode for the manual
   replication:
   * **Take-off**: ``AUTO.LOITER`` (PX4 will command a position hold).
   * **Forward flight**: ``MANUAL`` (you will apply
     ``x=150, y=0, z=500, r=0`` for ~1 s, then neutralize).
   * **Landing**: ``AUTO.LAND``.

**Publish the takeoff marker** before arming and taking off:

```bash
rosrun flying_car_test stage_marker.py takeoff_start
```

2. Arm the vehicle.
3. In ``AUTO.LOITER``, command a controlled take-off to the altitude
   declared in ``config/experiment.yaml``
   (``trajectory.takeoff_agl_m = 5.0``).

**Publish the air-forward marker** before switching to ``MANUAL``:

```bash
rosrun flying_car_test stage_marker.py air_forward_start
```

4. Switch to ``MANUAL`` and apply the **air forward** command
   (``x=150, y=0, z=500, r=0``) for the configured duration
   (``commands.air_manual_duration_s = 1.0``); then neutralize.

5. Hover for ``trajectory.hover_s = 3.0`` seconds.

**Publish the landing marker** before selecting ``AUTO.LAND``:

```bash
rosrun flying_car_test stage_marker.py landing_start
```

6. Switch to ``AUTO.LAND`` and wait for the autopilot to land.

QGC exposes ``OFFBOARD`` mode for the automatic trial only; the manual
trial does **not** enter ``OFFBOARD`` at any point.

## 8. Confirm landed and restore ground MAV_TYPE

After the vehicle touches down and disarms:

1. Confirm ``/mavros/state`` reports ``armed: false``.  Landed state
   is published separately on ``/mavros/extended_state``; confirm its
   ``landed_state`` is ``LANDED_STATE_ON_GROUND``:

   ```bash
   rostopic echo /mavros/state -n1
   rostopic echo /mavros/extended_state -n1
   ```
2. **Do NOT stop the rosbag yet** - the bag must keep recording
   through the final ground segment and the ``experiment_end`` /
   ``COMPLETE`` markers.  Closing the bag prematurely loses the most
   important evidence.  Stop the bag only after ``experiment_end``
   returns 0 (see step 12).
3. Restore ``MAV_TYPE=10`` (ground rover) so the next trial starts in
   the right mode.

BEFORE the next ``rosrun mavros mavparam``, repeat the precondition
check (disarmed, stopped, neutral manual output).

```bash
rosrun mavros mavparam set MAV_TYPE 10   # back to MAV_TYPE_GROUND_ROVER (decimal 10)
```

4. Confirm the parameter persists across a QGC reconnect.

## 9. Drive to the landing point's left-front

After step 8 restored MAV_TYPE=10 (and the precondition check passed):

**Publish the final-ground marker** immediately before starting the
final drive:

```bash
rosrun flying_car_test stage_marker.py final_ground_start
```

1. Select ``MANUAL`` mode again.
2. Arm the vehicle.
3. Drive forward (``x=0, y=0, z=650, r=0``) for the configured
   ``trajectory.final_forward_m = 3.0`` metres.
4. Neutralize and wait for the vehicle to stop (verify
   ``|velocity| < 0.15 m/s``).
5. Drive left-and-forward in a single diagonal segment
   (``x=0, y=500, z=500, r=0``) for ``trajectory.final_left_m = 3.0``
   metres.
6. Neutralize, verify again, then **disarm**.

## 10. Publish each stage marker

The full marker timeline is enforced in chronological order so the
recorded rosbag timeline and the manual report agree with the
operator's actual sequence.  Each marker must be published BEFORE the
segment it labels starts, not retroactively after the segment has run.

> **Why "BEFORE"**: rosbag records the marker's ``stamp`` and payload,
> so a late marker makes the manual evidence inaccurate.  The current
> automated validator and report generator consume monitor artifacts;
> they do not parse ``/flying_car_test/stage_marker``.  Inspect the
> marker topic in the bag separately—the automated PASS result does
> not detect a late or missing manual marker.

The final marker triggers the COMPLETE phase bridge:

```bash
rosrun flying_car_test stage_marker.py experiment_end
```

The ``experiment_end`` marker triggers **two** publish actions:
the regular latched JSON event on
``/flying_car_test/stage_marker`` and an additional pure-string
``"COMPLETE"`` payload on
``/flying_car_experiment_controller/phase``.  The latter is the
event the passive ``experiment_monitor`` listens for to close with
``clean=true`` (so the validator / report generator have a clean
artifact); without it the monitor would leave the trial in an
unclean state even when every operator segment succeeded.

If validation or subscriber checks fail, nothing is published.  A
transport interruption can occur after one of the two topic deliveries.
The durable pending record normally resumes only the missing delivery.
There is an unavoidable at-least-once boundary if the process is
terminated after ROS accepts a publish but before its delivery flag is
saved: re-running can replay the same event.  The replay keeps the same
sequence, timestamp, and payload, which consumers should use as the
event identity.  Common failures are:

* duplicate marker (you ran ``ground_forward_start`` twice);
* out-of-order marker (you skipped ``flight_switch_start``);
* unknown marker (typo);
* no subscriber attached to the rosbag within
  ``SUBSCRIBER_WAIT_TIMEOUT_S`` (5.0 s by default).  The script
  refuses to publish until the rosbag recorder attaches.  Verify
  ``rosnode list`` includes ``/flying_car_rosbag_record`` and
  re-invoke the same marker.
* parameter write failure before publication.  No message is
  published; re-invoke the same marker after the ROS master recovers.
  A final advisory parameter update can fail after the durable disk
  commit; the command still succeeds because disk is canonical.
* publish failure or process interruption.  The script records a
  write-ahead pending marker plus per-topic delivery state so the
  operator can re-invoke the *same* marker.  Normal caught failures
  skip topics already recorded as delivered; the abrupt boundary above
  may replay an identical event.

## 11. Abort on safety conditions

The ``flying_car_safety_guard`` node will publish a non-zero action
the moment any of the following conditions hold:

* **Automatic-controller heartbeat loss**: when
  ``heartbeat_required=true``, no message on
  ``/flying_car_experiment_controller/heartbeat`` for
  ``safety.heartbeat_timeout_s`` (0.3 s) causes an abort.
  ``monitor_only.launch`` sets ``heartbeat_required=false``, so the
  expected absence of that controller heartbeat does not abort this
  manual run.
* **MAVROS/FCU connection loss**: a disconnected MAVROS state is a
  separate ``connection_loss`` safety condition and still aborts the
  manual run.
* **Tilt**: roll or pitch exceeds ``safety.soft_tilt_deg`` (30 deg) -
  guard neutralises; above ``safety.hard_tilt_deg`` (45 deg) the guard
  aborts.
* **Altitude ceiling**: AGL above ``safety.maximum_agl_m`` (8 m).
* **Planar deviation**: more than ``safety.maximum_planar_deviation_m``
  (10 m) from the launch origin.
* **Stopped-speed violation**: vehicle still moving faster than
  ``safety.stopped_speed_mps`` (0.15 m/s) when the operator tries to
  disarm.

When the guard aborts:

1. Stop driving immediately (release sticks, return QGC to ``MANUAL``
   mode if needed).
2. Confirm the vehicle has disarmed via ``/mavros/state``.
3. Inspect the rosbag and ROS console/log output for the abort reason.
4. **Do not** publish any further stage markers - the trial is
   invalid.  Restart from step 1.

If the guard intervention happens in the air, switch to ``AUTO.LAND``
immediately and let the autopilot land the vehicle.

## 12. Validate and generate the manual report

After ``experiment_end`` returns 0, stop the rosbag recorder so its
last record is flushed:

```bash
rosnode kill /flying_car_rosbag_record
```

This sends ``SIGTERM`` to the rosbag recorder so it flushes and exits.
Then press ``Ctrl-C`` in the terminal running
``roslaunch flying_car_test monitor_only.launch`` and wait for the
monitor and safety guard to shut down cleanly.  Do not run validation
while that launch is still active: the monitor writes
``monitor_complete.json`` and finalizes its stream during shutdown.

After shutdown, the result directory
(``/tmp/flying_car_test_result`` by default) contains:

* ``monitor_only.bag`` - the recorded topics, including the
  ``experiment_end`` JSON event (whose ``phase`` field is
  ``"COMPLETE"``) on ``/flying_car_test/stage_marker``.  The launch
  does not record the separate controller-phase topic.
* ``monitor_complete.json`` - the monitor's final report.  ``clean``
  is ``true`` and the last ``phase`` is ``COMPLETE``.

Validate the trial end-to-end with:

```bash
PYTHONPATH=Tools/flying_car_test/src \
    python3 -m flying_car_test.validation \
    --result-dir /tmp/flying_car_test_result
```

The validator must report **overall PASS**.  Generate the human-
readable report with:

```bash
PYTHONPATH=Tools/flying_car_test/src \
    python3 -m flying_car_test.reporting \
    --result-dir /tmp/flying_car_test_result \
    --out-dir /tmp/flying_car_test_report
```

Open ``/tmp/flying_car_test_report/report.md`` and confirm:

* the trial is marked **PASS**;
* the monitor phase timeline reaches ``COMPLETE``;
* the MAV_TYPE values show the documented
  ``10 -> 14 (octo) -> 10`` transitions.

The generated report does not include the seven manual stage markers.
Inspect their chronological evidence directly from the bag:

```bash
rostopic echo -b /tmp/flying_car_test_result/monitor_only.bag \
    /flying_car_test/stage_marker
```

If validation or reporting reports FAIL, the manual trial cannot be
considered a valid replication; inspect the validator output, fix
the underlying issue, and restart from step 1.

## 13. Physical (RC) transmitter equivalent procedure

Operators can drive the same trajectory with a **handheld RC
transmitter** bound directly to the PX4 receiver.  Calibrate the radio
in QGC first, then use its sticks and mode switch for the same segments
above.  The procedure uses calibrated RC channels; it does not reuse
the numeric ManualControl values from ``experiment.yaml`` as PWM.

**Important: ``/mavros/rc/in`` is *telemetry*, not control input.**  The
``RCIOPlugin`` in MAVROS subscribes to the FC's RC receiver channel
state and republishes it on ``/mavros/rc/in`` so a downstream
listener can observe what the FC sees.  **It is not** the topic
the operator publishes to in order to drive the vehicle.

The supported physical path is:

1. Pair the receiver with PX4's RC input and complete QGC radio
   calibration.  Typical channel endpoints are approximately
   ``1000..2000 microseconds`` with centre near ``1500 microseconds``,
   but the calibrated values shown by QGC are authoritative.
2. Before arming, observe ``/mavros/rc/in`` and confirm that centred
   sticks report the calibrated neutral values and that each channel
   moves in the intended direction.
3. Use proportional stick positions corresponding to the percentages
   in the table above; do not transmit values such as ``650
   microseconds``.

``/mavros/manual_control/send`` and ``/mavros/rc/override`` are
software-injection interfaces, not the physical-transmitter path.
When used for diagnostics, ``manual_control/send`` carries normalized
ManualControl axis units.  ``rc/override`` carries
``RC_CHANNELS_OVERRIDE`` values in microseconds: standard endpoints
are 1000 and 2000, 0 releases channels 1-8, and 65535 leaves them
unchanged.  Never use ``0..1000``/``500 neutral`` scaling on
``/mavros/rc/override``.

Subject-specific variations:

* Set the **mode** on the transmitter (often a 3-position switch or a
  rotary) to ``MANUAL`` for ground phases 1-3 and 5-6, and to
  ``AUTO.LOITER`` / ``AUTO.LAND`` for the flight phases.  Confirm the
  selected mode on ``/mavros/state``; RC channel topics do not contain
  mode-name strings.
* ``monitor_only.launch``'s rosbag already records
  ``/mavros/manual_control/control`` (the MAVROS
  ``ManualControlPlugin`` output) and ``/mavros/rc/in`` (the
  ``RCIOPlugin`` output, which is **telemetry** from the FC's RC
  receiver).  The passive monitor does not subscribe to or validate
  either manual-input topic; inspect both topics in the bag as
  separate operator evidence.
* The seven canonical markers, the monitor, the safety guard, the
  rosbag, the result directory, and the validator / report
  generator are exactly the same as for the QGC replication.  Do
  NOT publish markers from a script attached to the transmitter -
  the marker CLI must be invoked manually from a separate shell so
  that the operator (or a designated marker scribe) makes a
  conscious decision for each segment.

The MAV_TYPE preconditions in steps 4 and 6 (disarmed, stopped,
neutral) apply unchanged - the ``safety_guard`` blocks any
unsupported state regardless of the input device.

## Quick reference (QGC and RC transmitter)

```text
sequence 1: ground_forward_start   (QGC z=+65% or calibrated RC equivalent)
sequence 2: flight_switch_start    (MAV_TYPE 10 -> 14 (octo) while disarmed + stopped + neutral)
sequence 3: takeoff_start          (MANUAL -> AUTO.LOITER, arm, take off)
sequence 4: air_forward_start      (MANUAL, x=150 forward, neutralize)
sequence 5: landing_start          (AUTO.LAND)
sequence 6: final_ground_start     (MAV_TYPE 14 (octo) -> 10, MANUAL, final drive)
sequence 7: experiment_end         (validator + report both PASS; phase=COMPLETE)
```

Use QGC virtual-joystick percentages or calibrated physical RC channel
positions.  The MAVLink Inspector is for observation, not control input.
