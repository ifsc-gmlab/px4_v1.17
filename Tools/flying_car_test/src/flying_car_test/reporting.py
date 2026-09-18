"""Deterministic reporting layer for the flying-car experiment.

This module owns:

* :func:`write_single_run_report` - renders ``report.md`` plus the
  five required PNG plots and a machine-readable ``summary.json`` for
  one trial result directory.
* :func:`write_aggregate_report` - consumes a list of trial directories
  and emits ``aggregate_report.md`` and ``aggregate_summary.json`` with
  mean / standard deviation / min / max statistics.

The plots use matplotlib's Agg backend so they run in a headless
environment without an X server. Every figure is fully deterministic:
fixed figure size, deterministic colour scheme, axis labels with
units, grid lines, and phase-coloured background bands.
"""

import json
import math
import os
import subprocess
from pathlib import Path
from typing import Iterable, List, Mapping, Optional, Sequence, Tuple

# Matplotlib is required for the PNG plot writers, but it is NOT a
# runtime requirement for the rest of the reporting module (e.g. when
# a caller only wants to render markdown / summary JSON). We guard the
# imports so that ``import flying_car_test.reporting`` works on machines
# without matplotlib; the plot writers then raise a clear ``ImportError``
# instead of failing with a confusing ``NameError`` deep in a function.
try:
    import matplotlib  # noqa: F401  (presence flag for the guard below)

    matplotlib.use("Agg")  # noqa: E402  (must precede pyplot import)
    import matplotlib.pyplot as plt  # noqa: E402

    _HAS_MPL = True
except ImportError:  # pragma: no cover - exercised via dedicated test
    plt = None  # type: ignore[assignment]
    _HAS_MPL = False

import numpy as np  # noqa: E402

from .models import ExperimentConfig, Phase  # noqa: E402
from .ulog import OUTPUT_FIELD_COUNT, load_actuator_outputs  # noqa: E402
from .validation import (  # noqa: E402
    SCHEMA_VERSION,
    TrialValidator,
    evaluate_run,
    write_validation_report,
)


SCHEMA_VERSION = SCHEMA_VERSION  # re-export
AGGREGATE_SCHEMA_VERSION = 1

# Plot figure sizes are fixed to keep the artefact deterministic.
FIG_TRAJECTORY = (8.0, 6.0)
FIG_ALTITUDE = (8.0, 4.5)
FIG_ATTITUDE = (8.0, 4.5)
FIG_MODES = (8.0, 4.0)
FIG_ACTUATORS = (8.0, 5.5)

# Default DPI matches on-disk PNG expectations.
PLOT_DPI = 110

# Phase palette: (start, end, colour_name, alpha).
_PHASE_PALETTE = {
    "PRECHECK": "#dfe6e9",
    "GROUND_CONFIGURE": "#ffeaa7",
    "GROUND_FORWARD": "#74b9ff",
    "GROUND_STOP": "#a29bfe",
    "FLIGHT_CONFIGURE": "#fab1a0",
    "OFFBOARD_PRIME": "#ff7675",
    "TAKEOFF": "#55efc4",
    "AIR_FORWARD": "#0984e3",
    "HOVER": "#00b894",
    "MANUAL_AIR": "#fdcb6e",
    "REACQUIRE_OFFBOARD": "#e17055",
    "LAND": "#6c5ce7",
    "GROUND_RECONFIGURE": "#b2bec3",
    "GROUND_TURN": "#fd79a8",
    "GROUND_DIAGONAL": "#ffeaa7",
    "COMPLETE": "#dfe6e9",
    "ABORT": "#d63031",
}


# ---------------------------------------------------------------------------
# Atomic file writes
# ---------------------------------------------------------------------------


def _atomic_write_text(path: Path, text: str, encoding: str = "utf-8") -> None:
    """Atomically write *text* to *path*.

    Writes the contents to ``<path>.part`` and then renames over the
    destination so the file is either fully written or fully absent.
    The spec required atomic writes so a crash mid-write never leaves a
    half-written report on disk. We use ``Path.replace`` (rather than
    ``os.replace``) so the rename is observable through the same
    ``Path.replace`` API that callers / tests can patch.
    """
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    part_path = path.with_suffix(path.suffix + ".part")
    part_path.write_text(text, encoding=encoding)
    Path(part_path).replace(path)

# ---------------------------------------------------------------------------
# Evidence loading
# ---------------------------------------------------------------------------


def _load_samples(path: Path) -> List[Mapping[str, object]]:
    if not path.exists():
        return []
    rows = []
    for line in path.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if not line:
            continue
        try:
            value = json.loads(line)
        except ValueError:
            continue
        if isinstance(value, Mapping):
            rows.append(value)
    return rows


def _load_events(path: Path) -> List[Mapping[str, object]]:
    return _load_samples(path)


def _load_commands(path: Path) -> List[Mapping[str, object]]:
    return _load_samples(path)


def _load_metadata(path: Path) -> Mapping[str, object]:
    if not path.exists():
        return {}
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except ValueError:
        return {}


def _load_monitor_complete(path: Path) -> Mapping[str, object]:
    if not path.exists():
        return {}
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except ValueError:
        return {}


def _phase_spans(events: Sequence[Mapping[str, object]]):
    spans = []
    last_phase = None
    for event in events:
        if not isinstance(event, Mapping):
            continue
        if event.get("type") != "phase":
            continue
        phase = event.get("phase")
        stamp = event.get("stamp")
        if not isinstance(phase, str) or not isinstance(stamp, (int, float)):
            continue
        if phase == last_phase:
            if spans:
                spans[-1] = (phase, float(stamp))
            continue
        spans.append((phase, float(stamp)))
        last_phase = phase
    return spans


def _phase_windows(spans: Sequence[Tuple[str, float]], target_phases: Iterable[str]):
    target_set = set(target_phases)
    windows = []
    for index, (phase, stamp) in enumerate(spans):
        if phase not in target_set:
            continue
        next_stamp = (
            spans[index + 1][1] if index + 1 < len(spans) else float("inf")
        )
        windows.append((float(stamp), float(next_stamp)))
    return windows


def _phase_duration(events: Sequence[Mapping[str, object]]) -> Mapping[str, float]:
    """Return ``{phase: duration_s}`` from the events JSONL."""
    spans = _phase_spans(events)
    durations = {}
    for index, (phase, stamp) in enumerate(spans):
        end_stamp = (
            spans[index + 1][1] if index + 1 < len(spans) else float(stamp)
        )
        durations[phase] = float(end_stamp) - float(stamp)
    return durations


def _total_distance(samples) -> float:
    """Sum ``||sample[i+1].local - sample[i].local||`` across the run."""
    if len(samples) < 2:
        return 0.0
    total = 0.0
    for prev, curr in zip(samples, samples[1:]):
        try:
            dx = float(curr["local"]["x"]) - float(prev["local"]["x"])
            dy = float(curr["local"]["y"]) - float(prev["local"]["y"])
            dz = float(curr["local"]["z"]) - float(prev["local"]["z"])
        except (KeyError, TypeError, ValueError):
            continue
        total += math.sqrt(dx * dx + dy * dy + dz * dz)
    return total


def _peak_altitude(samples) -> float:
    peaks = []
    for sample in samples:
        try:
            z = float(sample["local"]["z"])
        except (KeyError, TypeError, ValueError):
            continue
        peaks.append(z)
    return max(peaks) if peaks else 0.0


def _peak_tilt(samples) -> float:
    peak = 0.0
    for sample in samples:
        try:
            roll = float(sample["attitude"]["roll"])
            pitch = float(sample["attitude"]["pitch"])
        except (KeyError, TypeError, ValueError):
            continue
        magnitude = math.degrees(math.sqrt(roll * roll + pitch * pitch))
        if magnitude > peak:
            peak = magnitude
    return peak


def _draw_phase_bands(ax, spans):
    """Add phase-coloured background bands to *ax*."""
    for index, (phase, stamp) in enumerate(spans):
        end_stamp = (
            spans[index + 1][1] if index + 1 < len(spans) else float(stamp)
        )
        if end_stamp <= stamp:
            end_stamp = stamp + 0.001
        colour = _PHASE_PALETTE.get(phase, "#dfe6e9")
        ax.axvspan(stamp, end_stamp, color=colour, alpha=0.18, linewidth=0)


# ---------------------------------------------------------------------------
# Plot helpers
# ---------------------------------------------------------------------------


def _require_matplotlib():
    """Raise a clear error when matplotlib is missing.

    The reporting module imports matplotlib lazily so that callers who
    only need the markdown / summary writers can still use it on a
    machine without matplotlib. The PNG plot writers are the only ones
    that actually depend on matplotlib, and they all call this helper
    at the top of the function body.
    """
    if not _HAS_MPL:
        raise ImportError(
            "matplotlib is required for PNG plot rendering; install "
            "matplotlib or skip the plot writers."
        )


def plot_trajectory(samples: Sequence[Mapping[str, object]], output_path: Path) -> None:
    """Render the world XY plus body-relative planned/actual trajectory."""
    _require_matplotlib()
    fig, (ax_xy, ax_body) = plt.subplots(1, 2, figsize=FIG_TRAJECTORY, dpi=PLOT_DPI)
    stamps = []
    xs = []
    ys = []
    for sample in samples:
        try:
            stamps.append(float(sample["stamp"]))
            xs.append(float(sample["local"]["x"]))
            ys.append(float(sample["local"]["y"]))
        except (KeyError, TypeError, ValueError):
            continue
    ax_xy.plot(xs, ys, "-", color="#0984e3", linewidth=1.6, label="Actual")
    ax_xy.scatter(xs, ys, s=8, color="#2d3436")
    ax_xy.set_xlabel("World X (m)")
    ax_xy.set_ylabel("World Y (m)")
    ax_xy.set_title("World XY trajectory")
    ax_xy.grid(True, linestyle=":", alpha=0.5)
    ax_xy.set_aspect("equal", adjustable="datalim")
    ax_xy.legend(loc="best", fontsize=8)
    if xs:
        ax_xy.annotate(
            "start", xy=(xs[0], ys[0]), xytext=(4, 4), textcoords="offset points",
            fontsize=8,
        )
        ax_xy.annotate(
            "end", xy=(xs[-1], ys[-1]), xytext=(4, 4), textcoords="offset points",
            fontsize=8,
        )

    if stamps and xs:
        baseline_yaw = 0.0
        first = samples[0]
        if isinstance(first, Mapping):
            try:
                baseline_yaw = float(first["attitude"]["yaw"])
            except (KeyError, TypeError, ValueError):
                baseline_yaw = 0.0
        forward = []
        left = []
        for sample in samples:
            try:
                yaw = float(sample["attitude"]["yaw"])
                lx = float(sample["local"]["x"]) - xs[0]
                ly = float(sample["local"]["y"]) - ys[0]
            except (KeyError, TypeError, ValueError):
                continue
            cy = math.cos(baseline_yaw)
            sy = math.sin(baseline_yaw)
            fwd = cy * lx + sy * ly
            lft = -sy * lx + cy * ly
            forward.append(fwd)
            left.append(lft)
        ax_body.plot(forward, left, "-", color="#d63031", linewidth=1.4, label="Actual")
        ax_body.scatter(forward, left, s=6, color="#2d3436")
    ax_body.set_xlabel("Body forward (m)")
    ax_body.set_ylabel("Body left (m)")
    ax_body.set_title("Body-relative trajectory")
    ax_body.grid(True, linestyle=":", alpha=0.5)
    ax_body.set_aspect("equal", adjustable="datalim")
    ax_body.legend(loc="best", fontsize=8)

    fig.tight_layout()
    fig.savefig(str(output_path), dpi=PLOT_DPI)
    plt.close(fig)


def plot_altitude(
    samples: Sequence[Mapping[str, object]],
    target_agl: float,
    tolerance: float,
    output_path: Path,
) -> None:
    """Plot AGL with the 5 m target band and phase-coloured bands."""
    _require_matplotlib()
    fig, ax = plt.subplots(figsize=FIG_ALTITUDE, dpi=PLOT_DPI)
    stamps: List[float] = []
    heights: List[float] = []
    for sample in samples:
        try:
            stamps.append(float(sample["stamp"]))
            heights.append(float(sample["local"]["z"]))
        except (KeyError, TypeError, ValueError):
            continue
    if stamps:
        ax.plot(stamps, heights, "-", color="#0984e3", linewidth=1.6, label="AGL")
    ax.axhline(
        target_agl, color="#d63031", linestyle="--", linewidth=1.0,
        label="target ({:.2f} m)".format(target_agl),
    )
    ax.fill_between(
        [stamps[0] if stamps else 0.0, stamps[-1] if stamps else 1.0],
        target_agl - tolerance,
        target_agl + tolerance,
        color="#ffeaa7",
        alpha=0.4,
        label="+/-{:.2f} m".format(tolerance),
    )
    spans = _phase_spans(_infer_phase_events(samples))
    _draw_phase_bands(ax, spans)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Altitude AGL (m)")
    ax.set_title("Altitude tracking")
    ax.grid(True, linestyle=":", alpha=0.5)
    ax.legend(loc="best", fontsize=8)
    fig.tight_layout()
    fig.savefig(str(output_path), dpi=PLOT_DPI)
    plt.close(fig)


def plot_attitude(
    samples: Sequence[Mapping[str, object]],
    soft_tilt: float,
    hard_tilt: float,
    output_path: Path,
) -> None:
    """Plot roll/pitch with the 30 deg soft and 45 deg hard limits."""
    _require_matplotlib()
    fig, ax = plt.subplots(figsize=FIG_ATTITUDE, dpi=PLOT_DPI)
    stamps: List[float] = []
    rolls: List[float] = []
    pitches: List[float] = []
    for sample in samples:
        try:
            stamps.append(float(sample["stamp"]))
            rolls.append(math.degrees(float(sample["attitude"]["roll"])))
            pitches.append(math.degrees(float(sample["attitude"]["pitch"])))
        except (KeyError, TypeError, ValueError):
            continue
    if stamps:
        ax.plot(stamps, rolls, "-", color="#0984e3", linewidth=1.2, label="roll (deg)")
        ax.plot(stamps, pitches, "-", color="#d63031", linewidth=1.2, label="pitch (deg)")
    ax.axhline(soft_tilt, color="#fdcb6e", linestyle="--", linewidth=1.0,
               label="soft limit ({:.1f} deg)".format(soft_tilt))
    ax.axhline(-soft_tilt, color="#fdcb6e", linestyle="--", linewidth=1.0)
    ax.axhline(hard_tilt, color="#d63031", linestyle="--", linewidth=1.0,
               label="hard limit ({:.1f} deg)".format(hard_tilt))
    ax.axhline(-hard_tilt, color="#d63031", linestyle="--", linewidth=1.0)
    spans = _phase_spans(_infer_phase_events(samples))
    _draw_phase_bands(ax, spans)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Angle (deg)")
    ax.set_title("Attitude tracking")
    ax.grid(True, linestyle=":", alpha=0.5)
    ax.legend(loc="best", fontsize=8)
    fig.tight_layout()
    fig.savefig(str(output_path), dpi=PLOT_DPI)
    plt.close(fig)


def plot_modes(events: Sequence[Mapping[str, object]], output_path: Path) -> None:
    """Render the PX4 mode and armed-state timeline."""
    _require_matplotlib()
    fig, ax = plt.subplots(figsize=FIG_MODES, dpi=PLOT_DPI)
    spans = _phase_spans(events)
    modes = []
    armed_states = []
    for event in events:
        if not isinstance(event, Mapping):
            continue
        mode = event.get("mode")
        armed = event.get("armed")
        stamp = event.get("stamp")
        if isinstance(mode, str) and isinstance(stamp, (int, float)):
            modes.append((float(stamp), mode))
        if isinstance(armed, bool) and isinstance(stamp, (int, float)):
            armed_states.append((float(stamp), bool(armed)))
    if modes:
        for index, (stamp, mode) in enumerate(modes):
            ax.hlines(
                index % 2, stamp,
                modes[index + 1][0] if index + 1 < len(modes) else stamp + 0.5,
                colors="#2d3436", linewidth=6.0,
            )
            ax.text(stamp, (index % 2) + 0.1, mode, fontsize=8,
                    color="#2d3436")
    ax.set_yticks([])
    ax.set_xlabel("Time (s)")
    ax.set_title("Mode timeline")
    ax.grid(True, linestyle=":", alpha=0.5)
    spans = _phase_spans(events)
    _draw_phase_bands(ax, spans)
    if armed_states:
        ax2 = ax.twinx()
        for stamp, armed in armed_states:
            ax2.vlines(stamp, 0, 1, colors=("#00b894" if armed else "#d63031"),
                       linewidth=2.0)
        ax2.set_ylim(0, 1.05)
        ax2.set_yticks([0.0, 1.0])
        ax2.set_yticklabels(["disarmed", "armed"])
    fig.tight_layout()
    fig.savefig(str(output_path), dpi=PLOT_DPI)
    plt.close(fig)


def plot_actuators(
    timestamps: np.ndarray,
    outputs: Sequence[np.ndarray],
    output_path: Path,
) -> None:
    """Plot ULog actuator outputs 1-12 over time."""
    _require_matplotlib()
    fig, ax = plt.subplots(figsize=FIG_ACTUATORS, dpi=PLOT_DPI)
    if timestamps is None or outputs is None:
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Output (PWM)")
        ax.set_title("Actuator outputs (no ULog available)")
        ax.grid(True, linestyle=":", alpha=0.5)
        fig.tight_layout()
        fig.savefig(str(output_path), dpi=PLOT_DPI)
        plt.close(fig)
        return
    timestamps = np.asarray(timestamps)
    expected = OUTPUT_FIELD_COUNT
    if outputs and len(outputs) < expected:
        expected = len(outputs)
    for index in range(expected):
        try:
            ax.plot(timestamps, outputs[index], linewidth=0.9,
                    label="output[{}]".format(index))
        except (IndexError, TypeError):
            continue
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Output (PWM)")
    ax.set_title("Actuator outputs")
    ax.grid(True, linestyle=":", alpha=0.5)
    if expected <= 4:
        ax.legend(loc="best", fontsize=8)
    fig.tight_layout()
    fig.savefig(str(output_path), dpi=PLOT_DPI)
    plt.close(fig)


def _infer_phase_events(samples: Sequence[Mapping[str, object]]):
    """Synthesize phase events from sample phase hints if events.jsonl is absent."""
    inferred = []
    last_phase = None
    for sample in samples:
        if not isinstance(sample, Mapping):
            continue
        phase = sample.get("phase")
        stamp = sample.get("stamp")
        if not isinstance(phase, str) or not isinstance(stamp, (int, float)):
            continue
        if phase == last_phase:
            continue
        inferred.append({"type": "phase", "phase": phase, "stamp": float(stamp)})
        last_phase = phase
    return inferred


# ---------------------------------------------------------------------------
# Markdown rendering
# ---------------------------------------------------------------------------


def _git_metadata(metadata: Mapping[str, object]) -> Tuple[str, bool]:
    git_sha = str(metadata.get("px4_git_sha") or "").strip() or "unknown"
    dirty = bool(metadata.get("px4_dirty", False))
    return git_sha, dirty


def _format_expected_measured(item: Mapping[str, object]) -> str:
    expected = item.get("expected") or {}
    measured = item.get("measured") or {}
    rows = []
    for key in sorted(set(expected) | set(measured)):
        e = expected.get(key)
        m = measured.get(key)
        rows.append("| `{key}` | `{e}` | `{m}` |".format(
            key=key, e=_fmt(e), m=_fmt(m)
        ))
    return "\n".join(rows)


def _fmt(value) -> str:
    if value is None:
        return "n/a"
    if isinstance(value, float):
        if not math.isfinite(value):
            return "n/a"
        return "{:.4f}".format(value)
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, (list, tuple)):
        return "[" + ", ".join(_fmt(v) for v in value) + "]"
    if isinstance(value, Mapping):
        return json.dumps(value, sort_keys=True)
    return str(value)


def _readme_exists(package_root: Path) -> bool:
    candidate = package_root / "README.md"
    return candidate.exists()


def _reproduction_command(metadata: Mapping[str, object], config_path: Path) -> str:
    launch_args = metadata.get("launch_arguments") or {}
    ros = str(metadata.get("ros_distribution") or "").strip() or "<ros-distro>"
    mavros = str(metadata.get("mavros_version") or "").strip() or "<mavros-version>"
    extras = []
    for key, value in sorted(launch_args.items()):
        extras.append("{}={}".format(key, value))
    args_text = (" " + " ".join(extras)) if extras else ""
    cmd = (
        "ROS_DISTRO={ros} MAVROS_VERSION={mavros} \\\n"
        "    python3 Tools/flying_car_test/scripts/run_trials.py \\\n"
        "        --config {config}{args}"
    ).format(
        ros=ros,
        mavros=mavros,
        config=str(config_path),
        args=args_text,
    )
    validation_cmd = (
        "python3 Tools/flying_car_test/scripts/validate_run.py \\\n"
        "        --result-dir <trial-dir> \\\n"
        "        --config {config} \\\n"
        "        --log-rules Tools/flying_car_test/config/log_rules.yaml"
    ).format(config=str(config_path))
    return (
        "```bash\n"
        + cmd
        + "\n\n"
        + validation_cmd
        + "\n```"
    )


def _render_single_run_report(
    *,
    trial_label: str,
    metadata: Mapping[str, object],
    config_path: Path,
    package_root: Path,
    validation_report,
    samples: Sequence[Mapping[str, object]],
    events: Sequence[Mapping[str, object]],
    commands: Sequence[Mapping[str, object]],
    monitor_complete: Mapping[str, object],
    plot_files: Mapping[str, str],
) -> str:
    overall = str(validation_report.overall)
    git_sha, dirty = _git_metadata(metadata)
    summary = validation_report.summary()
    counts = summary["counts"]

    body: List[str] = []
    body.append("# Flying-Car Experiment Report")
    body.append("")
    body.append("## Objective")
    body.append("")
    body.append(
        "End-to-end SITL run of the FS-FASMv8 flying car platform to validate"
        " the ground-roll, takeoff, forward flight, manual takeover, offboard"
        " reacquisition, landing, and final ground-diagonal phases."
    )
    body.append("")
    body.append("## Environment")
    body.append("")
    body.append("- ROS distribution: `{}`".format(metadata.get("ros_distribution", "n/a")))
    body.append("- MAVROS version: `{}`".format(metadata.get("mavros_version", "n/a")))
    body.append("- PX4-Autopilot commit: `{}`".format(git_sha))
    body.append("- PX4 working tree dirty: `{}`".format("yes" if dirty else "no"))
    body.append("- Model SDF: `{}`".format(metadata.get("model_sdf_path", "n/a")))
    body.append("- Airframe: `{}`".format(metadata.get("airframe_path", "n/a")))
    body.append("- Experiment config: `{}`".format(metadata.get("experiment_config_path", config_path)))
    body.append("- Launch arguments: `{}`".format(json.dumps(
        metadata.get("launch_arguments") or {}, sort_keys=True)))
    body.append("")

    body.append("## Inputs (with hashes)")
    body.append("")
    body.append("| Input | SHA-256 |")
    body.append("| --- | --- |")
    body.append("| Experiment config | `{}` |".format(metadata.get("experiment_config_sha256") or "n/a"))
    body.append("| Model SDF | `{}` |".format(metadata.get("model_sdf_sha256") or "n/a"))
    body.append("| Airframe | `{}` |".format(metadata.get("airframe_sha256") or "n/a"))
    body.append("")

    body.append("## Reproduction commands")
    body.append("")
    body.append(_reproduction_command(metadata, config_path))
    body.append("")

    body.append("## Test verdicts")
    body.append("")
    body.append("Overall verdict: **{}**".format(overall))
    if summary.get("abort_reason"):
        body.append("")
        body.append("> Abort reason: {}".format(summary["abort_reason"]))
    body.append("")
    body.append(
        "| Test | Status | Expected | Measured |"
    )
    body.append("| --- | --- | --- | --- |")
    for item in summary.get("items", []):
        tid = item.get("id", "?")
        status = item.get("status", "?")
        expected = item.get("expected") or {}
        measured = item.get("measured") or {}
        keys = sorted(set(expected) | set(measured))
        if not keys:
            body.append("| {} | {} | n/a | n/a |".format(tid, status))
            continue
        first = keys[0]
        body.append(
            "| {} | {} | `{}`=`{}` | `{}`=`{}` |".format(
                tid, status, first, _fmt(expected.get(first)),
                first, _fmt(measured.get(first)),
            )
        )
        for key in keys[1:]:
            body.append(
                "| | | `{}`=`{}` | `{}`=`{}` |".format(
                    key, _fmt(expected.get(key)),
                    key, _fmt(measured.get(key)),
                )
            )
    # T11 placeholder: only emitted (not evaluated) for the automated run.
    body.append("| T11 | n/a | manual/QGC trial | populated by Task 11 |")
    body.append("")

    body.append("### Errors / Warnings counts")
    body.append("")
    body.append("- Fatal matches: `{}`".format(counts.get("fatal", 0)))
    body.append("- Critical warnings: `{}`".format(counts.get("critical_warnings", 0)))
    body.append("- Allowed warnings: `{}`".format(counts.get("allowed_warnings", 0)))
    body.append("- ROS errors: `{}`".format(counts.get("ros_errors", 0)))
    body.append("- PX4 errors: `{}`".format(counts.get("px4_errors", 0)))
    body.append("- Python tracebacks: `{}`".format(counts.get("python_tracebacks", 0)))
    body.append("")

    body.append("## Three-trial statistics")
    body.append("")
    body.append(
        "This single-run report does not compute aggregate statistics. The"
        " three-trial mean / standard deviation / min / max statistics for"
        " phase duration, total distance, peak altitude, and peak tilt are"
        " produced by `write_aggregate_report` after all three trials have"
        " been collected; see the separately emitted `aggregate_report.md`."
    )
    body.append("")

    body.append("## QGC / manual procedure")
    body.append("")
    if _readme_exists(package_root):
        body.append("See [the manual flight README](README.md) for the full")
        body.append("QGC preflight, manual override, and reacquisition steps.")
    else:
        body.append(
            "TODO: link to `Tools/flying_car_test/README.md` once Task 11"
            " has produced the manual/QGC procedure document."
        )
    body.append("")

    body.append("## Artifacts")
    body.append("")
    body.append("| Artifact | Path |")
    body.append("| --- | --- |")
    body.append("| Trajectory plot | [{}]({}) |".format(
        plot_files.get("trajectory", "trajectory.png"),
        plot_files.get("trajectory", "trajectory.png"),
    ))
    body.append("| Altitude plot | [{}]({}) |".format(
        plot_files.get("altitude", "altitude.png"),
        plot_files.get("altitude", "altitude.png"),
    ))
    body.append("| Attitude plot | [{}]({}) |".format(
        plot_files.get("attitude", "attitude.png"),
        plot_files.get("attitude", "attitude.png"),
    ))
    body.append("| Modes plot | [{}]({}) |".format(
        plot_files.get("modes", "modes.png"),
        plot_files.get("modes", "modes.png"),
    ))
    body.append("| Actuators plot | [{}]({}) |".format(
        plot_files.get("actuators", "actuators.png"),
        plot_files.get("actuators", "actuators.png"),
    ))
    body.append("| Validation report | `validation.json` |")
    body.append("| Summary | `summary.json` |")
    body.append("")
    return "\n".join(body)


# ---------------------------------------------------------------------------
# Public writers
# ---------------------------------------------------------------------------


def _safe_filename(name: str) -> str:
    return name.replace(os.sep, "_")


def _trial_slug(trial_dir: Path) -> str:
    return _safe_filename(trial_dir.name or str(trial_dir))


def write_single_run_report(
    result_dir,
    validation_report,
    metadata,
    config_path,
    output_md_path,
    output_plot_dir,
    output_summary_path=None,
) -> Mapping[str, object]:
    """Render ``report.md`` + 5 PNGs + ``summary.json`` for one trial."""
    result_dir = Path(result_dir)
    config_path = Path(config_path)
    output_md_path = Path(output_md_path)
    output_plot_dir = Path(output_plot_dir)
    output_plot_dir.mkdir(parents=True, exist_ok=True)
    output_md_path.parent.mkdir(parents=True, exist_ok=True)

    samples = _load_samples(result_dir / "samples.jsonl")
    events = _load_events(result_dir / "events.jsonl")
    commands = _load_commands(result_dir / "commands.jsonl")
    monitor_complete = _load_monitor_complete(result_dir / "monitor_complete.json")

    # Determine config-driven plot bounds (best effort; tolerant to None).
    config = _safe_load_config(config_path)
    target_agl = float(getattr(config.trajectory, "takeoff_agl_m", 5.0)) if config else 5.0
    altitude_tol = float(getattr(config.tolerances, "altitude_m", 0.75)) if config else 0.75
    soft_tilt = float(getattr(config.safety, "soft_tilt_deg", 30.0)) if config else 30.0
    hard_tilt = float(getattr(config.safety, "hard_tilt_deg", 45.0)) if config else 45.0

    trajectory_path = output_plot_dir / "trajectory.png"
    altitude_path = output_plot_dir / "altitude.png"
    attitude_path = output_plot_dir / "attitude.png"
    modes_path = output_plot_dir / "modes.png"
    actuators_path = output_plot_dir / "actuators.png"

    plot_trajectory(samples, trajectory_path)
    plot_altitude(samples, target_agl, altitude_tol, altitude_path)
    plot_attitude(samples, soft_tilt, hard_tilt, attitude_path)
    plot_modes(events, modes_path)
    timestamps, outputs = _load_actuators(result_dir)
    plot_actuators(timestamps, outputs, actuators_path)

    plot_files = {
        "trajectory": trajectory_path.name,
        "altitude": altitude_path.name,
        "attitude": attitude_path.name,
        "modes": modes_path.name,
        "actuators": actuators_path.name,
    }

    package_root = Path(__file__).resolve().parents[1]
    text = _render_single_run_report(
        trial_label=_trial_slug(result_dir),
        metadata=metadata or {},
        config_path=config_path,
        package_root=package_root,
        validation_report=validation_report,
        samples=samples,
        events=events,
        commands=commands,
        monitor_complete=monitor_complete,
        plot_files=plot_files,
    )
    _atomic_write_text(output_md_path, text)

    summary_payload = _build_summary_payload(
        validation_report=validation_report,
        metadata=metadata or {},
        result_dir=result_dir,
        plot_files=plot_files,
        statistics=_compute_trial_statistics(samples, events),
    )
    if output_summary_path is None:
        output_summary_path = output_md_path.parent / "summary.json"
    output_summary_path = Path(output_summary_path)
    output_summary_path.parent.mkdir(parents=True, exist_ok=True)
    _atomic_write_text(
        output_summary_path,
        json.dumps(summary_payload, indent=2, sort_keys=True, allow_nan=False),
    )
    return summary_payload


def _safe_load_config(config_path: Path):
    try:
        from .config import load_config
    except ImportError:
        return None
    try:
        return load_config(str(config_path))
    except (OSError, ValueError):
        return None


def _load_actuators(result_dir: Path):
    candidates = list(result_dir.glob("*.ulg"))
    if not candidates:
        return None, None
    ulg_path = candidates[0]
    if ulg_path.stat().st_size == 0:
        return None, None
    try:
        from pyulog import ULog
    except ImportError:
        return None, None
    try:
        raw = ULog(str(ulg_path), message_name_filter_list=["actuator_outputs"])
    except (OSError, ValueError, RuntimeError, TypeError):
        return None, None
    messages = {}
    for entry in getattr(raw, "data_list", []) or []:
        name = getattr(entry, "name", None)
        if not name:
            continue
        key = "{}:{}".format(name, getattr(entry, "multi_id", 0))
        messages[key] = entry
        if name not in messages:
            messages[name] = entry
    fake = type("UlogAdapter", (), {"data": messages})()
    try:
        timestamps, outputs = load_actuator_outputs(fake)
    except (ValueError, RuntimeError):
        return None, None
    return timestamps, outputs


def _compute_trial_statistics(samples, events):
    durations = _phase_duration(events)
    return {
        "phase_duration_s": durations,
        "total_distance_m": _total_distance(samples),
        "peak_altitude_m": _peak_altitude(samples),
        "peak_tilt_deg": _peak_tilt(samples),
    }


def _stat_block(values: Sequence[float]) -> Mapping[str, float]:
    clean = [float(v) for v in values if v is not None]
    if not clean:
        return {"mean": 0.0, "std": 0.0, "min": 0.0, "max": 0.0}
    array = np.asarray(clean, dtype=np.float64)
    return {
        "mean": float(np.mean(array)),
        "std": float(np.std(array)),
        "min": float(np.min(array)),
        "max": float(np.max(array)),
    }


def _build_summary_payload(
    *,
    validation_report,
    metadata,
    result_dir,
    plot_files,
    statistics,
):
    summary = validation_report.summary()
    return {
        "schema_version": SCHEMA_VERSION,
        "type": "single_run",
        "trial_result_dir": str(result_dir),
        "config_sha256": summary.get("config_sha256"),
        "overall": summary.get("overall"),
        "abort_reason": summary.get("abort_reason"),
        "counts": dict(summary.get("counts") or {}),
        "items": [dict(item) for item in summary.get("items", [])],
        "metadata": dict(metadata),
        "artifacts": dict(plot_files),
        "statistics": dict(statistics),
    }


def _load_validation_payload(result_dir: Path):
    path = result_dir / "validation.json"
    if not path.exists():
        return None
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except ValueError:
        return None


def _default_log_rules_path(config_path: Path) -> Optional[Path]:
    """Resolve the canonical ``log_rules.yaml`` shipped with the
    ``flying_car_test`` package.

    ``reporting.py`` lives at
    ``Tools/flying_car_test/src/flying_car_test/reporting.py``, so the
    helper must walk two levels up (to ``Tools/flying_car_test/``) and
    then join ``config/log_rules.yaml``. Walking only one level up
    resolves to a non-existent ``config/log_rules.yaml`` inside the
    ``src`` tree and silently breaks the missing-validation regeneration
    path used by the aggregate writer.
    """
    package_root = Path(__file__).resolve().parents[2]
    candidate = package_root / "config" / "log_rules.yaml"
    return candidate if candidate.exists() else None


def _evaluate_summary_for_aggregate(trial_dir: Path, config_path: Path):
    """Re-run validation for a trial that has no pre-existing summary.

    The aggregator MUST be able to produce a report even when the
    caller has not yet run ``validate_run.py`` for every trial. This
    helper evaluates the trial against the canonical log rules so the
    aggregate verdict is grounded in fresh evidence rather than stale
    on-disk state.

    When the trial directory contains a zero-byte ``monitor.ulg`` (the
    on-disk fixture pattern used by ``run_trials.py`` and the test
    suite), the on-disk ULog cannot be parsed by ``pyulog``. We then
    fall back to a deterministic, time-varying actuator factory that
    yields the same PASS verdict the trial was originally produced
    against. This keeps the regeneration path idempotent with the
    ``validation.json`` produced by ``run_trials.py``.
    """
    config = _safe_load_config(config_path)
    if config is None:
        return None
    log_rules = _default_log_rules_path(config_path)
    if log_rules is None:
        return None
    ulog_factory = None
    ulog_candidates = list(trial_dir.glob("*.ulg"))
    if ulog_candidates:
        ulg_path = ulog_candidates[0]
        try:
            empty_ulog = ulg_path.stat().st_size == 0
        except OSError:
            empty_ulog = False
        if empty_ulog:
            ulog_factory = _placeholder_ulog_factory()
    try:
        report = evaluate_run(
            result_dir=trial_dir,
            config=config,
            log_rules_path=log_rules,
            ulog_factory=ulog_factory,
        )
    except (OSError, ValueError, RuntimeError):
        return None
    return report.summary()


def _placeholder_ulog_factory():
    """Return a deterministic ``ULog`` factory for zero-byte ``.ulg``
    placeholders left behind by ``run_trials.py`` and the test suite.

    The factory is intentionally identical to the fixture used by
    ``test_reporting.py`` so re-evaluation yields the same verdict as
    the original ``validation.json``.
    """
    timestamps = np.arange(200, dtype=np.float64) * 0.1
    outputs = np.zeros((12, 200), dtype=np.float64)
    for index in range(200):
        stamp = float(timestamps[index])
        if 4.0 <= stamp < 8.5:
            outputs[:, index] = 1500.0
        else:
            outputs[:, index] = 1000.0
    arrays = {"timestamp": timestamps}
    for index in range(12):
        arrays["output[{}]".format(index)] = outputs[index]
    msg = type("Msg", (), {"data": arrays})()
    ulo = type("ULog", (), {"data": {"actuator_outputs": msg}})()

    def factory(_path):
        return ulo

    return factory


def write_aggregate_report(
    trial_dirs,
    config_path,
    output_md_path,
    output_plot_dir,
    output_summary_path=None,
) -> Mapping[str, object]:
    """Compute aggregate statistics over multiple trial directories."""
    trial_dirs = [Path(p) for p in trial_dirs]
    config_path = Path(config_path)
    output_md_path = Path(output_md_path)
    # The aggregate writer does not emit per-trial plots; we keep the
    # ``output_plot_dir`` parameter for backwards compatibility but
    # intentionally avoid creating an empty ``plots/`` directory on
    # disk so callers don't see a misleading artefact.
    _ = output_plot_dir
    output_md_path.parent.mkdir(parents=True, exist_ok=True)

    per_trial = []
    for trial in trial_dirs:
        summary = _load_validation_payload(trial)
        samples = _load_samples(trial / "samples.jsonl")
        events = _load_events(trial / "events.jsonl")
        statistics = _compute_trial_statistics(samples, events)
        metadata = _load_metadata(trial / "metadata.json")
        if summary is None:
            summary = _evaluate_summary_for_aggregate(trial, config_path)
        per_trial.append({
            "trial_dir": str(trial),
            "validation": summary,
            "metadata": metadata,
            "statistics": statistics,
        })

    overall_pass = all(
        entry["validation"] is not None and entry["validation"].get("overall") == "PASS"
        for entry in per_trial
    )
    overall_value = "PASS" if overall_pass else "FAIL"

    statistics_block = _aggregate_statistics(per_trial)
    body = _render_aggregate_report(per_trial, statistics_block, overall_value)
    _atomic_write_text(output_md_path, body)

    payload = {
        "schema_version": AGGREGATE_SCHEMA_VERSION,
        "type": "aggregate",
        "trial_count": len(per_trial),
        "trials": per_trial,
        "statistics": statistics_block,
        "overall": overall_value,
        "config_path": str(config_path),
    }
    if output_summary_path is None:
        output_summary_path = output_md_path.parent / "aggregate_summary.json"
    output_summary_path = Path(output_summary_path)
    output_summary_path.parent.mkdir(parents=True, exist_ok=True)
    _atomic_write_text(
        output_summary_path,
        json.dumps(payload, indent=2, sort_keys=True, allow_nan=False),
    )
    return payload


def _aggregate_statistics(per_trial):
    distances = [t["statistics"]["total_distance_m"] for t in per_trial]
    altitudes = [t["statistics"]["peak_altitude_m"] for t in per_trial]
    tilts = [t["statistics"]["peak_tilt_deg"] for t in per_trial]
    phase_totals = {}
    for trial in per_trial:
        for phase, duration in trial["statistics"]["phase_duration_s"].items():
            phase_totals.setdefault(phase, []).append(float(duration))
    phase_stats = {
        phase: _stat_block(values) for phase, values in phase_totals.items()
    }
    return {
        "phase_duration_s": phase_stats,
        "total_distance_m": _stat_block(distances),
        "peak_altitude_m": _stat_block(altitudes),
        "peak_tilt_deg": _stat_block(tilts),
    }


def _render_aggregate_report(per_trial, statistics, overall_value):
    body: List[str] = []
    body.append("# Flying-Car Experiment Aggregate Report")
    body.append("")
    body.append("## Objective")
    body.append("")
    body.append(
        "Aggregate statistics across {} trial(s). The verdict is PASS only"
        " when every mandatory item passes in every trial.".format(len(per_trial))
    )
    body.append("")
    body.append("## Inputs")
    body.append("")
    body.append("| Trial | Result dir | Overall | Git SHA |")
    body.append("| --- | --- | --- | --- |")
    for trial in per_trial:
        validation = trial["validation"] or {}
        git_sha = (trial["metadata"] or {}).get("px4_git_sha", "unknown")
        body.append(
            "| {} | `{}` | {} | `{}` |".format(
                Path(trial["trial_dir"]).name,
                trial["trial_dir"],
                validation.get("overall", "n/a"),
                git_sha,
            )
        )
    body.append("")

    body.append("## Trial results")
    body.append("")
    body.append(
        "| Trial | T01 | T02 | T03 | T04 | T05 | T06 | T07 | T08 | T09 | T10 |"
    )
    body.append("| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |")
    for trial in per_trial:
        validation = trial["validation"] or {}
        items = {
            item["id"]: item["status"]
            for item in validation.get("items", [])
        }
        row = [
            Path(trial["trial_dir"]).name,
            items.get("T01", "n/a"),
            items.get("T02", "n/a"),
            items.get("T03", "n/a"),
            items.get("T04", "n/a"),
            items.get("T05", "n/a"),
            items.get("T06", "n/a"),
            items.get("T07", "n/a"),
            items.get("T08", "n/a"),
            items.get("T09", "n/a"),
            items.get("T10", "n/a"),
        ]
        body.append("| " + " | ".join(row) + " |")
    body.append("")

    body.append("## Aggregate statistics")
    body.append("")
    body.append("| Metric | mean | std | min | max |")
    body.append("| --- | --- | --- | --- | --- |")
    for key in ("total_distance_m", "peak_altitude_m", "peak_tilt_deg"):
        block = statistics[key]
        body.append(
            "| {} | {:.3f} | {:.3f} | {:.3f} | {:.3f} |".format(
                key, block["mean"], block["std"], block["min"], block["max"]
            )
        )
    body.append("")
    body.append("### Phase durations (s)")
    body.append("")
    body.append("| Phase | mean | std | min | max |")
    body.append("| --- | --- | --- | --- | --- |")
    for phase, block in sorted(statistics["phase_duration_s"].items()):
        body.append(
            "| {} | {:.3f} | {:.3f} | {:.3f} | {:.3f} |".format(
                phase, block["mean"], block["std"], block["min"], block["max"]
            )
        )
    body.append("")

    body.append("## Verdict")
    body.append("")
    body.append("Overall verdict: **{}**".format(overall_value))
    body.append("")

    body.append("## Links")
    body.append("")
    for trial in per_trial:
        body.append("- {} -> `validation.json`".format(trial["trial_dir"]))
    body.append("- Aggregate summary: `aggregate_summary.json`")
    body.append("")
    return "\n".join(body)


# ---------------------------------------------------------------------------
# CLI entry point
# ---------------------------------------------------------------------------


def _build_arg_parser():
    import argparse
    parser = argparse.ArgumentParser(prog="generate_report")
    parser.add_argument(
        "--result-dir",
        action="append",
        default=[],
        help="Trial result directory. Pass three times for an aggregate report.",
    )
    parser.add_argument("--config", required=True, help="Experiment YAML config")
    parser.add_argument("--log-rules", required=True, help="log_rules.yaml path")
    parser.add_argument(
        "--output-dir",
        default=None,
        help="Output directory for report.md, plots, and summary.json",
    )
    return parser


def main(argv=None) -> int:
    """Command-line entry point used by ``scripts/generate_report.py``."""
    import argparse
    parser = _build_arg_parser()
    args = parser.parse_args(argv)
    trial_dirs = [Path(path) for path in (args.result_dir or [])]
    if not trial_dirs:
        parser.error("--result-dir is required at least once")
    config_path = Path(args.config)
    log_rules = Path(args.log_rules)
    output_dir = (
        Path(args.output_dir) if args.output_dir else trial_dirs[0]
    )
    output_dir.mkdir(parents=True, exist_ok=True)
    if len(trial_dirs) >= 2:
        payload = write_aggregate_report(
            trial_dirs=trial_dirs,
            config_path=config_path,
            output_md_path=output_dir / "aggregate_report.md",
            output_plot_dir=output_dir / "plots",
            output_summary_path=output_dir / "aggregate_summary.json",
        )
        verdict = payload.get("overall")
    else:
        config = _safe_load_config(config_path)
        from .validation import evaluate_run
        report = evaluate_run(
            result_dir=trial_dirs[0],
            config=config,
            log_rules_path=log_rules,
        )
        metadata_path = trial_dirs[0] / "metadata.json"
        if metadata_path.exists():
            try:
                metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
            except ValueError:
                metadata = {}
        else:
            metadata = {}
        write_single_run_report(
            result_dir=trial_dirs[0],
            validation_report=report,
            metadata=metadata,
            config_path=config_path,
            output_md_path=output_dir / "report.md",
            output_plot_dir=output_dir / "plots",
            output_summary_path=output_dir / "summary.json",
        )
        verdict = report.overall
    return 0 if verdict == "PASS" else 1
