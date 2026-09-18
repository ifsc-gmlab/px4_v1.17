"""Static launch-file contracts for the flying-car MAVROS setup."""

import os
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

import yaml


REPOSITORY_ROOT = Path(__file__).resolve().parents[3]
FLYING_CAR_LAUNCH = REPOSITORY_ROOT / "launch" / "FS_flying_car.launch"
SITL_MAVROS_LAUNCH = (
    REPOSITORY_ROOT / "Tools" / "flying_car_test" / "launch"
    / "mavros_sitl_no_timesync.launch"
)
EXPERIMENT_LAUNCHES = (
    REPOSITORY_ROOT / "Tools/flying_car_test/launch/monitor_only.launch",
    REPOSITORY_ROOT / "Tools/flying_car_test/launch/full_experiment.launch",
)
ROS_NOETIC_PYTHON_PATH = "/opt/ros/noetic/lib/python3/dist-packages"


def launch_root(path):
    return ET.parse(path).getroot()


def arg_default(path, name):
    for argument in launch_root(path).findall("arg"):
        if argument.attrib.get("name") == name:
            return argument.attrib.get("default")
    return None


def launch_params(path):
    return {
        parameter.attrib["name"]: parameter.attrib["value"]
        for parameter in launch_root(path).iter("param")
        if "name" in parameter.attrib and "value" in parameter.attrib
    }


def test_default_launch_remains_unchanged():
    """Removing the override default would break normal MAVROS startup."""
    assert (
        arg_default(FLYING_CAR_LAUNCH, "mavros_launch")
        == "$(find mavros)/launch/px4.launch"
    )


def test_sitl_mavros_disables_only_time_traffic():
    """Wrong or missing no-timesync settings would add clock traffic to SITL."""
    assert SITL_MAVROS_LAUNCH.is_file()
    params = launch_params(SITL_MAVROS_LAUNCH)
    assert params["conn/timesync_rate"] == "0.0"
    assert params["conn/system_time_rate"] == "0.0"
    assert params["time/timesync_mode"] == "NONE"


def test_sitl_mavros_does_not_enable_lockstep():
    """Adding MAVROS lockstep would alter the simulation execution model."""
    assert SITL_MAVROS_LAUNCH.is_file()
    assert "enable_lockstep" not in SITL_MAVROS_LAUNCH.read_text()


def test_experiment_launches_pass_mapping_metadata_to_monitor():
    """String launch metadata would make the required monitor exit at startup."""
    package_paths = (
        REPOSITORY_ROOT,
        REPOSITORY_ROOT
        / "Tools/simulation/gazebo-classic/sitl_gazebo-classic",
        REPOSITORY_ROOT / "Tools/flying_car_test",
    )
    environment = os.environ.copy()
    inherited_paths = environment.get("ROS_PACKAGE_PATH", "")
    environment["ROS_PACKAGE_PATH"] = os.pathsep.join(
        [*(str(path) for path in package_paths), inherited_paths]
    )
    inherited_python_paths = environment.get("PYTHONPATH", "")
    environment["PYTHONPATH"] = os.pathsep.join(
        [inherited_python_paths, ROS_NOETIC_PYTHON_PATH]
    )

    for launch_file in EXPERIMENT_LAUNCHES:
        completed = subprocess.run(
            [
                "roslaunch",
                "--dump-params",
                str(launch_file),
                "gui:=false",
                "x:=0",
                "y:=30",
            ],
            check=True,
            capture_output=True,
            text=True,
            env=environment,
        )
        parameters = yaml.safe_load(completed.stdout)
        launch_arguments_raw = {
            key: value
            for key, value in parameters.items()
            if key.startswith("/flying_car_experiment_monitor/launch_arguments/")
        }
        # roslaunch --dump-params flattens rosparam mappings into dotted keys
        # and does not substitute $(arg ...) on all supported versions, so the
        # contract is: the metadata must be a YAML mapping (not a string param).
        assert launch_arguments_raw
        launch_arguments = {
            key.rsplit("/", 1)[1]: value
            for key, value in launch_arguments_raw.items()
        }
        assert set(launch_arguments) == {"gui", "x", "y"}
