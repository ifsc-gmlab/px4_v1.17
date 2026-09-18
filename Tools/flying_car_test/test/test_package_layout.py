from pathlib import Path
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]

def test_manifest_and_python_package_agree():
    manifest = ET.parse(ROOT / "package.xml").getroot()
    assert manifest.findtext("name") == "flying_car_test"
    assert (ROOT / "src/flying_car_test/__init__.py").is_file()

def test_workspace_script_refuses_non_symlink_collision():
    text = (ROOT / "scripts/setup_workspace.sh").read_text()
    assert 'FLYING_CAR_TEST_WS:-/home/px4/flying_car_test_ws' in text
    assert 'existing package path is not the expected symlink' in text
