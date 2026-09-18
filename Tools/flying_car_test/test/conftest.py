from pathlib import Path

import pytest


@pytest.fixture
def config_path():
    """Return the experiment configuration independently of the test CWD."""
    return Path(__file__).resolve().parents[1] / "config" / "experiment.yaml"
