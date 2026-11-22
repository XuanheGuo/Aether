"""Open the MuJoCo managed viewer with the position actuator controls."""
from pathlib import Path

import mujoco.viewer


if __name__ == "__main__":
    model_path = Path(__file__).resolve().parent / "model" / "aether_v1.xml"
    mujoco.viewer.launch_from_path(str(model_path))
