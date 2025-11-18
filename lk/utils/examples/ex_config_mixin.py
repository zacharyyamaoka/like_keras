from dataclasses import dataclass
from bam.utils.config_mixin import ConfigMixin

@dataclass
class CameraConfig(ConfigMixin):
    resolution: list[int, int] = (1920, 1080)
    fps: int = 30
    color: bool = True


# Usage
import os
path = os.path.join(os.path.dirname(__file__), "config_test.json")
cfg = CameraConfig()
cfg.save_json_file(path)
cfg.save_yaml_file(path.replace(".json", ".yaml"))

loaded = CameraConfig.from_json_file(path)
print(loaded._metadata)  # {'version': '1.0', 'created_by': 'ConfigBase'}

