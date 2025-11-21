"""
URDF asset metadata for robot descriptions.

Ok the challenge is that when you start to combine urdfs, then there could be multiple mesh_package_directories...
Pinoochio seems to be able to manage multiple directories... I think perhaps we can make this hold

In the most generic case, a URDF case hold multiple mesh_dir paths for the different meshes... but how to correspond them?
Potetially I can just do a matching scheme? An alternative is to specify the absolute path for each package... that would require a dictionary...? but I think is also much more exact...
"""

# BAM

# PYTHON
from dataclasses import dataclass, field


@dataclass
class ConfigFileInfo:
    dir: str = ""
    name: str = ""
    path: str = ""
    use_hash: bool = False


if __name__ == "__main__":
    config_file_info = ConfigFileInfo(
        dir="config", name="config.yaml", path="config/config.yaml"
    )
    print(config_file_info)
