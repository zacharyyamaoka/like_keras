"""
SRDF asset metadata for robot semantic descriptions.
"""

# BAM

# PYTHON
from dataclasses import dataclass


@dataclass
class SrdfInfo:
    path: str = ""
    xacro_path: str = ""
    macro_path: str = ""
    loaded_from_cache: bool = False
    newly_generated: bool = False
    num_samples: int = 1000

    force_regenerate: bool = False

    def srdf_paths_exist(self) -> bool:
        return self.path or self.xacro_path

    @staticmethod
    def combine(srdf_infos: list["SrdfInfo"]) -> "SrdfInfo":

        return SrdfInfo()

    def extend(self, srdf_infos: "SrdfInfo") -> "SrdfInfo":
        return SrdfInfo.combine([self, *srdf_infos])

    def get_xml(self, xacro_args: dict = {}, resolve_packages: bool = False) -> str:
        if self.path:
            with open(self.path, "r") as f:
                return f.read()
        elif self.xacro_path:
            from bam.utils import (
                xml_from_xacro,
            )  # Lazy import keeps message modules lightweight

            return xml_from_xacro(self.xacro_path, xacro_args, resolve_packages=False)
        else:
            raise ValueError("No SRDF path or xacro path set")

    def get_path(self) -> str:
        return self.path


if __name__ == "__main__":
    srdf_info = SrdfInfo(path="robot.srdf")
    print(srdf_info)
