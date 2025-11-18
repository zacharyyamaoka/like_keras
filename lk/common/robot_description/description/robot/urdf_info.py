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



# Hold Header and Foot and Str, it could be callable, but then you cannot store to config file
@dataclass
class UrdfInfo:
    path: str = ""
    xacro_path: str = ""
    xacro_imports: list[str] = field(default_factory=list)
    macro_path: str = ""
    abs_package_dirs: dict[str, str] = field(default_factory=dict)

    xml_header: str = (
        '<?xml version="1.0"?>\n'
        '<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="composite">\n'
    )
    xml_footer: str = '</robot>\n'


    def urdf_paths_exist(self) -> bool:
        return self.path or self.xacro_path

    def get_macro_xml(self, config_file_path: str) -> str:
        if not self.macro_path:
            raise ValueError("No macro path set")

        xml = (
            f'  <xacro:include filename="{self.macro_path}"/>\n'
            f'  <xacro:make_config config_file="{config_file_path}"/>\n'
        )

        return xml

    def get_xml_body(self, config_file_path: str, resolve_packages: bool = False) -> str:
        from bam.utils import xml_body_from_macro_xml  # Lazy import keeps message modules lightweight

        try:
            return xml_body_from_macro_xml(macro_xml=self.get_macro_xml(config_file_path), imports=self.xacro_imports, resolve_packages=resolve_packages)
        except Exception as e:
            # Print full error first, then config file path at the end for easy clicking
            import traceback
            error_msg = f"\n\n{'='*80}\n"
            error_msg += "Full error traceback:\n"
            error_msg += traceback.format_exc()
            error_msg += f"{'='*80}\n"
            error_msg += f"Config file path: {config_file_path}\n"
            error_msg += f"Macro path: {self.macro_path}\n"
            error_msg += f"Xacro imports: {self.xacro_imports}\n"
            error_msg += f"{'='*80}\n"
            
            raise type(e)(error_msg) from e

    def get_combined_xml(self, xml_body: str) -> str:

        return self.xml_header + xml_body + self.xml_footer
            

    # Defines the order of precedence for getting the xml
    def get_xml(self, xacro_args: dict = {}, config_file_path: str = "", resolve_packages: bool = False) -> str:

        # Very rarely used almost always we want to be dynamically generating...
        if self.path:
            with open(self.path, 'r') as f:
                return f.read()
        elif self.xacro_path:
            from bam.utils import xml_from_xacro  # Lazy import keeps message modules lightweight

            return xml_from_xacro(self.xacro_path, xacro_args, resolve_packages=False)

        else:
            # Read config_file_path from xacro_args (primary source) or fallback to parameter
            config_file_path = xacro_args.get('config_file', config_file_path)
            return self.get_combined_xml(self.get_xml_body(config_file_path, resolve_packages=resolve_packages))


    @staticmethod
    def combine(urdf_infos: list['UrdfInfo']) -> 'UrdfInfo':

        # the only thing that is relevant to the new urdf is the absolute package directories...

        assert len(urdf_infos) > 0, "At least one urdf_info must be provided"

        # Take the xml_header and xml_footer from the first urdf_info (if provided)
        xml_header = urdf_infos[0].xml_header
        xml_footer = urdf_infos[0].xml_footer

        abs_package_dirs = {}
        for urdf_info in urdf_infos:
            abs_package_dirs.update(urdf_info.abs_package_dirs)

        xacro_imports = []
        for urdf_info in urdf_infos:
            xacro_imports.extend(urdf_info.xacro_imports)

        return UrdfInfo(abs_package_dirs=abs_package_dirs, xml_header=xml_header, xml_footer=xml_footer, xacro_imports=xacro_imports)

    def extend(self, urdf_infos: list['UrdfInfo']) -> 'UrdfInfo':
        return UrdfInfo.combine([self, *urdf_infos])

    def get_path(self) -> str:
        return self.path

if __name__ == "__main__":
    urdf_info = UrdfInfo(path="robot.urdf", macro_path="/path/to/macro.xacro")
    print(urdf_info)


