


from .description_types import SrdfInfo, UrdfInfo, ConfigFileInfo


def load_srdf(srdf: SrdfInfo, urdf: UrdfInfo, config_file: ConfigFileInfo, verbose: bool = False) -> bool:
    
    if srdf.force_regenerate:
        # Force regeneration - always generate new SRDF
        generate_new_srdf(verbose=verbose)

    elif srdf.srdf_paths_exist():
        # SRDF already specified, do nothing
        pass
    elif load_srdf_path_from_cache(srdf):
        # Found cached SRDF, path assigned
        pass
    else:
        # Generate new SRDF
        generate_new_srdf(verbose=verbose)

    if not srdf.srdf_paths_exist():
        raise RuntimeError("SRDF path is empty after generation attempt")



def get_srdf_cache_path(config_file: ConfigFileInfo) -> Path:
    """Get the path for a cached SRDF file.
    
    Returns:
        Path to cached SRDF file (may not exist yet)
    """
    cache_dir = Path(self.config_file.dir) / "srdf_cache"
    cache_dir.mkdir(parents=True, exist_ok=True)
    
    # config_hash = get_file_name_hash(self.config_file_name, length=12)
    # filename = f"{self.info.name}_{config_hash}.srdf"
    filename = f"{self.config_file.name}.srdf"

    return cache_dir / filename

def load_srdf_path_from_cache(self) -> bool:
    """Check if cached SRDF exists and assign to self.srdf.path if found.
    
    Returns:
        True if cached SRDF found and path assigned, False otherwise
    """
    cached_path = self._get_srdf_cache_path()
    
    if cached_path.exists():
        self.srdf.path = str(cached_path)
        self.srdf.loaded_from_cache = True
        self.srdf.newly_generated = False
        return True
    
    return False

# Reduce from 100,000 to 10,000 for dev so faster, but in reality likey want to use more!
def generate_new_srdf(self, num_samples: int = 10000, verbose: bool = False):
    """Generate new SRDF from URDF and save to cache.
    
    DescriptionArgs:
        num_samples: Number of random configurations to sample for collision detection
        verbose: Print progress information
    """
    if verbose:
        print(f"Generating SRDF for {self.info.name}...")
    
    # Get paths
    srdf_save_path = self._get_srdf_cache_path()
    
    # Generate SRDF using pin_utils

    #BUG: Make sure you dump to file, as new SRDF will be generated and requires a urdf that requires the config file.
    self.dump_to_file(verbose=False)
    
    with self.get_temp_urdf_path() as urdf_path:
        generate_srdf_from_urdf(
            urdf_path=urdf_path,
            mesh_package_dirs=self.urdf.abs_package_dirs,
            srdf_save_path=str(srdf_save_path),
            num_samples=num_samples,
            verbose=verbose
        )
    
    # Assign path
    self.srdf.path = str(srdf_save_path)
    self.srdf.newly_generated = True
    self.srdf.loaded_from_cache = False
    
    if verbose:
        print(f"Saved SRDF to: {srdf_save_path}")