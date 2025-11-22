#!/usr/bin/env python3

"""
    Backend Abstraction for Launch Systems
    
    like-keras is a high-level abstraction that compiles to different backends:
    - Dora RS (primary)
    - ROS2 (secondary)
    - Custom (future)
    
    Philosophy (like Keras):
    - Keras doesn't implement tensor ops → uses TensorFlow/PyTorch/JAX
    - like-keras doesn't implement launch/IPC → uses Dora/ROS2
    
    Workflow:
    1. Define system in Python (Components, Nodes, Connections)
    2. Compile to backend format (dataflow.yml, launch.py, etc.)
    3. Launch using backend tools (dora start, ros2 launch, etc.)
"""

# BAM
from lk.common.system import System
from lk.common.node import Node
from lk.common.component import Component
from lk.common.graph import Connection

# PYTHON
from typing import Optional, Any
from dataclasses import dataclass
from enum import Enum
from abc import ABC, abstractmethod
from pathlib import Path
import subprocess


class BackendType(Enum):
    """Supported backend types."""
    
    DORA_RS = "dora_rs"  # Dora RS (https://dora-rs.ai)
    ROS2 = "ros2"  # ROS2 (https://ros.org)
    NATIVE = "native"  # Pure Python (same process, no external tools)


@dataclass
class BackendConfig:
    """Configuration for backend compilation."""
    
    backend_type: BackendType = BackendType.DORA_RS
    
    # Output paths
    output_dir: Optional[Path] = None  # Where to write generated files
    output_file: Optional[str] = None  # Specific filename
    
    # Dora-specific
    dora_coordinator_addr: str = "127.0.0.1:8080"
    
    # ROS2-specific
    ros2_package_name: Optional[str] = None
    
    # Runtime options
    auto_launch: bool = True  # Automatically launch after compilation
    verbose: bool = True


class Backend(ABC):
    """
        Base class for backend compilers.
        
        Each backend implements:
        1. compile() - Convert System to backend format
        2. launch() - Execute the compiled system
        3. shutdown() - Stop the system
    """
    
    def __init__(self, config: BackendConfig):
        """Initialize backend with configuration."""
        self.config = config
    
    @abstractmethod
    def compile(self, system: System) -> Path:
        """
            Compile system to backend-specific format.
            
            Args:
                system: System to compile
                
            Returns:
                Path to generated file(s)
        """
        pass
    
    @abstractmethod
    def launch(self, compiled_path: Path) -> subprocess.Popen:
        """
            Launch the compiled system using backend tools.
            
            Args:
                compiled_path: Path to compiled files
                
            Returns:
                Process handle for the launched system
        """
        pass
    
    @abstractmethod
    def shutdown(self, process: subprocess.Popen):
        """
            Shutdown the running system.
            
            Args:
                process: Process handle from launch()
        """
        pass
    
    def compile_and_launch(self, system: System) -> subprocess.Popen:
        """
            Convenience method: compile and launch in one step.
            
            Args:
                system: System to compile and launch
                
            Returns:
                Process handle
        """
        compiled_path = self.compile(system)
        return self.launch(compiled_path)


class NativeBackend(Backend):
    """
        Native Python backend (no external tools).
        
        Runs everything in the same process, uses direct Python calls.
        Good for development and debugging.
    """
    
    def compile(self, system: System) -> Path:
        """No compilation needed for native backend."""
        if self.config.verbose:
            print(f"[NativeBackend] Using native Python execution (no compilation)")
        return Path(".")
    
    def launch(self, compiled_path: Path) -> subprocess.Popen:
        """Run system directly (not a subprocess)."""
        if self.config.verbose:
            print(f"[NativeBackend] Launching system in same process")
        # Return None since we don't spawn a process
        return None
    
    def shutdown(self, process: subprocess.Popen):
        """Shutdown native system."""
        if self.config.verbose:
            print(f"[NativeBackend] Shutting down")


def create_backend(backend_type: BackendType, config: Optional[BackendConfig] = None) -> Backend:
    """
        Factory function to create backend instances.
        
        Args:
            backend_type: Type of backend to create
            config: Backend configuration
            
        Returns:
            Backend instance
    """
    if config is None:
        config = BackendConfig(backend_type=backend_type)
    
    if backend_type == BackendType.NATIVE:
        return NativeBackend(config)
    elif backend_type == BackendType.DORA_RS:
        from lk.common.backends.dora_backend import DoraBackend
        return DoraBackend(config)
    elif backend_type == BackendType.ROS2:
        from lk.common.backends.ros2_backend import ROS2Backend
        return ROS2Backend(config)
    else:
        raise ValueError(f"Unknown backend type: {backend_type}")


if __name__ == "__main__":
    # Example usage
    
    print("Backend Abstraction System")
    print("=" * 70)
    
    # Native backend
    config_native = BackendConfig(backend_type=BackendType.NATIVE)
    backend_native = create_backend(BackendType.NATIVE, config_native)
    print(f"Native backend: {backend_native}")
    
    # Dora backend (will import when implemented)
    print(f"\nDora RS backend: {BackendType.DORA_RS}")
    print("  → Compiles to dataflow.yml")
    print("  → Launches with: dora start dataflow.yml")
    
    # ROS2 backend
    print(f"\nROS2 backend: {BackendType.ROS2}")
    print("  → Compiles to launch.py")
    print("  → Launches with: ros2 launch pkg launch.py")

