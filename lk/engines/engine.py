#!/usr/bin/env python3

"""
    Abstract execution engine interface.
    
    Engines compile like_keras Systems into backend-specific configurations
    and handle execution via native tooling (Dora CLI, ROS2 launch, etc.).
"""

# PYTHON
from abc import ABC, abstractmethod
from pathlib import Path
from typing import Optional, TYPE_CHECKING

if TYPE_CHECKING:
    from lk.common.system import System


class Engine(ABC):
    """
    Abstract execution engine interface.
    
    Engines are responsible for:
    1. Compiling System graphs to backend-specific configs
    2. Launching systems using backend tooling
    3. Validating system compatibility
    
    Like Keras compiling to TensorFlow/PyTorch backends,
    like_keras compiles to Dora/ROS2/Native backends.
    """
    
    @abstractmethod
    def compile(self, system: "System") -> Optional[Path]:
        """
        Compile system to engine-specific configuration.
        
        Generates configuration files (YAML, launch files, etc.)
        needed to run the system on this engine.
        
        Args:
            system: System to compile
            
        Returns:
            Path to generated config file (or None for native)
        """
        pass
    
    @abstractmethod
    def launch(self, system: "System", config_path: Optional[Path] = None, **kwargs):
        """
        Launch system using engine's native tooling.
        
        For engines like Dora/ROS2, this invokes CLI commands.
        For native engine, this runs the system in-process.
        
        Args:
            system: System to launch
            config_path: Path to compiled config (from compile())
            **kwargs: Engine-specific launch options
        """
        pass
    
    @abstractmethod
    def validate(self, system: "System") -> tuple[bool, list[str]]:
        """
        Validate that system is compatible with this engine.
        
        Checks for:
        - Unsupported features
        - Missing dependencies
        - Configuration issues
        
        Args:
            system: System to validate
            
        Returns:
            Tuple of (is_valid, list_of_issues)
        """
        pass
    
    def __repr__(self) -> str:
        return f"{self.__class__.__name__}()"

