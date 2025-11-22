#!/usr/bin/env python3

"""
    Execution engines for like_keras.
    
    Engines compile System graphs to different execution backends:
    - native: Pure Python in-process execution
    - dora: Dora-rs dataflow system  
    - ros2: ROS2 launch system (future)
"""

# BAM
from lk.engines.engine import Engine

# PYTHON
from typing import Dict, Type

# Engine registry
_ENGINE_REGISTRY: Dict[str, Type[Engine]] = {}


def register_engine(name: str, engine_class: Type[Engine]):
    """
    Register an execution engine.
    
    Args:
        name: Engine identifier ('native', 'dora', 'ros2', etc.)
        engine_class: Engine class
    """
    _ENGINE_REGISTRY[name] = engine_class


def get_engine(name: str) -> Engine:
    """
    Get an execution engine by name.
    
    Args:
        name: Engine identifier
        
    Returns:
        Engine instance
        
    Raises:
        ValueError: If engine not found
    """
    if name not in _ENGINE_REGISTRY:
        available = ', '.join(_ENGINE_REGISTRY.keys())
        raise ValueError(
            f"Engine '{name}' not found. "
            f"Available engines: {available}"
        )
    
    engine_class = _ENGINE_REGISTRY[name]
    return engine_class()


def list_engines() -> list[str]:
    """
    List available engines.
    
    Returns:
        List of engine names
    """
    return list(_ENGINE_REGISTRY.keys())


# Import engines to trigger registration
# (Will be implemented in next steps)
try:
    from lk.engines.native import NativeEngine
    register_engine('native', NativeEngine)
except ImportError:
    pass

try:
    from lk.engines.dora import DoraEngine
    register_engine('dora', DoraEngine)
except ImportError:
    pass

try:
    from lk.engines.ros2 import ROS2Engine
    register_engine('ros2', ROS2Engine)
except ImportError:
    pass


__all__ = ['Engine', 'get_engine', 'register_engine', 'list_engines']

