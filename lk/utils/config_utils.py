#!/usr/bin/env python3

"""
    Configuration utilities for dataclass-based configs.
    
    Provides:
    - Dict to dataclass conversion (using dacite)
    - Dataclass to dict conversion
    - Support for nested configs
    - Support for flat dot-separated configs
"""

# PYTHON
from dataclasses import asdict, is_dataclass, fields
from typing import TypeVar, Type, Any, get_type_hints
from dacite import from_dict, Config as DaciteConfig

T = TypeVar('T')


def to_config_dict(obj: Any, flat: bool = False, sep: str = ".") -> dict:
    """
    Convert dataclass to dictionary.
    
    Args:
        obj: Dataclass instance to convert
        flat: If True, return flat dict with dot-separated keys
        sep: Separator for flat keys (default: ".")
        
    Returns:
        Dictionary representation
    """
    if not is_dataclass(obj):
        raise ValueError(f"Object must be a dataclass, got {type(obj)}")
    
    result = asdict(obj)
    
    if flat:
        from bam.utils.dot_dict_utils import dict_to_dot_dict
        result = dict_to_dot_dict(result, sep=sep)
    
    return result


def from_config_dict(
    config_class: Type[T],
    config_dict: dict,
    is_flat: bool = False,
    sep: str = "."
) -> T:
    """
    Reconstruct dataclass from dictionary using dacite.
    
    Args:
        config_class: Target dataclass type
        config_dict: Dictionary to convert (can be nested or flat)
        is_flat: If True, convert from flat dot-separated dict first
        sep: Separator for flat keys (default: ".")
        
    Returns:
        Instance of config_class
        
    Example:
        >>> @dataclass
        >>> class MyConfig:
        >>>     value: int = 10
        >>> 
        >>> # From nested dict
        >>> cfg = from_config_dict(MyConfig, {"value": 20})
        >>> 
        >>> # From flat dict
        >>> cfg = from_config_dict(MyConfig, {"sub.value": 20}, is_flat=True)
    """
    if is_flat:
        from bam.utils.dot_dict_utils import dot_dict_to_dict
        config_dict = dot_dict_to_dict(config_dict, sep=sep)
    
    # Use dacite to handle the conversion
    # Config tells dacite to be flexible with extra keys and type checking
    return from_dict(
        data_class=config_class,
        data=config_dict,
        config=DaciteConfig(
            check_types=False,  # Be flexible with types
            strict=False,  # Allow extra keys in dict
        )
    )


def reconstruct_node_config(
    config_dict: dict,
    config_types: dict[str, Type],
    is_flat: bool = False,
    sep: str = "."
) -> dict:
    """
    Reconstruct all component configs in a node config dict.
    
    Args:
        config_dict: Dictionary mapping component_id -> config_dict
        config_types: Dictionary mapping component_id -> config dataclass type
        is_flat: If True, config_dict is flat with dot-separated keys
        sep: Separator for flat keys (default: ".")
        
    Returns:
        Dictionary mapping component_id -> config dataclass instance
        
    Example:
        >>> config_dict = {
        ...     'actor_node_actor': {'policy_type': 'default'},
        ...     'actor_node_worldmodel': {'mode': 'local'},
        ... }
        >>> config_types = {
        ...     'actor_node_actor': ActorConfig,
        ...     'actor_node_worldmodel': WorldModelConfig,
        ... }
        >>> node_config = reconstruct_node_config(config_dict, config_types)
    """
    if is_flat:
        # First convert flat dict to nested dict
        from bam.utils.dot_dict_utils import dot_dict_to_dict
        config_dict = dot_dict_to_dict(config_dict, sep=sep)
    
    # Now reconstruct each component config
    result = {}
    for component_id, component_dict in config_dict.items():
        if component_id in config_types:
            # Reconstruct using the registered type
            config_class = config_types[component_id]
            result[component_id] = from_config_dict(
                config_class,
                component_dict,
                is_flat=False  # Already converted above
            )
        else:
            # No registered type, keep as dict
            result[component_id] = component_dict
    
    return result


class ConfigMixin:
    """
    Mixin for dataclasses to provide serialization/deserialization.
    
    Usage:
        @dataclass
        class MyConfig(ConfigMixin):
            value: int = 10
            
        # Serialize
        cfg = MyConfig(value=20)
        d = cfg.to_dict()  # {'value': 20}
        flat_d = cfg.to_dict(flat=True)  # {'value': 20}
        
        # Deserialize
        cfg2 = MyConfig.from_dict(d)
    """
    
    def to_dict(self, flat: bool = False, sep: str = ".") -> dict:
        """Convert to dictionary."""
        return to_config_dict(self, flat=flat, sep=sep)
    
    @classmethod
    def from_dict(
        cls: Type[T],
        config_dict: dict,
        is_flat: bool = False,
        sep: str = "."
    ) -> T:
        """Reconstruct from dictionary."""
        return from_config_dict(cls, config_dict, is_flat=is_flat, sep=sep)


if __name__ == "__main__":
    from dataclasses import dataclass
    
    @dataclass
    class SubConfig(ConfigMixin):
        value: int = 10
        name: str = "test"
    
    @dataclass
    class MainConfig(ConfigMixin):
        mode: str = "local"
        sub: SubConfig = None
    
    # Test nested
    print("=== Nested Config ===")
    cfg = MainConfig(mode="remote", sub=SubConfig(value=20, name="hello"))
    d = cfg.to_dict()
    print(f"To dict: {d}")
    
    cfg2 = MainConfig.from_dict(d)
    print(f"From dict: {cfg2}")
    
    # Test flat
    print("\n=== Flat Config ===")
    flat_d = cfg.to_dict(flat=True)
    print(f"To flat dict: {flat_d}")
    
    cfg3 = MainConfig.from_dict(flat_d, is_flat=True)
    print(f"From flat dict: {cfg3}")
    
    # Test node config reconstruction
    print("\n=== Node Config Reconstruction ===")
    try:
        from bam.actor import ActorConfig
        from bam.world_model import WorldModelConfig
        
        config_dict = {
            'actor_node_actor': {'policy_type': 'default'},
            'actor_node_worldmodel': {'mode': 'local'},
        }
        
        config_types = {
            'actor_node_actor': ActorConfig,
            'actor_node_worldmodel': WorldModelConfig,
        }
        
        node_config = reconstruct_node_config(config_dict, config_types)
        print(f"Reconstructed: {node_config}")
        
        # Test flat node config
        print("\n=== Flat Node Config ===")
        flat_config = {
            'actor_node_actor.policy_type': 'default',
            'actor_node_worldmodel.mode': 'local',
        }
        
        node_config2 = reconstruct_node_config(flat_config, config_types, is_flat=True)
        print(f"Reconstructed from flat: {node_config2}")
    except ImportError as e:
        print(f"Skipping node config test (missing dependencies): {e}")

