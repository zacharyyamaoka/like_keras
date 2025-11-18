#!/usr/bin/env python3

"""
    Component system with lifecycle management.
    
    Components are the building blocks of the system.
    They have input/output ports and optional lifecycle hooks.
"""

# BAM
from lk.common.port import InputPort, OutputPort, PortCollection

# PYTHON
from typing import Optional, Dict, Any
from enum import Enum
from abc import ABC
from dataclasses import dataclass


class LifecycleState(Enum):
    """
        Component lifecycle states (inspired by ROS2).
    """
    UNCONFIGURED = "unconfigured"
    INACTIVE = "inactive"
    ACTIVE = "active"
    FINALIZED = "finalized"


class Component(ABC):
    """
        Base component without lifecycle management.
        
        Simple stateless components can inherit from this.
        For stateful components with setup/teardown, use LifecycleComponent.
        
        Each Component subclass can define its own Config class:
        
        class MyComponent(Component):
            @dataclass
            class Config:
                param1: int = 10
                param2: str = "default"
            
            def __init__(self, config: Optional[Config] = None, **kwargs):
                super().__init__(config=config, **kwargs)
    """
    
    @dataclass
    class Config:
        """
            Base configuration class.
            
            Subclasses should override this with their own Config.
        """
        pass
    
    def __init__(self, 
                 name: Optional[str] = None, 
                 node: Optional['Node'] = None,
                 config: Optional['Config'] = None):
        """
            Initialize component.
            
            Args:
                name: Component identifier
                node: Node this component belongs to
                config: Component configuration
        """
        self.name = name or self.__class__.__name__
        self.node = node
        self.config = config if config is not None else self.__class__.Config()
        
        # Port collections
        self.inputs = PortCollection(owner=self)
        self.outputs = PortCollection(owner=self)
        
        # Auto-discover ports defined in subclasses
        self._discover_ports()
    
    @property
    def in_(self):
        """Shorthand for self.inputs"""
        return self.inputs
    
    @property
    def out(self):
        """Shorthand for self.outputs"""
        return self.outputs
    
    def _discover_ports(self):
        """
            Automatically discover InputPort and OutputPort attributes.
            
            Subclasses can define ports as class attributes, and they
            will be automatically added to inputs/outputs collections.
        """
        for attr_name in dir(self):
            if attr_name.startswith('_'):
                continue
            
            try:
                attr = getattr(self, attr_name)
                if isinstance(attr, InputPort):
                    self.inputs.add_port(attr_name, attr)
                    attr.owner = self
                elif isinstance(attr, OutputPort):
                    self.outputs.add_port(attr_name, attr)
                    attr.owner = self
            except AttributeError:
                continue
    
    def __call__(self, *args, **kwargs):
        """
            PyTorch-like callable interface.
            
            Subclasses can override to define behavior.
            This enables syntax like: output = component(input)
        """
        # Default: forward to forward() method if it exists
        if hasattr(self, 'forward'):
            return self.forward(*args, **kwargs)
        raise NotImplementedError(
            f"{self.__class__.__name__} must implement __call__ or forward()"
        )
    
    def __repr__(self) -> str:
        return f"{self.__class__.__name__}(name={self.name})"


class ComponentLifecycleMixin:
    """
        Mixin that adds lifecycle management to components.
        
        Provides ROS2-style lifecycle hooks:
        - configure() - Initialize resources
        - activate() - Start execution
        - deactivate() - Pause execution
        - shutdown() - Release resources
        - destroy() - Final cleanup
    """
    
    def __init__(self, *args, **kwargs):
        """Initialize lifecycle state."""
        super().__init__(*args, **kwargs)
        self._lifecycle_state = LifecycleState.UNCONFIGURED
    
    @property
    def lifecycle_state(self) -> LifecycleState:
        """Get current lifecycle state."""
        return self._lifecycle_state
    
    def configure(self):
        """
            Configure the component.
            
            Called before activation. Initialize resources here.
            Transition: UNCONFIGURED -> INACTIVE
        """
        if self._lifecycle_state != LifecycleState.UNCONFIGURED:
            raise RuntimeError(
                f"Cannot configure from state {self._lifecycle_state}"
            )
        
        self.on_configure()
        self._lifecycle_state = LifecycleState.INACTIVE
    
    def activate(self):
        """
            Activate the component.
            
            Called to start execution. Call reset() to populate initial outputs.
            Transition: INACTIVE -> ACTIVE
        """
        if self._lifecycle_state != LifecycleState.INACTIVE:
            raise RuntimeError(
                f"Cannot activate from state {self._lifecycle_state}"
            )
        
        self.on_activate()
        self._lifecycle_state = LifecycleState.ACTIVE
    
    def deactivate(self):
        """
            Deactivate the component.
            
            Called to pause execution.
            Transition: ACTIVE -> INACTIVE
        """
        if self._lifecycle_state != LifecycleState.ACTIVE:
            raise RuntimeError(
                f"Cannot deactivate from state {self._lifecycle_state}"
            )
        
        self.on_deactivate()
        self._lifecycle_state = LifecycleState.INACTIVE
    
    def shutdown(self):
        """
            Shutdown the component.
            
            Called to release resources.
            Transition: INACTIVE -> UNCONFIGURED
        """
        if self._lifecycle_state != LifecycleState.INACTIVE:
            raise RuntimeError(
                f"Cannot shutdown from state {self._lifecycle_state}"
            )
        
        self.on_shutdown()
        self._lifecycle_state = LifecycleState.UNCONFIGURED
    
    def destroy(self):
        """
            Destroy the component.
            
            Final cleanup. Can be called from any state.
            Transition: * -> FINALIZED
        """
        self.on_destroy()
        self._lifecycle_state = LifecycleState.FINALIZED
    
    # Override these methods in subclasses
    def on_configure(self):
        """Override to implement configuration logic."""
        pass
    
    def on_activate(self):
        """Override to implement activation logic (e.g., call reset())."""
        pass
    
    def on_deactivate(self):
        """Override to implement deactivation logic."""
        pass
    
    def on_shutdown(self):
        """Override to implement shutdown logic."""
        pass
    
    def on_destroy(self):
        """Override to implement destroy logic."""
        pass


class LifecycleComponent(ComponentLifecycleMixin, Component):
    """
        Component with full lifecycle support.
        
        Convenience class combining Component and ComponentLifecycleMixin.
        Use this for components that need setup/teardown.
    """
    
    def __init__(self, 
                 name: Optional[str] = None, 
                 node: Optional['Node'] = None,
                 config: Optional['Component.Config'] = None):
        """
            Initialize lifecycle component.
            
            Args:
                name: Component identifier
                node: Node this component belongs to
                config: Component configuration
        """
        super().__init__(name=name, node=node, config=config)


# Import at end to avoid circular imports
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from lk.common.node import Node

