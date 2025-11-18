#!/usr/bin/env python3

"""
    Tests for component system.
"""

# BAM
from lk.common.component import Component, LifecycleComponent, ComponentLifecycleMixin, LifecycleState
from lk.common.port import InputPort, OutputPort
from lk.common.node import Node
from lk.msgs.msg import Observation, Action

# PYTHON
import pytest


class SimpleComponent(Component):
    """Test component without lifecycle."""
    
    def __init__(self):
        super().__init__(name="simple")
        self.obs = InputPort("obs", Observation, owner=self)
        self.action = OutputPort("action", Action, owner=self)
        self._discover_ports()


class LifecycleTestComponent(LifecycleComponent):
    """Test component with lifecycle."""
    
    def __init__(self):
        super().__init__(name="lifecycle_test")
        self.configured = False
        self.activated = False
        self.deactivated = False
        self.shutdown_called = False
    
    def on_configure(self):
        self.configured = True
    
    def on_activate(self):
        self.activated = True
    
    def on_deactivate(self):
        self.deactivated = True
    
    def on_shutdown(self):
        self.shutdown_called = True


def test_simple_component():
    """Test basic component creation."""
    comp = SimpleComponent()
    
    assert comp.name == "simple"
    assert len(comp.inputs.all_ports()) == 1
    assert len(comp.outputs.all_ports()) == 1


def test_component_port_discovery():
    """Test automatic port discovery."""
    comp = SimpleComponent()
    
    assert comp.inputs.get_port("obs") is not None
    assert comp.outputs.get_port("action") is not None


def test_lifecycle_states():
    """Test lifecycle state transitions."""
    comp = LifecycleTestComponent()
    
    # Initial state
    assert comp.lifecycle_state == LifecycleState.UNCONFIGURED
    assert not comp.configured
    
    # Configure
    comp.configure()
    assert comp.lifecycle_state == LifecycleState.INACTIVE
    assert comp.configured
    
    # Activate
    comp.activate()
    assert comp.lifecycle_state == LifecycleState.ACTIVE
    assert comp.activated
    
    # Deactivate
    comp.deactivate()
    assert comp.lifecycle_state == LifecycleState.INACTIVE
    assert comp.deactivated
    
    # Shutdown
    comp.shutdown()
    assert comp.lifecycle_state == LifecycleState.UNCONFIGURED
    assert comp.shutdown_called


def test_lifecycle_invalid_transitions():
    """Test that invalid transitions raise errors."""
    comp = LifecycleTestComponent()
    
    # Can't activate before configure
    with pytest.raises(RuntimeError):
        comp.activate()
    
    # Can't deactivate before activate
    comp.configure()
    with pytest.raises(RuntimeError):
        comp.deactivate()


def test_component_with_node():
    """Test component attached to node."""
    node = Node("test_node")
    comp = Component(name="test_comp", node=node)
    
    assert comp.node == node


if __name__ == "__main__":
    pytest.main([__file__, '-v'])

