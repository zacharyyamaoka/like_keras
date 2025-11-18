#!/usr/bin/env python3

"""
    Tests for port system.
"""

# BAM
from lk.common.port import Port, InputPort, OutputPort, PortCollection
from lk.msgs.msg import Msg, Observation, Action

# PYTHON
import pytest


def test_port_creation():
    """Test basic port creation."""
    port = Port("test_port", Observation)
    
    assert port.name == "test_port"
    assert port.msg_type == Observation
    assert port.value is None


def test_port_type_validation():
    """Test port type validation."""
    port = Port("test_port", Observation)
    
    # Valid assignment
    obs = Observation(data=[1, 2, 3])
    port.value = obs
    assert port.value == obs
    
    # Invalid assignment
    with pytest.raises(TypeError):
        port.value = "not an observation"


def test_port_invalid_type():
    """Test that non-Msg types are rejected."""
    with pytest.raises(TypeError):
        Port("test_port", str)


def test_input_port():
    """Test InputPort functionality."""
    port = InputPort("obs", Observation)
    
    assert isinstance(port, Port)
    obs = Observation(data=[1, 2, 3])
    port.value = obs
    
    assert port.read() == obs


def test_output_port():
    """Test OutputPort functionality."""
    port = OutputPort("action", Action)
    
    assert isinstance(port, Port)
    action = Action(data=0.5)
    port.write(action)
    
    assert port.value == action


def test_port_collection():
    """Test PortCollection functionality."""
    collection = PortCollection()
    
    obs_port = InputPort("obs", Observation)
    action_port = OutputPort("action", Action)
    
    collection.add_port("obs", obs_port)
    collection.add_port("action", action_port)
    
    assert len(collection) == 2
    assert collection.get_port("obs") == obs_port
    assert collection.get_port("action") == action_port
    
    # Test iteration
    ports = list(collection)
    assert len(ports) == 2


if __name__ == "__main__":
    pytest.main([__file__, '-v'])

